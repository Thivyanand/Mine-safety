"""
routing.py — graph construction + shortest-path routing over a mine tunnel
point cloud. Deliberately Flask-agnostic: nothing in this file imports
Flask or touches request/response state, so it can be unit-tested or reused
standalone.

DSA notes (this is a course/hackathon demo and is meant to read well in a
viva):
  - MineGraph is an ADJACENCY LIST (dict[node_id] -> list[(neighbor, weight)]).
    O(1) average node lookup via a HASH MAP, O(degree) neighbor iteration.
  - build_graph_from_points turns an unordered 3D POINT CLOUD into a graph
    using a K-NEAREST-NEIGHBOR + distance-threshold heuristic (a cKDTree
    gives us O(N log N) neighbor queries instead of the naive O(N^2)).
  - Connectivity is checked/repaired with BFS over the adjacency list
    (O(V+E)); a SET tracks visited nodes.
  - AStarRouter.shortest_path uses A* with a binary MIN-HEAP (heapq),
    Euclidean 3D distance as an admissible heuristic, a hash map for
    best-known g-scores, and a SET for the closed frontier. This focuses
    the search toward the destination while still returning an optimal
    shortest path because edge weights are Euclidean distances.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Set, Tuple

Coord = Tuple[float, float, float]


def euclidean(a: Coord, b: Coord) -> float:
    """Straight-line 3D distance. Used only as an edge weight — real path
    length is the sum of edge weights along a route, not this function
    applied to endpoints."""
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


class MineGraph:
    """Adjacency-list graph over tunnel-walkable points.

    nodes:      hash map  node_id -> (x, y, z)
    adjacency:  hash map  node_id -> list[(neighbor_id, weight)]
    """

    def __init__(self) -> None:
        self.nodes: Dict[int, Coord] = {}
        self.adjacency: Dict[int, List[Tuple[int, float]]] = {}

    def add_node(self, node_id: int, coord: Coord) -> None:
        self.nodes[node_id] = coord
        self.adjacency.setdefault(node_id, [])

    def add_edge(self, a: int, b: int, weight: Optional[float] = None) -> None:
        if a == b:
            return
        if weight is None:
            weight = euclidean(self.nodes[a], self.nodes[b])
        if not self.has_edge(a, b):
            self.adjacency.setdefault(a, []).append((b, weight))
        if not self.has_edge(b, a):
            self.adjacency.setdefault(b, []).append((a, weight))

    def has_edge(self, a: int, b: int) -> bool:
        return any(n == b for n, _ in self.adjacency.get(a, []))

    def neighbors(self, node_id: int) -> List[Tuple[int, float]]:
        return self.adjacency.get(node_id, [])

    def num_nodes(self) -> int:
        return len(self.nodes)

    def num_edges(self) -> int:
        # each undirected edge is stored twice (once per endpoint)
        return sum(len(v) for v in self.adjacency.values()) // 2


def _connected_components(graph: MineGraph) -> List[Set[int]]:
    """BFS over the adjacency list to find connected components."""
    unvisited: Set[int] = set(graph.nodes.keys())
    components: List[Set[int]] = []
    while unvisited:
        start = next(iter(unvisited))
        component: Set[int] = set()
        queue = [start]
        unvisited.discard(start)
        while queue:
            cur = queue.pop()
            component.add(cur)
            for neighbor, _ in graph.neighbors(cur):
                if neighbor in unvisited:
                    unvisited.discard(neighbor)
                    queue.append(neighbor)
        components.append(component)
    return components


def is_connected(graph: MineGraph) -> bool:
    """Public helper — True if the graph is a single connected component."""
    if graph.num_nodes() == 0:
        return True
    return len(_connected_components(graph)) == 1


def _bridge_components(graph: MineGraph) -> None:
    """Repeatedly find the globally shortest edge between any two remaining
    components and add it, until everything is one component. Never adds
    an arbitrary/long "teleport" edge — always the shortest real edge
    available between the components at that point."""
    components = _connected_components(graph)
    while len(components) > 1:
        best = None  # (dist, node_a, node_b, comp_i, comp_j)
        # Merge the two closest components first, repeatedly — this keeps
        # each bridge as short as possible rather than picking arbitrarily.
        for i in range(len(components)):
            for j in range(i + 1, len(components)):
                comp_a, comp_b = components[i], components[j]
                for na in comp_a:
                    pa = graph.nodes[na]
                    for nb in comp_b:
                        d = euclidean(pa, graph.nodes[nb])
                        if best is None or d < best[0]:
                            best = (d, na, nb, i, j)
        dist, na, nb, i, j = best
        graph.add_edge(na, nb, dist)
        merged = components[i] | components[j]
        components = [c for k, c in enumerate(components) if k not in (i, j)]
        components.append(merged)


def build_graph_from_points(
    points: Iterable[Coord], k_neighbors: int, max_edge_distance: float
) -> Tuple[MineGraph, bool]:
    """Build a walkable graph from a raw, topology-free 3D point cloud.

    Strategy: connect each point to its k nearest neighbors, but only keep
    an edge if it's <= max_edge_distance (avoids bridging separate tunnel
    branches with an unrealistic long edge). Then check connectivity; if
    more than one component exists, bridge them with the shortest real
    edges available until the whole graph is connected.

    Returns (graph, was_fully_connected_before_bridging).
    """
    points = list(points)
    graph = MineGraph()
    for i, p in enumerate(points):
        graph.add_node(i, tuple(p))

    try:
        from scipy.spatial import cKDTree
        import numpy as np

        arr = np.asarray(points, dtype=float)
        tree = cKDTree(arr)
        k = min(k_neighbors + 1, len(points))  # +1: includes the point itself
        dists, idxs = tree.query(arr, k=k)
        if k == 1:
            dists = dists.reshape(-1, 1)
            idxs = idxs.reshape(-1, 1)
        for i in range(len(points)):
            for d, j in zip(dists[i], idxs[i]):
                j = int(j)
                if j == i:
                    continue
                if d <= max_edge_distance:
                    graph.add_edge(i, j, float(d))
    except ImportError:
        # Fallback: naive O(N^2) neighbor search if scipy isn't available.
        n = len(points)
        for i in range(n):
            dists = []
            for j in range(n):
                if i == j:
                    continue
                d = euclidean(graph.nodes[i], graph.nodes[j])
                dists.append((d, j))
            dists.sort(key=lambda t: t[0])
            for d, j in dists[:k_neighbors]:
                if d <= max_edge_distance:
                    graph.add_edge(i, j, d)

    was_fully_connected = is_connected(graph)
    if not was_fully_connected:
        _bridge_components(graph)

    return graph, was_fully_connected


def nearest_node(graph: MineGraph, position: Coord) -> Tuple[int, Coord, float]:
    """Brute-force nearest graph node to an arbitrary continuous (x,y,z)."""
    best_id = None
    best_coord = None
    best_dist = math.inf
    for node_id, coord in graph.nodes.items():
        d = euclidean(coord, position)
        if d < best_dist:
            best_dist = d
            best_id = node_id
            best_coord = coord
    return best_id, best_coord, best_dist


def blocked_nodes_for_hazard(
    graph: MineGraph, zone_center: Coord, radius: float
) -> Set[int]:
    """Node ids inside a hazard sphere — A* will never traverse these
    (except possibly as the source; see AStarRouter.shortest_path)."""
    return {
        node_id
        for node_id, coord in graph.nodes.items()
        if euclidean(coord, zone_center) <= radius
    }


@dataclass
class RouteResult:
    reachable: bool
    distance: Optional[float]
    nodes_explored: int
    path_nodes: List[int] = field(default_factory=list)
    path_coordinates: List[Coord] = field(default_factory=list)


class AStarRouter:
    """Hazard-aware A* shortest-path router over a MineGraph.

    A* uses:
      - g(n): exact path cost from source to the current node
      - h(n): Euclidean 3D distance from the current node to destination
      - f(n) = g(n) + h(n)

    Because each graph edge is weighted by Euclidean distance, the heuristic
    never overestimates the remaining route cost. A* therefore keeps shortest-
    path optimality while usually exploring fewer irrelevant tunnel nodes than
    an uninformed search.
    """

    def __init__(self, graph: MineGraph) -> None:
        self.graph = graph

    def _heuristic(self, node: int, destination: int) -> float:
        return euclidean(self.graph.nodes[node], self.graph.nodes[destination])

    def shortest_path(
        self,
        source: int,
        destination: int,
        blocked_nodes: Optional[Set[int]] = None,
    ) -> RouteResult:
        blocked = set(blocked_nodes) if blocked_nodes else set()
        # A worker/drone must be able to start from its current node even if a
        # hazard region has expanded over that exact position.
        blocked.discard(source)

        if destination in blocked:
            return RouteResult(reachable=False, distance=None, nodes_explored=0)

        if source == destination:
            return RouteResult(
                reachable=True,
                distance=0.0,
                nodes_explored=1,
                path_nodes=[source],
                path_coordinates=[self.graph.nodes[source]],
            )

        g_score: Dict[int, float] = {source: 0.0}
        prev: Dict[int, int] = {}
        closed: Set[int] = set()
        # heap item: (f_score, g_score, node_id)
        open_heap: List[Tuple[float, float, int]] = [
            (self._heuristic(source, destination), 0.0, source)
        ]
        nodes_explored = 0

        while open_heap:
            _f, current_g, node = heapq.heappop(open_heap)
            if node in closed:
                continue
            # Ignore stale heap entries that no longer match the best g-score.
            if current_g > g_score.get(node, math.inf):
                continue

            closed.add(node)
            nodes_explored += 1

            if node == destination:
                break

            for neighbor, weight in self.graph.neighbors(node):
                if neighbor in blocked or neighbor in closed:
                    continue

                tentative_g = current_g + weight
                if tentative_g < g_score.get(neighbor, math.inf):
                    g_score[neighbor] = tentative_g
                    prev[neighbor] = node
                    f_score = tentative_g + self._heuristic(neighbor, destination)
                    heapq.heappush(open_heap, (f_score, tentative_g, neighbor))

        if destination not in closed:
            return RouteResult(
                reachable=False, distance=None, nodes_explored=nodes_explored
            )

        path = [destination]
        cur = destination
        while cur != source:
            cur = prev[cur]
            path.append(cur)
        path.reverse()

        return RouteResult(
            reachable=True,
            distance=g_score[destination],
            nodes_explored=nodes_explored,
            path_nodes=path,
            path_coordinates=[self.graph.nodes[n] for n in path],
        )

    def closest_reachable_to_position(
        self,
        source: int,
        target_position: Coord,
        blocked_nodes: Optional[Set[int]] = None,
    ) -> Tuple[RouteResult, float]:
        """Find the reachable graph node geometrically closest to a target.

        This is used when a collapse blocks the exact worker location. First a
        BFS connectivity scan finds every safe graph node reachable from the
        selected exit. The geometrically closest reachable node to the worker
        is then chosen, and A* computes the actual shortest tunnel route to it.
        """
        blocked = set(blocked_nodes) if blocked_nodes else set()
        blocked.discard(source)

        reachable: Set[int] = set()
        queue: List[int] = [source]
        reachable.add(source)
        idx = 0
        while idx < len(queue):
            node = queue[idx]
            idx += 1
            for neighbor, _weight in self.graph.neighbors(node):
                if neighbor in blocked or neighbor in reachable:
                    continue
                reachable.add(neighbor)
                queue.append(neighbor)

        best_node = min(
            reachable,
            key=lambda n: euclidean(self.graph.nodes[n], target_position),
        )
        best_gap = euclidean(self.graph.nodes[best_node], target_position)
        result = self.shortest_path(source, best_node, blocked)
        return result, best_gap

    def shortest_distance(
        self,
        source: int,
        destination: int,
        blocked_nodes: Optional[Set[int]] = None,
    ) -> Optional[float]:
        return self.shortest_path(source, destination, blocked_nodes).distance

    def alternative_path(
        self,
        source: int,
        destination: int,
        primary_result: RouteResult,
        blocked_nodes: Optional[Set[int]] = None,
    ) -> RouteResult:
        """Produce a meaningfully different second route by temporarily
        blocking the interior nodes of the primary path and re-running A*."""
        if not primary_result.reachable or len(primary_result.path_nodes) <= 2:
            return RouteResult(reachable=False, distance=None, nodes_explored=0)

        blocked = set(blocked_nodes) if blocked_nodes else set()
        blocked |= set(primary_result.path_nodes[1:-1])
        blocked.discard(source)
        blocked.discard(destination)
        return self.shortest_path(source, destination, blocked_nodes=blocked)
