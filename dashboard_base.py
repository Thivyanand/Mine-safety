"""
dashboard_base.py — Flask backend for the Mine Safety Control Room.

All state lives in memory (dicts/lists guarded by a lock), reset on
restart. No database. Real hardware would POST telemetry to
/api/telemetry; the built-in simulate_loop fakes 5 workers for the demo.
"""

from __future__ import annotations

import json
import math
import random
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

from flask import Flask, jsonify, render_template, request

import config
import routing

app = Flask(__name__)

# ---------------------------------------------------------------------------
# CORS: the watch simulator runs as a plain HTML file (or its own tiny
# server) on a *second* laptop on the same WiFi, so its fetch() calls to
# this Flask server are cross-origin. No auth/cookies are involved here
# (LAN demo), so a permissive allow-all is fine — this just adds the
# headers browsers require to permit the cross-origin request at all.
# ---------------------------------------------------------------------------


@app.after_request
def _add_cors_headers(resp):
    resp.headers["Access-Control-Allow-Origin"] = "*"
    resp.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
    resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return resp


@app.route("/api/<path:_any>", methods=["OPTIONS"])
def _cors_preflight(_any):
    # Browsers send an OPTIONS preflight before cross-origin POSTs with a
    # JSON content-type. Answer it immediately; _add_cors_headers above
    # attaches the actual allow-headers to this response too.
    return ("", 204)


BASE_DIR = Path(__file__).resolve().parent

# ---------------------------------------------------------------------------
# Startup: load point cloud, define fixed exits/refuge, build the graph once
# ---------------------------------------------------------------------------

with open(BASE_DIR / "static" / "tunnel_points.json") as f:
    TUNNEL_POINTS: List[Tuple[float, float, float]] = [tuple(p) for p in json.load(f)]

# 3 permanently fixed Exit points, hardcoded as literal coordinates — NOT
# derived from list indices at runtime, so they can never silently shift if
# the point cloud is regenerated/reordered. Picked once via farthest-point
# sampling over TUNNEL_POINTS so they're spread across the mine.
EXIT_POINTS: List[Tuple[float, float, float]] = [
    (710.04, 295.43, -10.82),
    (-735.94, 186.84, 42.41),
    (25.10, -350.48, -15.75),
]

# Midpoint of TUNNEL_POINTS — a central refuge/muster point.
REFUGE_POINT: Tuple[float, float, float] = (-13.40, 3.01, -1.37)

GRAPH, GRAPH_WAS_FULLY_CONNECTED = routing.build_graph_from_points(
    TUNNEL_POINTS, config.GRAPH_K_NEIGHBORS, config.GRAPH_MAX_EDGE_DISTANCE
)
ROUTER = routing.AStarRouter(GRAPH)

# Resolve each exit + the refuge point to its nearest graph node once at
# startup — never recompute per-request.
EXIT_NODES: List[int] = []
for _pt in EXIT_POINTS:
    _node_id, _coord, _dist = routing.nearest_node(GRAPH, _pt)
    EXIT_NODES.append(_node_id)

REFUGE_NODE, _refuge_coord, _refuge_snap_dist = routing.nearest_node(GRAPH, REFUGE_POINT)

print(
    f"[MineGraph] nodes={GRAPH.num_nodes()} edges={GRAPH.num_edges()} "
    f"exit_nodes={EXIT_NODES} refuge_node={REFUGE_NODE} "
    f"fully_connected_before_bridging={GRAPH_WAS_FULLY_CONNECTED} "
    f"connected_now={routing.is_connected(GRAPH)}"
)

# ---------------------------------------------------------------------------
# In-memory state
# ---------------------------------------------------------------------------

workers: Dict[str, dict] = {}
events: List[dict] = []
emergency: dict = {"active": False}
buddy_alerts: Dict[str, dict] = {}

STATE_LOCK = threading.Lock()

STALE_AFTER_SECONDS = 8
HR_HIGH = 120
HR_LOW = 50
TEMP_HIGH = 38.0
HAZARD_RADIUS = 160

EVENT_LOG_CAP = 150

# ---------------------------------------------------------------------------
# Buddy Trigger System
#
# The moment a worker goes into distress (manual SOS, a fall, or stepping
# into an active hazard zone), the nearest other workers on shift are
# auto-notified as their "buddies" — same idea as a dive buddy system —
# alongside the control room. This runs independently of any zone-based
# emergency: it fires even for a single worker's SOS/fall with no
# declared emergency at all.
# ---------------------------------------------------------------------------

BUDDY_RADIUS = 220.0  # metres considered "nearby" for a buddy notification
MAX_BUDDIES = 4  # cap on how many nearby workers get pulled in per alert

REASON_LABELS = {"sos": "Manual SOS", "fall": "Fall detected", "hazard": "In hazard zone"}


def _is_distress(w: dict) -> bool:
    return bool(
        w.get("sos")
        or w.get("fall")
        or w.get("in_hazard")
        or w.get("emergency_affected")
    )


def _distress_reason(w: dict) -> str:
    if w.get("sos") or (
        w.get("emergency_affected") and emergency.get("type") == "manual_sos"
    ):
        return "sos"
    if w.get("fall"):
        return "fall"
    return "hazard"


def _compute_buddy_alerts() -> None:
    """Recomputes buddy_alerts from the current `workers` dict. Must be
    called with STATE_LOCK already held.

    Candidate ranking uses the real tunnel-graph walking distance (the same
    A* router that drives evacuation routes) — NOT straight-line
    distance — so a "nearby" buddy is someone who can actually walk there
    through the tunnel network, honoring any current hazard-blocked nodes.
    A candidate with no safe walkable path to the distressed worker is
    excluded entirely rather than ranked by an as-the-crow-flies number
    that could cut straight through rock or an active hazard.
    """
    now = time.time()
    distressed_ids = {wid for wid, w in workers.items() if _is_distress(w)}
    blocked = current_blocked_nodes()

    for wid in distressed_ids:
        w = workers[wid]
        is_new = wid not in buddy_alerts
        if is_new:
            buddy_alerts[wid] = {
                "worker_id": wid,
                "name": w.get("name", wid),
                "reason": _distress_reason(w),
                "triggered_at": now,
                "buddies": [],
            }

        target_node, _coord, _snap = routing.nearest_node(GRAPH, (w["x"], w["y"], w["z"]))

        candidates = []  # (walking_distance, oid, ow, route_result)
        for oid, ow in workers.items():
            if oid == wid or _is_distress(ow) or classify(ow) == "stale":
                continue
            source_node, _c, _s = routing.nearest_node(GRAPH, (ow["x"], ow["y"], ow["z"]))
            # Buddy walks FROM their own position TO the distressed worker,
            # over the real tunnel graph, respecting current hazard-blocked
            # nodes — a buddy is never routed through an active hazard just
            # to reach someone.
            result = ROUTER.shortest_path(source_node, target_node, blocked)
            if not result.reachable:
                continue  # no safe walkable path — not a valid buddy candidate
            candidates.append((result.distance, oid, ow, result))

        candidates.sort(key=lambda c: c[0])
        chosen = candidates[:MAX_BUDDIES]

        prev_ids = {b["worker_id"] for b in buddy_alerts[wid]["buddies"]}
        new_buddies = [
            {
                "worker_id": oid,
                "name": ow.get("name", oid),
                "distance_m": round(distance, 1),
                "eta_seconds": round(distance / config.WALKING_SPEED_MPS, 1),
                "within_radius": distance <= BUDDY_RADIUS,
                "route_coordinates": [list(c) for c in result.path_coordinates],
            }
            for distance, oid, ow, result in chosen
        ]
        buddy_alerts[wid]["buddies"] = new_buddies

        if is_new:
            names = ", ".join(b["name"] for b in new_buddies) or "no one reachable"
            log_event(
                f"BUDDY TRIGGER: {w.get('name', wid)} — {REASON_LABELS[buddy_alerts[wid]['reason']]}. "
                f"Notified nearby (via tunnel route): {names}. Control room alerted.",
                "critical",
            )
        else:
            added = {b["worker_id"] for b in new_buddies} - prev_ids
            if added:
                added_names = ", ".join(b["name"] for b in new_buddies if b["worker_id"] in added)
                log_event(f"Buddy alert for {w.get('name', wid)} — also notified: {added_names}", "warning")

    for wid in list(buddy_alerts.keys()):
        if wid not in distressed_ids:
            log_event(f"Buddy alert stood down for {buddy_alerts[wid]['name']} — condition cleared", "info")
            del buddy_alerts[wid]

EMERGENCY_TYPES = {
    "gas_leak": {
        "label": "Gas Leak",
        "color": "#f4c542",
        "zone_based": True,
        "action": "Evacuate the hazard zone immediately. Do not remove respirators.",
    },
    "co_leak": {
        "label": "CO Leak",
        "color": "#e88a2c",
        "zone_based": True,
        "action": "Move upwind and clear of the zone. CO is odorless — do not linger to investigate.",
    },
    "fire": {
        "label": "Fire",
        "color": "#e5533d",
        "zone_based": True,
        "action": "Evacuate away from smoke. Do not re-enter the zone for any reason.",
    },
    "explosion": {
        "label": "Explosion",
        "color": "#d81e3f",
        "zone_based": True,
        "action": "Move to the nearest exit or refuge immediately. Watch for secondary collapse.",
    },
    "tunnel_collapse": {
        "label": "Tunnel Collapse",
        "color": "#8a6d3b",
        "zone_based": True,
        "action": "Do not approach the collapse zone. Reroute via the alternative path.",
    },
    "rock_fall": {
        "label": "Rock Fall",
        "color": "#a08262",
        "zone_based": True,
        "action": "Clear the zone and stay alert for further falls along the route.",
    },
    "low_oxygen": {
        "label": "Low Oxygen",
        "color": "#4c9ed9",
        "zone_based": True,
        "action": "Evacuate the zone now. Low O2 can cause impairment before it's noticed.",
    },
    "water_ingress": {
        "label": "Water Ingress",
        "color": "#2f7cc9",
        "zone_based": True,
        "action": "Move to higher ground along the route. Avoid flooded sections entirely.",
    },
    "equipment_failure": {
        "label": "Equipment Failure",
        "color": "#8a8a8a",
        "zone_based": False,
        "action": "Shut down affected equipment. Non-zone incident — monitor affected crew.",
    },
    "manual_sos": {
        "label": "Manual SOS",
        "color": "#d81e3f",
        "zone_based": False,
        "action": "Worker-triggered SOS. Dispatch rescue to the worker's last known position.",
    },
}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def log_event(text: str, level: str = "info") -> None:
    events.append({"text": text, "level": level, "ts": time.time()})
    if len(events) > EVENT_LOG_CAP:
        del events[: len(events) - EVENT_LOG_CAP]


def dist3(a, b) -> float:
    """Plain Euclidean distance for game-loop-style code operating on
    worker dicts/lists (kept separate from routing.euclidean, which
    operates on graph node coordinates)."""
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def classify(w: dict) -> str:
    now = time.time()
    if now - w.get("last_seen", 0) > STALE_AFTER_SECONDS:
        return "stale"
    if w.get("sos") or w.get("fall") or w.get("in_hazard") or w.get("emergency_affected"):
        return "critical"
    hr = w.get("hr", 75)
    temp = w.get("temp", 36.5)
    if hr >= HR_HIGH or hr <= HR_LOW or temp >= TEMP_HIGH:
        return "warning"
    return "normal"


def compute_risk(w: dict) -> float:
    risk = 0.0
    hr = w.get("hr", 75)
    temp = w.get("temp", 36.5)
    if hr >= HR_HIGH:
        risk += min(3.0, (hr - HR_HIGH) / 15.0 + 1.0)
    elif hr <= HR_LOW:
        risk += min(3.0, (HR_LOW - hr) / 10.0 + 1.0)
    if temp >= TEMP_HIGH:
        risk += min(2.5, (temp - TEMP_HIGH) * 1.5 + 1.0)
    if w.get("in_hazard"):
        risk += 4.0
    if w.get("fall"):
        risk += 3.0
    if w.get("sos"):
        risk += 5.0
    return round(min(risk, 10.0), 1)


def compute_fatigue(w: dict) -> str:
    now = time.time()
    shift_seconds = now - w.get("first_seen", now)
    shift_hours = shift_seconds / 3600.0
    hr = w.get("hr", 75)
    score = shift_hours * 0.6 + max(0, hr - 90) * 0.03
    if score >= 3.0:
        return "High"
    if score >= 1.2:
        return "Moderate"
    return "Low"


def compute_ai_note(w: dict, risk: float) -> str:
    if w.get("in_hazard"):
        return "Worker is inside an active hazard zone — evacuate via the routed path now."
    if w.get("fall"):
        return "Fall detected — check in with worker immediately; may need physical assistance."
    if w.get("sos"):
        return "Manual SOS triggered — treat as highest priority, dispatch rescue now."
    hr = w.get("hr", 75)
    if hr >= HR_HIGH:
        return "Elevated heart rate — suggest a rest break and hydration check."
    if hr <= HR_LOW:
        return "Unusually low heart rate — verify sensor contact and worker responsiveness."
    if risk >= 5:
        return "Multiple mild risk factors — keep an eye on this worker."
    return "No concerns — vitals and position nominal."


def current_blocked_nodes() -> Set[int]:
    if not emergency.get("active"):
        return set()
    etype = EMERGENCY_TYPES.get(emergency.get("type", ""), {})
    if not etype.get("zone_based"):
        return set()
    zone_center = emergency.get("zone_center")
    radius = emergency.get("radius", HAZARD_RADIUS)
    if not zone_center:
        return set()
    return routing.blocked_nodes_for_hazard(GRAPH, tuple(zone_center), radius)


def _effective_blocked(w: dict, blocked_nodes: Set[int]) -> Set[int]:
    # The hazard-blocked node set exists to stop workers OUTSIDE the danger
    # zone from being routed into it. A worker who is already standing
    # inside the hazard has no such option — every neighbor of their own
    # graph node is typically also within the hazard radius (the radius is
    # larger than a single edge), so leaving the full blocked set in place
    # traps them with zero reachable neighbors and A* reports "no
    # safe route" even though a real path out exists. A worker already in
    # the hazard must be allowed to route straight through it to escape.
    if w.get("in_hazard"):
        return set()
    return blocked_nodes


def graph_evacuation_distances(
    w: dict, blocked_nodes: Set[int]
) -> Tuple[Optional[float], Optional[float]]:
    pos = (w["x"], w["y"], w["z"])
    node_id, _coord, _snap = routing.nearest_node(GRAPH, pos)
    blocked_nodes = _effective_blocked(w, blocked_nodes)

    best_exit_dist = None
    for exit_node in EXIT_NODES:
        d = ROUTER.shortest_distance(node_id, exit_node, blocked_nodes)
        if d is not None and (best_exit_dist is None or d < best_exit_dist):
            best_exit_dist = d

    refuge_dist = ROUTER.shortest_distance(node_id, REFUGE_NODE, blocked_nodes)
    return best_exit_dist, refuge_dist


def evacuation_route(w: dict, blocked_nodes: Set[int], include_alternative: bool = True) -> dict:
    pos = (w["x"], w["y"], w["z"])
    source_node, _coord, _snap = routing.nearest_node(GRAPH, pos)
    blocked_nodes = _effective_blocked(w, blocked_nodes)

    # Evacuation MUST terminate at one of the three exits.
    # The refuge is intentionally NOT a valid evacuation destination.
    # Calculate A* shortest safe path to every exit, then choose
    # the shortest reachable exit.
    candidates = []  # (distance, destination_type, exit_index, result)
    for i, exit_node in enumerate(EXIT_NODES):
        result = ROUTER.shortest_path(source_node, exit_node, blocked_nodes)
        if result.reachable:
            candidates.append((result.distance, "exit", i, result))

    if not candidates:
        return {
            "reachable": False,
            "destination_type": "exit",
            "destination_label": None,
            "destination_exit_index": None,
            "distance_m": None,
            "eta_seconds": None,
            "nodes_explored": 0,
            "route_nodes": [],
            "route_coordinates": [],
            "alternative_available": False,
            "alternative_route_coordinates": [],
            "status": "no_safe_route",
        }

    candidates.sort(key=lambda c: c[0])
    distance, dest_type, exit_index, result = candidates[0]

    if dest_type == "exit":
        label = f"Exit {exit_index + 1}"
        destination_node = EXIT_NODES[exit_index]
    else:
        label = "Refuge"
        destination_node = REFUGE_NODE

    out = {
        "reachable": True,
        "destination_type": dest_type,
        "destination_label": label,
        "destination_exit_index": exit_index,
        "distance_m": round(distance, 1),
        "eta_seconds": round(distance / config.WALKING_SPEED_MPS, 1),
        "nodes_explored": result.nodes_explored,
        "route_nodes": result.path_nodes,
        "route_coordinates": [list(c) for c in result.path_coordinates],
        "alternative_available": False,
        "alternative_route_coordinates": [],
        "status": "safe",
    }

    if include_alternative:
        alt = ROUTER.alternative_path(source_node, destination_node, result, blocked_nodes)
        if alt.reachable:
            out["alternative_available"] = True
            out["alternative_route_coordinates"] = [list(c) for c in alt.path_coordinates]

    return out


def enrich(w: dict, blocked_nodes: Optional[Set[int]] = None) -> dict:
    if blocked_nodes is None:
        blocked_nodes = current_blocked_nodes()
    out = dict(w)
    risk = compute_risk(w)
    out["status"] = classify(w)
    out["risk"] = risk
    out["fatigue"] = compute_fatigue(w)
    out["ai_note"] = compute_ai_note(w, risk)
    out["seconds_ago"] = round(time.time() - w.get("last_seen", time.time()), 1)
    dist_to_exit, dist_to_refuge = graph_evacuation_distances(w, blocked_nodes)
    out["dist_to_exit"] = round(dist_to_exit, 1) if dist_to_exit is not None else None
    out["dist_to_refuge"] = round(dist_to_refuge, 1) if dist_to_refuge is not None else None
    return out


def update_hazard_flags(w: dict) -> None:
    was_in_hazard = w.get("in_hazard", False)
    now_in_hazard = False
    if emergency.get("active"):
        etype = EMERGENCY_TYPES.get(emergency.get("type", ""), {})
        if etype.get("zone_based") and emergency.get("zone_center"):
            zc = emergency["zone_center"]
            radius = emergency.get("radius", HAZARD_RADIUS)
            if dist3((w["x"], w["y"], w["z"]), zc) <= radius:
                now_in_hazard = True
    w["in_hazard"] = now_in_hazard
    if now_in_hazard and not was_in_hazard:
        log_event(f"{w.get('name', w.get('worker_id'))} entered the hazard zone", "critical")
    elif was_in_hazard and not now_in_hazard:
        log_event(f"{w.get('name', w.get('worker_id'))} is clear of the hazard zone", "info")


def ingest_telemetry(payload: dict) -> dict:
    worker_id = payload["worker_id"]
    now = time.time()
    with STATE_LOCK:
        existing = workers.get(worker_id)
        w = dict(existing) if existing else {}
        w["worker_id"] = worker_id
        w["name"] = payload.get("name", w.get("name", worker_id))
        w["hr"] = payload.get("hr", w.get("hr", 75))
        w["temp"] = payload.get("temp", w.get("temp", 36.6))
        w["spo2"] = payload.get("spo2", w.get("spo2", 98))
        w["battery"] = payload.get("battery", w.get("battery", 100))
        w["x"] = payload.get("x", w.get("x", 0.0))
        w["y"] = payload.get("y", w.get("y", 0.0))
        w["z"] = payload.get("z", w.get("z", 0.0))
        w["zone"] = payload.get("zone", w.get("zone", "unassigned"))
        w["activity"] = payload.get("activity", w.get("activity", "walking"))
        w["fall"] = payload.get("fall", w.get("fall", False))
        w["sos"] = payload.get("sos", w.get("sos", False))
        w["first_seen"] = w.get("first_seen", now)
        w["last_seen"] = now
        update_hazard_flags(w)
        workers[worker_id] = w
        _compute_buddy_alerts()
        return w


# ---------------------------------------------------------------------------
# Emergency-victim identification
# ---------------------------------------------------------------------------


def _choose_scenario_worker(preferred_worker_id: Optional[str] = None) -> Optional[str]:
    """Choose one real worker as the anchor/victim for a dashboard simulation.

    An explicit worker_id supplied by the API is honored. Otherwise, choose
    among fresh workers and avoid always defaulting to the first roster item
    (J. Vance). This worker becomes the center of the simulated incident. Must be called
    with STATE_LOCK held.
    """
    if preferred_worker_id and preferred_worker_id in workers:
        return preferred_worker_id

    fresh = [
        wid for wid, w in workers.items()
        if classify(w) != "stale"
    ]
    if not fresh:
        return next(iter(workers), None)

    previous = emergency.get("previous_primary_worker_id")
    choices = [wid for wid in fresh if wid != previous] or fresh
    return random.choice(choices)


def _identify_affected_workers(primary_worker_id: Optional[str]) -> List[str]:
    """Return the workers actually affected by the currently declared event.

    Zone emergencies use the live in_hazard flag. Non-zone simulations are
    worker-specific, so only their chosen primary worker is affected.
    """
    if not emergency.get("active"):
        return []

    etype = EMERGENCY_TYPES.get(emergency.get("type", ""), {})
    if etype.get("zone_based"):
        affected = [wid for wid, w in workers.items() if w.get("in_hazard")]
        if primary_worker_id and primary_worker_id in workers and primary_worker_id not in affected:
            # Safety net for graph/model coordinate mismatch: the worker used
            # to anchor the simulated incident must remain the primary victim.
            affected.insert(0, primary_worker_id)
        return affected

    if primary_worker_id and primary_worker_id in workers:
        return [primary_worker_id]
    return []


def _set_emergency_affected_flags(affected_ids: List[str]) -> None:
    affected = set(affected_ids)
    for wid, w in workers.items():
        w["emergency_affected"] = wid in affected


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------


@app.route("/api/telemetry", methods=["POST"])
def api_telemetry():
    payload = request.get_json(force=True)
    if not payload or "worker_id" not in payload:
        return jsonify({"ok": False, "error": "worker_id required"}), 400
    ingest_telemetry(payload)
    return jsonify({"ok": True})


@app.route("/api/workers", methods=["GET"])
def api_workers():
    with STATE_LOCK:
        blocked = current_blocked_nodes()
        enriched = [enrich(w, blocked) for w in workers.values()]
    return jsonify({"workers": enriched, "server_time": time.time()})


@app.route("/api/emergency", methods=["GET"])
def api_emergency():
    with STATE_LOCK:
        if not emergency.get("active"):
            return jsonify({"active": False})

        blocked = current_blocked_nodes()
        # Only workers actually tied to the current emergency belong in the
        # emergency rescue set. Unrelated abnormal vitals must not make a
        # different worker appear as the incident victim.
        affected_ids = [
            wid for wid, w in workers.items()
            if w.get("in_hazard") or w.get("emergency_affected")
        ]
        primary = emergency.get("primary_worker_id")
        if primary and primary in workers and primary not in affected_ids:
            affected_ids.insert(0, primary)

        rescue_routes = {}
        for wid in affected_ids:
            w = workers[wid]
            route = evacuation_route(w, blocked, include_alternative=True)
            rescue_routes[wid] = {
                "primary": route["route_coordinates"],
                "alternative": route["alternative_route_coordinates"],
                "eta_seconds": route["eta_seconds"],
                "algorithm": "A*",
                "route_status": "SAFE" if route["reachable"] else "NO SAFE ROUTE",
                "destination_type": route["destination_type"],
                "destination_label": route["destination_label"],
                "destination_exit_index": route["destination_exit_index"],
                "distance_m": route["distance_m"],
                "nodes_explored": route["nodes_explored"],
                "alternative_available": route["alternative_available"],
            }

        out = dict(emergency)
        out["workers_affected"] = affected_ids
        out["rescue_routes"] = rescue_routes
        return jsonify(out)


@app.route("/api/emergency/start", methods=["POST"])
def api_emergency_start():
    payload = request.get_json(force=True) or {}
    etype_key = payload.get("type")
    if etype_key not in EMERGENCY_TYPES:
        return jsonify({"ok": False, "error": "unknown emergency type"}), 400

    etype = EMERGENCY_TYPES[etype_key]
    with STATE_LOCK:
        # Preserve only the previous primary id so repeated simulations do not
        # keep choosing the first/default roster worker.
        previous_primary = emergency.get("primary_worker_id")

        emergency.clear()
        emergency["active"] = True
        emergency["type"] = etype_key
        emergency["label"] = etype["label"]
        emergency["color"] = etype["color"]
        emergency["zone_based"] = etype["zone_based"]
        emergency["action"] = etype["action"]
        emergency["started_at"] = time.time()
        emergency["previous_primary_worker_id"] = previous_primary

        # Step 1 — choose/identify the actual worker this simulation affects.
        primary_worker_id = _choose_scenario_worker(payload.get("worker_id"))
        emergency["primary_worker_id"] = primary_worker_id

        # Step 2 — anchor a zone emergency on that same real worker unless an
        # explicit zone_center is supplied through the API. This removes the
        # old mismatch where the simulated hazard could be tied to a different
        # worker than the incident shown in the dashboard.
        if etype["zone_based"]:
            zone_center = payload.get("zone_center")
            if not zone_center:
                if primary_worker_id and primary_worker_id in workers:
                    w = workers[primary_worker_id]
                    zone_center = [w["x"], w["y"], w["z"]]
                else:
                    p = random.choice(TUNNEL_POINTS)
                    zone_center = list(p)
            emergency["zone_center"] = zone_center
            emergency["radius"] = payload.get("radius", HAZARD_RADIUS)

        # Step 3 — update hazard flags first, then identify the worker(s) truly
        # affected by this event. The primary worker remains the incident anchor.
        for w in workers.values():
            update_hazard_flags(w)
        affected_ids = _identify_affected_workers(primary_worker_id)
        _set_emergency_affected_flags(affected_ids)
        emergency["workers_affected"] = affected_ids

        # If an explicit zone center was supplied, choose the affected worker
        # closest to that center as the primary target instead of keeping an
        # unrelated preselected roster worker.
        if etype["zone_based"] and affected_ids and payload.get("zone_center"):
            zc = tuple(emergency["zone_center"])
            primary_worker_id = min(
                affected_ids,
                key=lambda wid: dist3(
                    (workers[wid]["x"], workers[wid]["y"], workers[wid]["z"]), zc
                ),
            )
            emergency["primary_worker_id"] = primary_worker_id

        _compute_buddy_alerts()

        victim_name = (
            workers.get(primary_worker_id, {}).get("name", primary_worker_id)
            if primary_worker_id else "no worker"
        )
        log_event(
            f"EMERGENCY: {etype['label']} declared — primary affected worker: {victim_name}",
            "critical",
        )
        # Prototype scope: after identifying the actual affected worker, the
        # dashboard focuses on buddy alerts and safe evacuation routing.
        out_emergency = dict(emergency)

    return jsonify({"ok": True, "emergency": out_emergency})


@app.route("/api/emergency/reset", methods=["POST"])
def api_emergency_reset():
    with STATE_LOCK:
        emergency.clear()
        emergency["active"] = False
        for w in workers.values():
            w["in_hazard"] = False
            w["emergency_affected"] = False
        _compute_buddy_alerts()
        log_event("Emergency cleared — all-clear", "info")
    return jsonify({"ok": True})


@app.route("/api/buddy-alerts", methods=["GET"])
def api_buddy_alerts():
    with STATE_LOCK:
        _compute_buddy_alerts()
        alerts = sorted(buddy_alerts.values(), key=lambda a: a["triggered_at"])
        out = [dict(a) for a in alerts]
    return jsonify({"alerts": out, "radius_m": BUDDY_RADIUS, "server_time": time.time()})


@app.route("/api/events", methods=["GET"])
def api_events():
    with STATE_LOCK:
        return jsonify({"events": list(events)})


@app.route("/api/routes", methods=["GET"])
def api_routes():
    with STATE_LOCK:
        blocked = current_blocked_nodes()
        out = {wid: evacuation_route(w, blocked) for wid, w in workers.items()}
    return jsonify(out)


@app.route("/api/routes/<worker_id>", methods=["GET"])
def api_route_single(worker_id):
    with STATE_LOCK:
        w = workers.get(worker_id)
        if not w:
            return jsonify({"error": "unknown worker"}), 404
        blocked = current_blocked_nodes()
        return jsonify(evacuation_route(w, blocked))


@app.route("/api/routing/status", methods=["GET"])
def api_routing_status():
    return jsonify(
        {
            "algorithm": "A* (binary heap, Euclidean heuristic, adjacency list)",
            "num_nodes": GRAPH.num_nodes(),
            "num_edges": GRAPH.num_edges(),
            "exit_nodes": EXIT_NODES,
            "exit_points": [list(p) for p in EXIT_POINTS],
            "refuge_node": REFUGE_NODE,
            "refuge_point": list(REFUGE_POINT),
            "graph_connected": routing.is_connected(GRAPH),
            "graph_ready": True,
            "k_neighbors": config.GRAPH_K_NEIGHBORS,
            "max_edge_distance": config.GRAPH_MAX_EDGE_DISTANCE,
        }
    )


@app.route("/")
def index():
    return render_template("dashboard.html")


@app.route("/watch")
def watch():
    # Served from this same Flask app so a phone on the WiFi can just
    # browse to http://<lan-ip>:<port>/watch — no file:// opening, no
    # mobile-browser CORS/private-network quirks. See templates/watch.html.
    return render_template("watch.html")


# ---------------------------------------------------------------------------
# Built-in worker simulator
# ---------------------------------------------------------------------------

WORKER_NAMES = ["W1", "W2", "W3", "W4", "W5"]

# Per-worker walk state: the graph-node waypoint list currently being
# followed, the index of the next waypoint, and whether that path was
# computed as an evacuation route (so we know to recompute if the hazard
# status flips). Workers ALWAYS move waypoint-to-waypoint along a real
# A* path over the tunnel graph now — never in a raw straight line
# to a distant target — so they can't cut through rock or drift outside
# the tunnel network.
_sim_paths: Dict[str, List[Tuple[float, float, float]]] = {}
_sim_path_idx: Dict[str, int] = {}
_sim_evacuating: Dict[str, bool] = {}

WAYPOINT_ARRIVAL_TOLERANCE = 3.0


def _random_point() -> Tuple[float, float, float]:
    return random.choice(TUNNEL_POINTS)


def _new_wander_path(pos: Tuple[float, float, float]) -> List[Tuple[float, float, float]]:
    """A real A* path along the tunnel graph from pos to a random
    reachable node — used for normal (non-emergency) wandering."""
    src, _coord, _snap = routing.nearest_node(GRAPH, pos)
    for _ in range(5):
        dest, _c, _s = routing.nearest_node(GRAPH, _random_point())
        if dest == src:
            continue
        result = ROUTER.shortest_path(src, dest)
        if result.reachable and len(result.path_coordinates) >= 2:
            return result.path_coordinates
    return [pos]


def _new_evac_path(
    pos: Tuple[float, float, float], blocked: Set[int]
) -> List[Tuple[float, float, float]]:
    """A real A* path along the tunnel graph to the nearest reachable
    exit or refuge, honoring current hazard-blocked nodes — used once a
    worker enters a hazard zone."""
    src, _coord, _snap = routing.nearest_node(GRAPH, pos)
    # This is only ever called for a worker who is already in_hazard (see
    # the caller). Same reasoning as _effective_blocked() above: a worker
    # standing inside the hazard needs to route straight through it to
    # escape, so the hazard-blocked set is not applied to their own path.
    candidates = []
    for exit_node in EXIT_NODES:
        r = ROUTER.shortest_path(src, exit_node, set())
        if r.reachable:
            candidates.append((r.distance, r.path_coordinates))
    if not candidates:
        return [pos]
    candidates.sort(key=lambda c: c[0])
    return candidates[0][1]


def _step_toward(pos, target, step):
    d = dist3(pos, target)
    if d < 1e-6:
        return target
    t = min(1.0, step / d)
    return (
        pos[0] + (target[0] - pos[0]) * t,
        pos[1] + (target[1] - pos[1]) * t,
        pos[2] + (target[2] - pos[2]) * t,
    )


def _nudge(value, low, high, step):
    value += random.uniform(-step, step)
    return max(low, min(high, value))


def simulation_loop():
    log_event("Simulator started — spawning 5 workers", "info")
    sim_state: Dict[str, dict] = {}
    for name in WORKER_NAMES:
        start = _random_point()
        sim_state[name] = {
            "pos": start,
            "hr": random.uniform(68, 88),
            "temp": random.uniform(36.4, 37.1),
            "spo2": random.uniform(96, 99),
            "battery": random.uniform(70, 100),
        }
        _sim_evacuating[name] = False

    while True:
        for name in WORKER_NAMES:
            s = sim_state[name]
            with STATE_LOCK:
                w = workers.get(name)
                in_hazard = w.get("in_hazard", False) if w else False
                blocked = current_blocked_nodes()

            path = _sim_paths.get(name)
            idx = _sim_path_idx.get(name, 0)
            was_evacuating = _sim_evacuating.get(name, False)

            # Recompute the path (over the real tunnel graph) whenever we've
            # run out of waypoints, or the hazard state just changed —
            # switching between normal wandering and evacuation always
            # re-routes via A* rather than jumping straight-line to a
            # distant point.
            needs_new_path = (
                not path
                or idx >= len(path)
                or in_hazard != was_evacuating
            )
            if needs_new_path:
                if in_hazard:
                    path = _new_evac_path(s["pos"], blocked)
                else:
                    path = _new_wander_path(s["pos"])
                idx = 0
                _sim_evacuating[name] = in_hazard

            waypoint = path[min(idx, len(path) - 1)]
            s["pos"] = _step_toward(s["pos"], waypoint, step=random.uniform(2.5, 5.0))

            if dist3(s["pos"], waypoint) < WAYPOINT_ARRIVAL_TOLERANCE:
                idx += 1

            _sim_paths[name] = path
            _sim_path_idx[name] = idx
            s["hr"] = _nudge(s["hr"], 55, 135, 3)
            s["temp"] = _nudge(s["temp"], 35.8, 39.0, 0.15)
            s["spo2"] = _nudge(s["spo2"], 90, 99.5, 0.5)
            s["battery"] = max(5.0, s["battery"] - random.uniform(0, 0.05))

            ingest_telemetry(
                {
                    "worker_id": name,
                    "name": name,
                    "hr": round(s["hr"], 1),
                    "temp": round(s["temp"], 2),
                    "spo2": round(s["spo2"], 1),
                    "battery": round(s["battery"], 1),
                    "x": s["pos"][0],
                    "y": s["pos"][1],
                    "z": s["pos"][2],
                    "zone": "auto",
                    "activity": "evacuating" if in_hazard else "walking",
                    "fall": random.random() < 0.002,
                    "sos": False,
                }
            )
        time.sleep(1.5)
