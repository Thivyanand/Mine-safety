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
import sensors

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

# Sensor Digital Twin layer — fixed gas/oxygen/temperature/vibration/
# structural sensors and UWB anchors placed on the same point cloud the
# tunnel graph is built from. All readings are SIMULATED SENSOR TELEMETRY.
sensors.build_sensors(TUNNEL_POINTS)

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

# Rescue-drone V1 state.  The backend plans the route; the browser animates
# the drone along the returned graph coordinates and POSTs its live position
# back here for the command-center telemetry panel.
drone_state: dict = {
    "drone_id": "DRONE-01",
    "active": False,
    "mission_status": "STANDBY",
    "target_worker_id": None,
    "entry_exit_index": None,
    "entry_exit_label": None,
    "route_status": "IDLE",
    "route_coordinates": [],
    "route_revision": 0,
    "distance_m": None,
    "remaining_gap_m": None,
    "terminal_state": "STANDBY",
    "position": None,
    "battery": 100.0,
    "signal": 100.0,
    "speed": 0.0,
    "started_at": None,
}

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

# Which sensor category(ies) are the origin of each zone-based emergency
# type. Triggering one of these types no longer creates the hazard zone
# immediately — it arms the relevant sensor(s), which climb NORMAL ->
# WARNING -> CRITICAL over a few seconds, and only crossing CRITICAL
# actually creates the hazard zone / affects workers / reroutes A*. Types
# not listed here (explosion, water_ingress, equipment_failure, manual_sos)
# keep the original immediate-activation behavior.
# Drone dispatch policy: the rescue/inspection drone is intentionally
# reserved for events where remote reconnaissance is actually useful.
# It must NOT launch for routine environmental alerts.
DRONE_TRIGGER_EMERGENCIES = {"manual_sos", "tunnel_collapse"}


EMERGENCY_SENSOR_MAP = {
    "gas_leak": ["gas"],
    "co_leak": ["gas"],
    "fire": ["temperature"],
    "low_oxygen": ["oxygen"],
    "tunnel_collapse": ["vibration", "structural"],
    "rock_fall": ["vibration"],
}

# sensor_id -> pending group id, and group id -> group info, for emergencies
# that are "armed" (a sensor is deteriorating) but not yet active.
_pending_sensor_targets: Dict[str, str] = {}
_pending_groups: Dict[str, dict] = {}


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
    # exit_options records EVERY exit (reachable or not) purely for the
    # frontend's "compare reachable exits" demo visualization (Worker
    # Intelligence -> Simulate Safe Route). It does not change which exit
    # is actually chosen below — that still only ever considers `candidates`.
    exit_options: List[dict] = []
    for i, exit_node in enumerate(EXIT_NODES):
        result = ROUTER.shortest_path(source_node, exit_node, blocked_nodes)
        exit_options.append({
            "exit_index": i,
            "label": f"Exit {i + 1}",
            "reachable": result.reachable,
            "distance_m": round(result.distance, 1) if result.reachable else None,
        })
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
            "exit_options": exit_options,
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
        "exit_options": exit_options,
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
        # Direct wearable dispatch rule: ONLY a worker SOS can launch the
        # drone without a dashboard emergency. A fall alert still triggers
        # worker/buddy safety logic, but it does not launch the drone.
        if w.get("sos") and not drone_state.get("active"):
            _plan_drone_mission(worker_id)
        return w


# ---------------------------------------------------------------------------
# Emergency-victim identification
# ---------------------------------------------------------------------------


def _choose_scenario_worker(preferred_worker_id: Optional[str] = None) -> Optional[str]:
    """Choose one real worker as the anchor/victim for a dashboard simulation.

    An explicit worker_id supplied by the API is honored. Otherwise, choose
    among fresh workers and avoid always defaulting to the first roster item
    (J. Vance). This worker becomes the center of a zone emergency and the
    primary drone target. Must be called with STATE_LOCK held.
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


def _drone_blocked_nodes() -> Set[int]:
    """Physical blockages the drone must not cross.

    A gas/oxygen/fire hazard may be unsafe for a human but an inspection drone
    can still enter it to inspect the worker. A tunnel collapse is different:
    the passage is physically blocked, so the drone must approach the closest
    safely reachable point and hold position there.
    """
    if not emergency.get("active"):
        return set()
    if emergency.get("type") == "tunnel_collapse":
        return current_blocked_nodes()
    return set()


# ---------------------------------------------------------------------------
# Rescue drone V1 helpers
# ---------------------------------------------------------------------------


def _select_drone_target(preferred_worker_id: Optional[str] = None) -> Optional[str]:
    """Choose the worker the drone should inspect.

    During a qualifying dashboard emergency, the worker identified as the
    primary victim is the single source of truth. Outside a declared
    emergency, only a direct watch SOS is allowed to dispatch the drone.
    Must be called with STATE_LOCK held.
    """
    primary = emergency.get("primary_worker_id") if emergency.get("active") else None
    if primary and primary in workers:
        return primary

    if preferred_worker_id and preferred_worker_id in workers:
        return preferred_worker_id

    distressed = []
    for wid, w in workers.items():
        priority = 99
        if w.get("sos"):
            priority = 0
        elif w.get("fall"):
            priority = 1
        elif w.get("in_hazard") or w.get("emergency_affected"):
            priority = 2
        if priority < 99:
            distressed.append((priority, wid))
    if distressed:
        distressed.sort()
        return distressed[0][1]

    return next(iter(workers), None)


def _drone_sensor_snapshot() -> dict:
    """Simple simulated environmental telemetry for Version 1."""
    vals = {"ch4": 0.08, "co": 4.0, "o2": 20.8, "temperature": 29.5}
    etype = emergency.get("type") if emergency.get("active") else None
    if etype == "gas_leak":
        vals["ch4"] = 1.35
    elif etype == "co_leak":
        vals["co"] = 78.0
    elif etype == "fire":
        vals["temperature"] = 48.0
        vals["co"] = 42.0
    elif etype == "low_oxygen":
        vals["o2"] = 17.4
    elif etype in ("explosion", "tunnel_collapse", "rock_fall"):
        vals["temperature"] = 32.0
    return vals


def _plan_drone_mission(worker_id: str) -> bool:
    """Plan DRONE-01 from the best of the three exits to a worker.

    The primary attempt uses the normal hazard-aware graph path. If the
    worker is unreachable (for example behind a collapse), each exit is
    searched for the safely reachable graph node closest to the worker and
    the best approach is selected. The drone never receives a straight-line
    path through rock.

    Must be called with STATE_LOCK held.
    """
    w = workers.get(worker_id)
    if not w:
        return False

    target_pos = (w["x"], w["y"], w["z"])
    target_node, _target_coord, _snap = routing.nearest_node(GRAPH, target_pos)
    blocked = _drone_blocked_nodes()
    force_standby_near_worker = (
        emergency.get("active") and emergency.get("type") == "tunnel_collapse"
    )

    direct = []
    for i, exit_node in enumerate(EXIT_NODES):
        # A drone must not enter through an exit that is itself inside the
        # currently blocked/hazardous region.
        if exit_node in blocked:
            continue
        if force_standby_near_worker:
            continue
        result = ROUTER.shortest_path(exit_node, target_node, blocked)
        if result.reachable:
            direct.append((result.distance, i, result))

    if direct:
        direct.sort(key=lambda x: x[0])
        distance, exit_index, result = direct[0]
        remaining_gap = routing.euclidean(GRAPH.nodes[target_node], target_pos)
        terminal_state = "WORKER_REACHED"
        route_status = "SAFE"
    else:
        approaches = []
        for i, exit_node in enumerate(EXIT_NODES):
            if exit_node in blocked:
                continue
            result, gap = ROUTER.closest_reachable_to_position(
                exit_node, target_pos, blocked
            )
            # Prefer getting physically closest to the worker; route length
            # breaks ties so the selected entry is still sensible.
            approaches.append((gap, result.distance or 0.0, i, result))
        if not approaches:
            return False
        approaches.sort(key=lambda x: (x[0], x[1]))
        remaining_gap, distance, exit_index, result = approaches[0]
        terminal_state = "STANDBY_NEAR_BLOCKAGE"
        route_status = "BLOCKED_APPROACH"

    coords = [list(c) for c in result.path_coordinates]
    literal_exit = list(EXIT_POINTS[exit_index])
    if not coords or routing.euclidean(tuple(coords[0]), tuple(literal_exit)) > 0.01:
        coords.insert(0, literal_exit)

    drone_state.update(
        {
            "active": True,
            "mission_status": "DISPATCHED",
            "target_worker_id": worker_id,
            "entry_exit_index": exit_index,
            "entry_exit_label": f"EXIT {exit_index + 1}",
            "route_status": route_status,
            "route_coordinates": coords,
            "route_revision": int(drone_state.get("route_revision", 0)) + 1,
            "distance_m": round(float(distance or 0.0), 1),
            "remaining_gap_m": round(float(remaining_gap), 1),
            "terminal_state": terminal_state,
            "position": literal_exit,
            "battery": 100.0,
            "signal": 100.0,
            "speed": 0.0,
            "started_at": time.time(),
        }
    )

    if terminal_state == "WORKER_REACHED":
        log_event(
            f"DRONE-01 dispatched from Exit {exit_index + 1} to {w.get('name', worker_id)} — safe tunnel route selected",
            "critical",
        )
    else:
        log_event(
            f"DRONE-01 dispatched from Exit {exit_index + 1}; worker {w.get('name', worker_id)} is blocked — approaching closest safe tunnel point",
            "critical",
        )
    return True


def _stand_down_drone() -> None:
    drone_state.update(
        {
            "active": False,
            "mission_status": "STANDBY",
            "target_worker_id": None,
            "entry_exit_index": None,
            "entry_exit_label": None,
            "route_status": "IDLE",
            "route_coordinates": [],
            "route_revision": int(drone_state.get("route_revision", 0)) + 1,
            "distance_m": None,
            "remaining_gap_m": None,
            "terminal_state": "STANDBY",
            "position": None,
            "speed": 0.0,
            "started_at": None,
        }
    )


# ---------------------------------------------------------------------------
# Sensor-driven hazard activation
#
# A zone-based emergency mapped in EMERGENCY_SENSOR_MAP does not create a
# second, disconnected emergency system — it is the SAME hazard/Buddy/A*
# workflow as before, just triggered from a sensor crossing CRITICAL
# instead of firing the instant the operator clicks the button. This is
# the one place that actually flips `emergency["active"]` for those types.
# ---------------------------------------------------------------------------


def _activate_zone_emergency(
    etype_key: str,
    zone_center: List[float],
    primary_worker_id: Optional[str] = None,
    radius: Optional[float] = None,
    origin_sensor_id: Optional[str] = None,
) -> None:
    """Must be called with STATE_LOCK already held."""
    etype = EMERGENCY_TYPES[etype_key]
    previous_primary = emergency.get("primary_worker_id")

    emergency.clear()
    emergency["active"] = True
    emergency["type"] = etype_key
    emergency["label"] = etype["label"]
    emergency["color"] = etype["color"]
    emergency["zone_based"] = True
    emergency["action"] = etype["action"]
    emergency["started_at"] = time.time()
    emergency["previous_primary_worker_id"] = previous_primary
    emergency["zone_center"] = list(zone_center)
    emergency["radius"] = radius or HAZARD_RADIUS
    emergency["origin_sensor_id"] = origin_sensor_id

    for w in workers.values():
        update_hazard_flags(w)

    affected_ids = [wid for wid, w in workers.items() if w.get("in_hazard")]
    if not affected_ids and primary_worker_id and primary_worker_id in workers:
        affected_ids = [primary_worker_id]

    if affected_ids:
        zc = tuple(emergency["zone_center"])
        primary_worker_id = min(
            affected_ids,
            key=lambda wid: dist3((workers[wid]["x"], workers[wid]["y"], workers[wid]["z"]), zc),
        )
    emergency["primary_worker_id"] = primary_worker_id
    _set_emergency_affected_flags(affected_ids)
    emergency["workers_affected"] = affected_ids

    _compute_buddy_alerts()

    victim_name = (
        workers.get(primary_worker_id, {}).get("name", primary_worker_id)
        if primary_worker_id else "no worker"
    )
    zone_label = sensors.sensors.get(origin_sensor_id, {}).get("zone") if origin_sensor_id else None

    if origin_sensor_id:
        log_event(f"{zone_label or 'Zone'} marked hazardous — origin sensor {origin_sensor_id}", "critical")
        log_event(
            f"EMERGENCY: {etype['label']} confirmed by {origin_sensor_id} — primary affected worker: {victim_name}",
            "critical",
        )
    else:
        log_event(f"EMERGENCY: {etype['label']} declared — primary affected worker: {victim_name}", "critical")

    if affected_ids:
        names = ", ".join(workers[wid].get("name", wid) for wid in affected_ids)
        log_event(f"Worker(s) identified at risk: {names}", "critical")

    if origin_sensor_id:
        log_event("Route recalculated due to critical sensor alert.", "warning")

    if etype_key == "tunnel_collapse" and primary_worker_id:
        # Tunnel collapse is one of only two events that can launch the
        # drone. The planner already keeps the drone on the safe side of the
        # physical blockage and sends it to the closest reachable point.
        _plan_drone_mission(primary_worker_id)
    else:
        # Gas, CO, fire, low oxygen, rock fall, explosion, water ingress,
        # and equipment incidents are handled by sensor alerts, buddy logic,
        # and evacuation rerouting. No drone launch for those events.
        _stand_down_drone()


def _on_sensor_critical(sensor_id: str) -> None:
    """Called from sensor_loop (STATE_LOCK already held) the instant a
    targeted sensor's reading first crosses its critical threshold."""
    group_id = _pending_sensor_targets.get(sensor_id)
    if not group_id:
        return
    group = _pending_groups.get(group_id)
    if not group or group.get("activated"):
        return
    group["activated"] = True

    s = sensors.sensors.get(sensor_id)
    if not s:
        return
    zone_center = [s["x"], s["y"], s["z"]]
    _activate_zone_emergency(
        group["etype_key"],
        zone_center,
        primary_worker_id=group.get("primary_worker_id"),
        origin_sensor_id=sensor_id,
    )


def sensor_loop() -> None:
    """Background thread: advances every sensor's simulated reading and
    reports NORMAL/WARNING/CRITICAL transitions into the event log. Runs
    independently of the worker simulator — it works whether workers come
    from the built-in simulator or a real watch/telemetry POST."""
    log_event("Sensor Digital Twin online — SIMULATED SENSOR TELEMETRY", "info")
    while True:
        with STATE_LOCK:
            sensors.tick(workers, on_critical=_on_sensor_critical, log_fn=log_event)
        time.sleep(sensors.TICK_SECONDS)


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
            return jsonify({
                "active": False,
                "pending": bool(emergency.get("pending")),
                "type": emergency.get("type"),
                "label": emergency.get("label"),
                "color": emergency.get("color"),
                "origin_sensor_ids": emergency.get("origin_sensor_ids", []),
            })

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
    categories = EMERGENCY_SENSOR_MAP.get(etype_key) if etype["zone_based"] else None

    with STATE_LOCK:
        # Every new dashboard scenario starts from a clean drone state.
        # A fresh mission is created only later if this event is allowed to
        # launch the drone (Manual SOS or Tunnel Collapse).
        _stand_down_drone()

        # Step 1 — choose/identify the actual worker this simulation affects.
        primary_worker_id = _choose_scenario_worker(payload.get("worker_id"))

        if categories:
            # Sensor-driven path: arm the relevant sensor(s) near the chosen
            # worker/zone and let them climb NORMAL -> WARNING -> CRITICAL.
            # No hazard zone, worker impact, buddy alert, or reroute happens
            # yet — that all happens once a sensor actually crosses CRITICAL
            # (see _on_sensor_critical), so the emergency is the sensor's
            # doing, not a second disconnected system.
            if payload.get("zone_center"):
                target_point = tuple(payload["zone_center"])
            elif primary_worker_id and primary_worker_id in workers:
                w = workers[primary_worker_id]
                target_point = (w["x"], w["y"], w["z"])
            else:
                target_point = random.choice(TUNNEL_POINTS)

            chosen_sensor_ids = []
            for cat in categories:
                sid = sensors.nearest_sensor_of_type(cat, target_point)
                if sid:
                    sensors.start_targeting(sid)
                    chosen_sensor_ids.append(sid)

            if chosen_sensor_ids:
                group_id = f"{etype_key}:{time.time()}"
                for sid in chosen_sensor_ids:
                    _pending_sensor_targets[sid] = group_id
                _pending_groups[group_id] = {
                    "etype_key": etype_key,
                    "primary_worker_id": primary_worker_id,
                    "sensor_ids": chosen_sensor_ids,
                    "activated": False,
                }
                emergency.clear()
                emergency["active"] = False
                emergency["pending"] = True
                emergency["type"] = etype_key
                emergency["label"] = etype["label"]
                emergency["color"] = etype["color"]
                emergency["origin_sensor_ids"] = chosen_sensor_ids
                victim_name = (
                    workers.get(primary_worker_id, {}).get("name", primary_worker_id)
                    if primary_worker_id else "the mine"
                )
                log_event(
                    f"{etype['label']} scenario armed — monitoring {', '.join(chosen_sensor_ids)} "
                    f"near {victim_name}'s position",
                    "warning",
                )
                out_emergency = dict(emergency)
                out_drone = dict(drone_state)
                return jsonify({"ok": True, "emergency": out_emergency, "drone": out_drone})
            # No sensor of the required type exists (shouldn't normally
            # happen) — fall through to the immediate-activation path below.

        # Immediate-activation path — used for emergency types with no
        # mapped sensor category (explosion, water_ingress,
        # equipment_failure, manual_sos), and as a fallback above.
        if etype["zone_based"]:
            zone_center = payload.get("zone_center")
            if not zone_center:
                if primary_worker_id and primary_worker_id in workers:
                    w = workers[primary_worker_id]
                    zone_center = [w["x"], w["y"], w["z"]]
                else:
                    zone_center = list(random.choice(TUNNEL_POINTS))
            _activate_zone_emergency(
                etype_key, zone_center, primary_worker_id, radius=payload.get("radius")
            )
        else:
            previous_primary = emergency.get("primary_worker_id")
            emergency.clear()
            emergency["active"] = True
            emergency["type"] = etype_key
            emergency["label"] = etype["label"]
            emergency["color"] = etype["color"]
            emergency["zone_based"] = False
            emergency["action"] = etype["action"]
            emergency["started_at"] = time.time()
            emergency["previous_primary_worker_id"] = previous_primary
            emergency["primary_worker_id"] = primary_worker_id

            affected_ids = _identify_affected_workers(primary_worker_id)
            _set_emergency_affected_flags(affected_ids)
            emergency["workers_affected"] = affected_ids
            _compute_buddy_alerts()

            victim_name = (
                workers.get(primary_worker_id, {}).get("name", primary_worker_id)
                if primary_worker_id else "no worker"
            )
            log_event(
                f"EMERGENCY: {etype['label']} declared — primary affected worker: {victim_name}",
                "critical",
            )

            if etype_key == "manual_sos" and primary_worker_id:
                _plan_drone_mission(primary_worker_id)
            else:
                # Equipment failure and any other non-SOS incident do not
                # justify a drone launch in this prototype.
                _stand_down_drone()

        out_emergency = dict(emergency)
        out_drone = dict(drone_state)

    return jsonify({"ok": True, "emergency": out_emergency, "drone": out_drone})


@app.route("/api/emergency/reset", methods=["POST"])
def api_emergency_reset():
    with STATE_LOCK:
        _pending_sensor_targets.clear()
        _pending_groups.clear()
        sensors.reset_all()

        emergency.clear()
        emergency["active"] = False
        for w in workers.values():
            w["in_hazard"] = False
            w["emergency_affected"] = False
        _compute_buddy_alerts()
        _stand_down_drone()
        log_event("Emergency cleared — all-clear; sensors returning to baseline; DRONE-01 stood down", "info")
    return jsonify({"ok": True})


@app.route("/api/sensors", methods=["GET"])
def api_sensors():
    with STATE_LOCK:
        return jsonify(sensors.snapshot())


@app.route("/api/drone", methods=["GET"])
def api_drone():
    with STATE_LOCK:
        out = dict(drone_state)
        out["sensors"] = _drone_sensor_snapshot()
        target_id = out.get("target_worker_id")
        target = workers.get(target_id) if target_id else None
        if target and out.get("position"):
            out["distance_to_worker_m"] = round(
                dist3(tuple(out["position"]), (target["x"], target["y"], target["z"])), 1
            )
        else:
            out["distance_to_worker_m"] = None
        return jsonify(out)


@app.route("/api/drone/telemetry", methods=["POST"])
def api_drone_telemetry():
    payload = request.get_json(force=True) or {}
    with STATE_LOCK:
        pos = payload.get("position")
        if isinstance(pos, list) and len(pos) == 3:
            drone_state["position"] = [float(pos[0]), float(pos[1]), float(pos[2])]
        for key in ("battery", "signal", "speed"):
            if key in payload:
                drone_state[key] = float(payload[key])
        state = payload.get("mission_status")
        if state:
            drone_state["mission_status"] = str(state)
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




@app.route("/api/localization/<worker_id>", methods=["GET"])
def api_worker_localization(worker_id):
    """Return a worker-specific simulated UWB + IMU localization snapshot.

    UWB anchors are fixed around the mine.  For each request we select the
    three nearest ONLINE anchors to the worker's current location, generate
    a small ranging error, and return the worker-mounted IMU record produced
    by sensors.py.  This keeps the demo tied to the selected worker instead
    of reusing one hard-coded anchor set.
    """
    with STATE_LOCK:
        w = workers.get(worker_id)
        if not w:
            return jsonify({"error": "unknown worker"}), 404

        # Make sure the selected worker has a current wearable-IMU record
        # even if this request arrives before the next background sensor tick.
        if worker_id not in sensors.imu_units:
            sensors._update_imu_units(workers, time.time())

        wx, wy, wz = float(w["x"]), float(w["y"]), float(w["z"])
        candidates = []
        for a in sensors.uwb_anchors.values():
            if a.get("connectivity") != "ONLINE":
                continue
            d = math.sqrt((wx - a["x"]) ** 2 + (wy - a["y"]) ** 2 + (wz - a["z"]) ** 2)
            candidates.append((d, a))
        candidates.sort(key=lambda item: item[0])
        chosen = candidates[:3]

        anchors = []
        for true_d, a in chosen:
            measured = max(0.0, true_d + random.uniform(-0.18, 0.18))
            quality = max(35.0, min(99.9, 99.0 - true_d * 0.06 + random.uniform(-2.0, 2.0)))
            anchors.append({
                "anchor_id": a["anchor_id"],
                "x": a["x"],
                "y": a["y"],
                "z": a["z"],
                "true_distance_m": round(true_d, 2),
                "measured_distance_m": round(measured, 2),
                "signal_quality": round(quality, 1),
                "connectivity": a.get("connectivity", "ONLINE"),
            })

        # The prototype's displayed estimate is intentionally close to the
        # known simulated worker position, with small noise to make the
        # trilateration story visible without pretending hardware precision.
        estimated = {
            "x": round(wx + random.uniform(-0.28, 0.28), 2),
            "y": round(wy + random.uniform(-0.28, 0.28), 2),
            "z": round(wz + random.uniform(-0.28, 0.28), 2),
        }
        imu = dict(sensors.imu_units.get(worker_id, {
            "imu_id": f"IMU-{worker_id}",
            "worker_id": worker_id,
            "motion_state": "UNKNOWN",
            "connectivity": "ONLINE",
            "simulated": True,
        }))

        return jsonify({
            "worker_id": worker_id,
            "worker_name": w.get("name", worker_id),
            "zone": w.get("zone"),
            "anchors": anchors,
            "imu": imu,
            "estimated_position": estimated,
            "method": "UWB trilateration + worker-mounted IMU sensor fusion",
            "accuracy_label": "SIMULATED PROTOTYPE DATA",
            "server_time": time.time(),
        })


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
