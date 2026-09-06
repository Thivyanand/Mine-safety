"""
sensors.py — Sensor Digital Twin layer for the Mine Safety Control Room.

Adds fixed Gas / Oxygen / Temperature / Vibration / Structural-Displacement
sensors and UWB worker-positioning anchors, placed on real points from the
tunnel point cloud. Every value produced here is SIMULATED SENSOR
TELEMETRY — no real hardware is connected, and nothing in this module
claims otherwise.

This module intentionally does NOT run its own hazard/emergency system.
It only tracks sensor readings and reports transitions (NORMAL -> WARNING
-> CRITICAL) back to dashboard_base.py via the `on_critical` callback
passed into tick(); dashboard_base.py is the single source of truth for
hazard zones, worker impact, the Buddy Trigger System, and A* routing.

All functions that mutate module state are called by dashboard_base.py
while it already holds STATE_LOCK — this module has no lock of its own.
"""

from __future__ import annotations

import math
import random
import time
from typing import Callable, Dict, List, Optional, Tuple

Coord = Tuple[float, float, float]

# ---------------------------------------------------------------------------
# Sensor type definitions
# ---------------------------------------------------------------------------

SENSOR_TYPE_DEFS = {
    "gas": {
        "label": "Gas",
        "unit": "%",
        "baseline": 0.31,
        "warning_threshold": 0.50,
        "critical_threshold": 1.00,
        "critical_target": 1.35,
        "wander": 0.01,
        "rising": True,
    },
    "oxygen": {
        "label": "Oxygen",
        "unit": "%",
        "baseline": 20.8,
        "warning_threshold": 19.5,
        "critical_threshold": 19.0,
        "critical_target": 17.2,
        "wander": 0.05,
        "rising": False,  # danger direction is DOWN
    },
    "temperature": {
        "label": "Temperature",
        "unit": "°C",
        "baseline": 28.0,
        "warning_threshold": 34.0,
        "critical_threshold": 42.0,
        "critical_target": 52.0,
        "wander": 0.10,
        "rising": True,
    },
    "vibration": {
        "label": "Vibration",
        "unit": "mm/s",
        "baseline": 0.20,
        "warning_threshold": 0.50,
        "critical_threshold": 1.00,
        "critical_target": 1.60,
        "wander": 0.02,
        "rising": True,
    },
    "structural": {
        "label": "Structural Displacement",
        "unit": "mm",
        "baseline": 0.40,
        "warning_threshold": 2.00,
        "critical_threshold": 5.00,
        "critical_target": 8.00,
        "wander": 0.05,
        "rising": True,
    },
}

SENSOR_ID_PREFIX = {
    "gas": "GAS",
    "oxygen": "O2",
    "temperature": "TEMP",
    "vibration": "VIB",
    "structural": "STRUCT",
}

SENSOR_COUNTS = {"gas": 6, "oxygen": 5, "temperature": 5, "vibration": 4, "structural": 4}
UWB_ANCHOR_COUNT = 24  # denser fixed anchor network for worker-specific localization

DETERIORATION_SECONDS = 9.0   # baseline -> critical_target, once a sensor is "targeted"
RECOVERY_SECONDS = 5.0        # critical/warning -> baseline, once reset
TICK_SECONDS = 1.5

OFFLINE_CHANCE_PER_TICK = 0.0012   # small chance an ONLINE, non-targeted sensor drops out
RECOVERY_CHANCE_PER_TICK = 0.15    # chance an OFFLINE sensor comes back per tick
UWB_NEARBY_RADIUS = 220.0

# ---------------------------------------------------------------------------
# State — populated once by build_sensors(), mutated by tick()
# ---------------------------------------------------------------------------

sensors: Dict[str, dict] = {}
uwb_anchors: Dict[str, dict] = {}
imu_units: Dict[str, dict] = {}
_imu_prev_pos: Dict[str, Coord] = {}
_imu_prev_time: Dict[str, float] = {}
_imu_yaw: Dict[str, float] = {}


def _zone_label(i: int) -> str:
    letters = "ABCDE"
    return f"Tunnel {letters[i % len(letters)]}"


def build_sensors(tunnel_points: List[Coord]) -> None:
    """Place a fixed, deterministic sensor network across the mine using
    the same point cloud the tunnel graph is built from, so every sensor
    sits on real tunnel geometry. Called once at startup."""
    sensors.clear()
    uwb_anchors.clear()
    imu_units.clear()
    _imu_prev_pos.clear()
    _imu_prev_time.clear()
    _imu_yaw.clear()
    if not tunnel_points:
        return
    n = len(tunnel_points)

    seq = 0
    idx = 0
    total = sum(SENSOR_COUNTS.values())
    stride = max(1, n // max(1, total))
    for stype, count in SENSOR_COUNTS.items():
        defs = SENSOR_TYPE_DEFS[stype]
        for i in range(count):
            pt = tunnel_points[idx % n]
            idx += stride + 7  # de-correlate types so they don't all land on the same points
            seq += 1
            sid = f"{SENSOR_ID_PREFIX[stype]}-{i + 1:02d}"
            sensors[sid] = {
                "sensor_id": sid,
                "sensor_type": stype,
                "label": defs["label"],
                "zone": _zone_label(seq),
                "x": pt[0],
                "y": pt[1],
                "z": pt[2],
                "reading": round(defs["baseline"], 3),
                "unit": defs["unit"],
                "warning_threshold": defs["warning_threshold"],
                "critical_threshold": defs["critical_threshold"],
                "status": "NORMAL",
                "connectivity": "ONLINE",
                "last_updated": time.time(),
                "targeted": False,
                "target_started_at": None,
                "target_start_value": None,
                "target_final_value": None,
                "recovering": False,
                "recover_started_at": None,
                "recover_start_value": None,
            }

    # UWB anchors are FIXED infrastructure.  Use deterministic farthest-point
    # placement across the mine point cloud instead of a simple stride.  The
    # old six-anchor layout left most workers using the same three anchors;
    # this denser, spatially distributed network lets each worker naturally
    # range against the nearest local anchors.
    candidate_step = max(1, n // 320)
    candidates = tunnel_points[::candidate_step]
    if tunnel_points[-1] not in candidates:
        candidates.append(tunnel_points[-1])

    selected: List[Coord] = []
    if candidates:
        # deterministic first anchor: point with smallest X, then greedily
        # choose the point farthest from all already selected anchors.
        selected.append(min(candidates, key=lambda p: (p[0], p[1], p[2])))
        while len(selected) < min(UWB_ANCHOR_COUNT, len(candidates)):
            best_pt = None
            best_min_d2 = -1.0
            for pt in candidates:
                if pt in selected:
                    continue
                min_d2 = min(
                    (pt[0] - q[0]) ** 2 + (pt[1] - q[1]) ** 2 + (pt[2] - q[2]) ** 2
                    for q in selected
                )
                if min_d2 > best_min_d2:
                    best_min_d2 = min_d2
                    best_pt = pt
            if best_pt is None:
                break
            selected.append(best_pt)

    for i, pt in enumerate(selected):
        aid = f"UWB-A{i + 1:02d}"
        uwb_anchors[aid] = {
            "anchor_id": aid,
            "sensor_type": "uwb",
            "x": pt[0],
            "y": pt[1],
            "z": pt[2],
            "nearby_worker": None,
            "signal_quality": round(random.uniform(90, 99), 1),
            "connectivity": "ONLINE",
            "last_updated": time.time(),
        }


def _status_for(stype: str, value: float) -> str:
    defs = SENSOR_TYPE_DEFS[stype]
    if defs["rising"]:
        if value >= defs["critical_threshold"]:
            return "CRITICAL"
        if value >= defs["warning_threshold"]:
            return "WARNING"
        return "NORMAL"
    else:
        if value <= defs["critical_threshold"]:
            return "CRITICAL"
        if value <= defs["warning_threshold"]:
            return "WARNING"
        return "NORMAL"


def nearest_sensor_of_type(stype: str, point: Coord) -> Optional[str]:
    best_id, best_d = None, None
    for sid, s in sensors.items():
        if s["sensor_type"] != stype:
            continue
        d = math.sqrt((s["x"] - point[0]) ** 2 + (s["y"] - point[1]) ** 2 + (s["z"] - point[2]) ** 2)
        if best_d is None or d < best_d:
            best_d, best_id = d, sid
    return best_id


def start_targeting(sensor_id: str) -> None:
    """Begin a gradual, deterministic NORMAL -> WARNING -> CRITICAL climb
    for this sensor. Called once when a dashboard emergency scenario is
    armed against it."""
    s = sensors.get(sensor_id)
    if not s:
        return
    defs = SENSOR_TYPE_DEFS[s["sensor_type"]]
    s["targeted"] = True
    s["recovering"] = False
    s["target_started_at"] = time.time()
    s["target_start_value"] = s["reading"]
    s["target_final_value"] = defs["critical_target"]
    s["connectivity"] = "ONLINE"  # a sensor that is the source of an incident must be reporting


def stop_targeting_and_recover(sensor_id: str) -> None:
    s = sensors.get(sensor_id)
    if not s:
        return
    s["targeted"] = False
    s["recovering"] = True
    s["recover_started_at"] = time.time()
    s["recover_start_value"] = s["reading"]


def reset_all() -> None:
    for s in sensors.values():
        defs = SENSOR_TYPE_DEFS[s["sensor_type"]]
        s["targeted"] = False
        s["recovering"] = False
        s["target_started_at"] = None
        s["reading"] = round(defs["baseline"], 3)
        s["status"] = "NORMAL"
        s["connectivity"] = "ONLINE"
        s["last_updated"] = time.time()
    for a in uwb_anchors.values():
        a["connectivity"] = "ONLINE"
    for imu in imu_units.values():
        imu["connectivity"] = "ONLINE"
        imu["fall_detected"] = False
        imu["motion_state"] = "STANDING"
        imu["last_updated"] = time.time()


def _update_imu_units(workers: Dict[str, dict], now: float) -> None:
    """Maintain one simulated wearable IMU per live worker.

    The IMU is attached to the worker, not installed in the tunnel.  Motion
    values are derived from the worker's change in position so each selected
    worker shows its own live IMU data instead of a random generic badge.
    """
    live_ids = set(workers.keys())
    for wid in list(imu_units.keys()):
        if wid not in live_ids:
            imu_units.pop(wid, None)
            _imu_prev_pos.pop(wid, None)
            _imu_prev_time.pop(wid, None)
            _imu_yaw.pop(wid, None)

    for wid, w in workers.items():
        pos = (float(w.get("x", 0.0)), float(w.get("y", 0.0)), float(w.get("z", 0.0)))
        prev = _imu_prev_pos.get(wid, pos)
        prev_t = _imu_prev_time.get(wid, now - TICK_SECONDS)
        dt = max(0.05, now - prev_t)
        dx, dy, dz = pos[0] - prev[0], pos[1] - prev[1], pos[2] - prev[2]
        speed = math.sqrt(dx * dx + dy * dy + dz * dz) / dt

        yaw = _imu_yaw.get(wid, 0.0)
        if abs(dx) + abs(dz) > 1e-4:
            yaw = (math.degrees(math.atan2(dz, dx)) + 360.0) % 360.0
        _imu_yaw[wid] = yaw

        if w.get("fall"):
            motion = "FALL DETECTED"
        elif speed > 0.35:
            motion = "WALKING"
        elif speed > 0.08:
            motion = "MOVING"
        else:
            motion = "STANDING"

        # Small simulated acceleration/gyro values around the motion state.
        # These are prototype telemetry, not hardware-grade inertial output.
        move_scale = min(1.5, speed / 2.0)
        ax = random.uniform(-0.08, 0.08) + (dx / dt) * 0.02
        ay = 1.0 + random.uniform(-0.03, 0.03)  # ~1 g incl. gravity
        az = random.uniform(-0.08, 0.08) + (dz / dt) * 0.02
        gx = random.uniform(-2.0, 2.0) * (0.3 + move_scale)
        gy = random.uniform(-2.5, 2.5) * (0.3 + move_scale)
        gz = random.uniform(-2.0, 2.0) * (0.3 + move_scale)

        imu_units[wid] = {
            "imu_id": f"IMU-{wid}",
            "worker_id": wid,
            "sensor_type": "imu",
            "ax_g": round(ax, 3),
            "ay_g": round(ay, 3),
            "az_g": round(az, 3),
            "gyro_x_dps": round(gx, 2),
            "gyro_y_dps": round(gy, 2),
            "gyro_z_dps": round(gz, 2),
            "yaw_deg": round(yaw, 1),
            "pitch_deg": round(random.uniform(-4.0, 4.0), 1),
            "roll_deg": round(random.uniform(-5.0, 5.0), 1),
            "speed_mps": round(speed, 2),
            "motion_state": motion,
            "fall_detected": bool(w.get("fall")),
            "connectivity": "ONLINE",
            "last_updated": now,
            "simulated": True,
        }
        _imu_prev_pos[wid] = pos
        _imu_prev_time[wid] = now


def tick(
    workers: Dict[str, dict],
    on_critical: Optional[Callable[[str], None]] = None,
    log_fn: Optional[Callable[[str, str], None]] = None,
) -> None:
    """Advance every sensor by one simulated step. Must be called with the
    caller's state lock already held (this module has none of its own)."""
    now = time.time()
    _update_imu_units(workers, now)

    for sid, s in sensors.items():
        defs = SENSOR_TYPE_DEFS[s["sensor_type"]]

        # Connectivity flicker — never applied to a sensor mid-incident, so a
        # sensor already driving an active scenario can't vanish on the demo.
        if not s["targeted"]:
            if s["connectivity"] == "ONLINE" and random.random() < OFFLINE_CHANCE_PER_TICK:
                s["connectivity"] = "OFFLINE"
                if log_fn:
                    log_fn(f"{sid} — Sensor Communication Lost", "warning")
            elif s["connectivity"] == "OFFLINE" and random.random() < RECOVERY_CHANCE_PER_TICK:
                s["connectivity"] = "ONLINE"
                if log_fn:
                    log_fn(f"{sid} back online", "info")

        if s["connectivity"] == "OFFLINE":
            continue  # keep last known reading visible; don't advance it

        if s["targeted"]:
            elapsed = now - (s["target_started_at"] or now)
            t = min(1.0, elapsed / DETERIORATION_SECONDS)
            base = s["target_start_value"] + (s["target_final_value"] - s["target_start_value"]) * t
            value = base + random.uniform(-defs["wander"], defs["wander"]) * 0.3
        elif s["recovering"]:
            elapsed = now - (s["recover_started_at"] or now)
            t = min(1.0, elapsed / RECOVERY_SECONDS)
            base = s["recover_start_value"] + (defs["baseline"] - s["recover_start_value"]) * t
            if t >= 1.0:
                s["recovering"] = False
            value = base + random.uniform(-defs["wander"], defs["wander"])
        else:
            # Gentle random walk, pulled back toward baseline so it never drifts.
            value = s["reading"] + random.uniform(-defs["wander"], defs["wander"])
            value += (defs["baseline"] - value) * 0.05

        prev_status = s["status"]
        new_status = _status_for(s["sensor_type"], value)
        s["reading"] = round(value, 3)
        s["status"] = new_status
        s["last_updated"] = now

        if s["targeted"] and log_fn:
            if new_status == "WARNING" and prev_status == "NORMAL":
                log_fn(f"{sid} entered WARNING", "warning")
            if new_status == "CRITICAL" and prev_status != "CRITICAL":
                log_fn(f"{sid} reading increased to critical levels", "critical")
                if on_critical:
                    on_critical(sid)

    for aid, a in uwb_anchors.items():
        a["signal_quality"] = max(35.0, min(99.9, a["signal_quality"] + random.uniform(-1.5, 1.5)))
        a["last_updated"] = now
        best_id, best_d = None, None
        for wid, w in workers.items():
            d = math.sqrt(
                (w.get("x", 0.0) - a["x"]) ** 2
                + (w.get("y", 0.0) - a["y"]) ** 2
                + (w.get("z", 0.0) - a["z"]) ** 2
            )
            if best_d is None or d < best_d:
                best_d, best_id = d, wid
        a["nearby_worker"] = best_id if (best_id is not None and best_d is not None and best_d < UWB_NEARBY_RADIUS) else None


def snapshot() -> dict:
    out_sensors = []
    by_type: Dict[str, dict] = {}
    totals = {"total": 0, "online": 0, "warning": 0, "critical": 0, "offline": 0}

    for sid, s in sensors.items():
        out_sensors.append(dict(s))
        totals["total"] += 1
        cat = by_type.setdefault(s["sensor_type"], {"label": SENSOR_TYPE_DEFS[s["sensor_type"]]["label"],
                                                      "normal": 0, "warning": 0, "critical": 0, "offline": 0})
        if s["connectivity"] == "OFFLINE":
            totals["offline"] += 1
            cat["offline"] += 1
        else:
            totals["online"] += 1
            key = s["status"].lower()
            cat[key] = cat.get(key, 0) + 1
            if s["status"] == "WARNING":
                totals["warning"] += 1
            elif s["status"] == "CRITICAL":
                totals["critical"] += 1

    uwb_out = [dict(a) for a in uwb_anchors.values()]
    uwb_online = sum(1 for a in uwb_anchors.values() if a["connectivity"] == "ONLINE")
    imu_out = [dict(imu) for imu in imu_units.values()]
    imu_online = sum(1 for imu in imu_units.values() if imu.get("connectivity") == "ONLINE")

    return {
        "sensors": out_sensors,
        "uwb_anchors": uwb_out,
        "imu_units": imu_out,
        "by_type": by_type,
        "totals": totals,
        "uwb_online": uwb_online,
        "uwb_total": len(uwb_anchors),
        "imu_online": imu_online,
        "imu_total": len(imu_units),
        "simulated": True,
    }
