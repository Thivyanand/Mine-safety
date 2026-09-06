# MineGuard Digital Twin — Final Hackathon Build

MineGuard is a real-time underground mine safety and rescue-support prototype. It combines worker telemetry, a 3D mine digital twin, emergency detection, hazard-aware routing, and an autonomous rescue-drone simulation.

## Core algorithm used in this final build

### Hazard-Aware A* Search
The mine tunnel network is represented as a weighted graph. Tunnel points are nodes and valid tunnel connections are edges. A* finds the shortest currently reachable tunnel route from a worker or drone entry point to a target.

A* uses:

- `g(n)` — actual route distance already travelled
- `h(n)` — straight-line 3D distance from the current node to the destination
- `f(n) = g(n) + h(n)` — priority used by the search

Hazard or collapse nodes are excluded from the search. If the mine state changes, the route is recalculated using the updated graph.

## Other algorithms / methods in the project

- **Graph / adjacency list** — represents the tunnel network.
- **K-nearest-neighbor + distance threshold** — builds graph connections from the tunnel point cloud.
- **BFS connectivity check** — checks whether graph regions are connected.
- **Hazard-radius filtering** — disables tunnel nodes inside an active hazard zone.
- **A* rerouting** — recalculates a route after a blockage or emergency-state change.
- **Threshold / weighted risk scoring** — current hackathon logic for worker and emergency risk classification.

## Real-world algorithm stack

The hackathon build demonstrates the decision and routing layer. A production mine deployment can connect it to:

- **UWB trilateration / TDoA** for underground worker positioning.
- **Extended Kalman Filter (EKF)** for smoothing UWB + IMU location estimates.
- **LiDAR-Inertial SLAM** for autonomous drone localization and mapping underground.
- **RRT* or another local 3D planner** for obstacle-aware local drone movement.
- **PID / MPC** for real drone flight control.
- **Threshold safety rules + anomaly detection** for mine-sensor monitoring.

## Main emergency flow

1. Worker telemetry or an emergency event reaches the control room.
2. The digital twin updates the worker and hazard state.
3. Hazard nodes are removed from the traversable mine graph.
4. A* calculates a safe reachable route.
5. The best available drone entry is selected.
6. The rescue drone follows the tunnel route.
7. If the route becomes blocked, A* recalculates the path.
8. If the worker is unreachable, the drone moves to the closest safe reachable tunnel point and enters standby.

## Run on Windows

```bat
py -m pip install -r requirements.txt
py app.py
```

To run without the desktop pywebview window:

```bat
py app.py --no-browser
```

Open:

`http://127.0.0.1:5000`

Worker watch simulator:

`http://127.0.0.1:5000/watch`

## Prototype limitations

- Worker and environmental telemetry are simulated for the hackathon.
- Drone movement is a 3D software simulation, not a real flight controller.
- The current build does not implement real UWB, SLAM, LiDAR, ROS, or mine-certified networking.
- Those systems are the intended production integrations around the same digital-twin and routing architecture.


## Emergency Dispatch Consistency (Bug-Fixed)

The emergency simulator now uses a single source of truth for the incident worker:

1. The backend first chooses/identifies the worker actually affected by the simulated emergency.
2. Zone emergencies are anchored on that worker, so the hazard marker and worker status match.
3. For drone-eligible events only (`manual_sos` and `tunnel_collapse`), the same `primary_worker_id` becomes the rescue-drone target.
4. For a Manual SOS, A* evaluates all three mine exits and dispatches from the shortest safe route.
5. For `tunnel_collapse`, the collapse region is treated as a physical blockage; the drone approaches the closest safely reachable graph point and enters `STANDBY_NEAR_BLOCKAGE` instead of flying through the collapsed tunnel.
6. The dashboard no longer auto-targets the first roster worker (J. Vance) just because that worker is selected for the detail panel. Only workers truly tied to the active emergency are marked critical by the emergency state.

### Drone behavior by emergency type

The drone is intentionally reserved for only two scenarios:

- **Manual SOS:** dispatch to the SOS worker using the shortest safe A* route.
- **Tunnel Collapse:** dispatch toward the affected worker, but stop at the closest safely reachable point and enter `STANDBY_NEAR_BLOCKAGE` because the collapsed passage is physically blocked.
- **Gas / CO / Low Oxygen / Fire / Explosion / Rock Fall / Water Ingress / Equipment Failure:** **NO DRONE DISPATCH**. These incidents are handled by sensor alerts, hazard mapping, Buddy Trigger, and evacuation rerouting.
- **Fall alert without SOS:** **NO DRONE DISPATCH**. Buddy/control-room safety logic still operates.

## UWB + IMU localization demo update

The localization demo now uses 24 fixed UWB anchors distributed across the mine. For any selected worker, the backend chooses the three nearest online anchors based on that worker's current XYZ position and returns simulated range measurements. Each worker also has a worker-mounted IMU digital twin with acceleration, gyroscope, heading, motion state and fall status. The demo combines UWB ranging and IMU motion data before A* route planning. All localization telemetry is simulated prototype data.
