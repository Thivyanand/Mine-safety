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
