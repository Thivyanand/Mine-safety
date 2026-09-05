<<<<<<< HEAD
# Mine-safety
=======
MineGuard Prototype (No Drone)

Short description
-----------------

# Mine-safety

## MineGuard Digital Twin — Beginning Prototype

### Overview
MineGuard is an early-stage underground mine safety digital-twin prototype. It focuses on bringing worker status, emergency events, buddy alerts, and safe evacuation routing into one control-room dashboard.

This version intentionally keeps the scope small so the core concept can be demonstrated clearly before adding more advanced hardware integrations.

### Current Prototype Features
- 3D underground mine visualization
- Worker roster and live worker markers
- Worker telemetry through the watch simulator
- Manual SOS and fall detection inputs
- Emergency simulation for mine hazards
- Hazard-zone visualization
- Buddy Trigger System
- A* based safe evacuation routing
- Alternative route calculation when a route is blocked
- Event log and worker detail panel

### Core Workflow
1. Worker telemetry or a simulated mine emergency is received.
2. The digital twin updates the affected worker and hazard zone.
3. The system identifies the actual worker affected by the event.
4. The Buddy Trigger System alerts nearby reachable workers.
5. A* calculates a safe route to an exit or refuge while avoiding blocked or hazardous graph nodes.
6. The control-room dashboard shows the worker status, hazard, buddy alert, and evacuation route.

### Main Algorithm — A* Search
The mine tunnel network is represented as a weighted graph. Tunnel points are nodes and valid tunnel connections are edges. A* searches for the shortest currently reachable evacuation route while excluding blocked or hazardous graph nodes.

### Buddy Trigger System
When a worker triggers SOS, a fall is detected, or the worker is affected by an active hazard, the system checks nearby workers using tunnel-graph walking distance. Only reachable workers are considered valid buddies, so the alert does not suggest a path through an active hazard.

### Prototype vs Real Deployment
**Prototype**
- Worker telemetry is simulated through the included watch page.
- Emergency conditions are triggered from the dashboard.
- Worker positions and evacuation routes are shown in the 3D mine model.

**Real-world integration path**
A later deployment could connect the same digital-twin layer to existing mine infrastructure such as gas sensors, vibration sensors, temperature/airflow sensors, UWB or RFID worker-location systems, industrial Wi-Fi or mesh networks, leaky-feeder systems, and Ethernet/fiber backbones.

## Quick start

- Create a virtual environment and activate it:

```bash
python -m venv venv
source venv/bin/activate
```

- Install dependencies:

```bash
pip install -r requirements.txt
```

- Run the app:

```bash
python app.py
```

- Open a browser at `http://localhost:5000` (or the address printed by the app).

## Run (alternative)
```bash
pip install -r requirements.txt
python app.py
```

Open the dashboard at `http://127.0.0.1:5000` and the worker watch simulator at `http://127.0.0.1:5000/watch`.

## License
This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

Notes
-----

- If `pywebview` GTK backends are unavailable on Linux, the app should fall back to server-only mode.
