# Drone Trigger Policy Fix

## Final dispatch rule
The rescue drone can start only for:

1. **Manual SOS / direct watch SOS**
2. **Tunnel Collapse**

All other emergency simulations keep the drone in `STANDBY`.

## Behavior
- **Manual SOS:** target the actual SOS worker and choose the shortest safe A* entry route.
- **Tunnel Collapse:** target the actual affected worker, treat the collapse as a physical blockage, fly only to the closest safely reachable point, then use `STANDBY_NEAR_BLOCKAGE`.
- **Gas Leak, CO Leak, Fire, Explosion, Rock Fall, Low Oxygen, Water Ingress, Equipment Failure:** no drone launch. Sensor twin, Buddy Trigger, hazard mapping, and A* evacuation routing continue normally.
- **Fall without SOS:** no drone launch. Buddy/control-room alerts still continue.

## Code changes
- Direct telemetry dispatch now checks `sos` only; `fall` no longer launches the drone.
- Every new dashboard emergency starts from a clean drone standby state.
- Sensor-driven zone emergencies call the drone planner only when the emergency type is `tunnel_collapse`.
- Non-zone dashboard emergencies call the drone planner only when the emergency type is `manual_sos`.
