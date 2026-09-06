# MineGuard Emergency Logic Fixes

## Fixed
- Dashboard emergency simulation no longer auto-targets the first roster worker (J. Vance).
- Backend first identifies a primary affected worker and uses that same worker for the incident location, critical state, rescue route, and drone target.
- Zone emergencies are anchored on the chosen incident worker unless an explicit API zone center is supplied.
- Only workers tied to the current emergency are included in `workers_affected`; unrelated abnormal vitals no longer become emergency victims.
- Rescue drone checks all 3 exits and uses the shortest A* route to the identified worker.
- Tunnel Collapse is treated as a physical blockage: the drone approaches the closest safe reachable point and stays in `STANDBY_NEAR_BLOCKAGE`.
- Other inspection emergencies allow the drone to reach the affected worker instead of treating every hazard zone as a physical wall.
- Dashboard automatically focuses the actual primary affected worker when a new emergency starts.

## Verification
Core backend logic was exercised against all major scenarios using the five watch-simulator workers. Verified:
- multiple emergency simulations do not always target Vance;
- `primary_worker_id == drone.target_worker_id`;
- direct emergencies use the shortest available A* entry route;
- Tunnel Collapse terminates in `STANDBY_NEAR_BLOCKAGE` at the closest safe approach.


## Drone dispatch policy update
- Drone starts only for **Manual SOS/direct watch SOS** and **Tunnel Collapse**.
- A fall without SOS no longer launches the drone.
- Gas, CO, fire, explosion, rock fall, low oxygen, water ingress, and equipment failure keep the drone in standby.
- Tunnel Collapse still uses closest-safe-approach behavior and `STANDBY_NEAR_BLOCKAGE`.
