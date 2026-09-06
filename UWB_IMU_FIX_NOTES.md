# UWB + IMU Localization Fix

## What was fixed

- Replaced the sparse six-anchor UWB layout with **24 fixed UWB anchors** distributed across the mine using deterministic farthest-point placement.
- Worker localization now requests the **three nearest online anchors to the selected worker's current position** through `/api/localization/<worker_id>`.
- Different workers therefore use different local anchor sets whenever their locations differ.
- Added animated UWB ranging pulses between the selected worker and those exact anchors.
- Added one **worker-mounted IMU digital twin per worker** (not fixed around the mine).
- IMU telemetry includes acceleration, gyroscope, heading, motion state, fall state, connectivity, and update time.
- The Worker Intelligence panel now displays live simulated IMU values.
- Localization demo now follows the technically correct sequence:

  `fixed UWB anchors -> range measurements -> UWB trilateration + worker-mounted IMU -> XYZ worker location -> A* route planning`

## Prototype accuracy note

All UWB/IMU values are simulated prototype telemetry. No real UWB or IMU hardware is connected.
