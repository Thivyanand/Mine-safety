# MineGuard — Underground Mine Safety Digital Twin

> **An intelligent digital-twin prototype for real-time underground worker safety, emergency response, and hazard-aware evacuation.**

MineGuard is an early-stage **underground mine safety Digital Twin** designed to provide control-room operators with a live operational view of workers, hazards, emergency events, and evacuation routes.

The current prototype focuses on the core safety workflow:

**SENSE → UNDERSTAND → UPDATE → IDENTIFY RISK → ROUTE → RESPOND**

The system is intentionally implemented without physical hardware or drones at this stage. Worker telemetry and emergency conditions are simulated so that the core decision-making and digital-twin workflow can be demonstrated clearly.

---

## The Problem

Underground mining environments are dynamic and difficult to monitor.

During an emergency:

- Worker locations may change continuously.
- Tunnel sections can become unsafe or inaccessible.
- Static evacuation plans may become invalid.
- Control-room operators must combine information from multiple sources.
- Sending another worker into an uncertain or hazardous area can increase the risk.
- Emergency response decisions can become slow when information is fragmented.

MineGuard addresses this by creating a **live digital representation of the underground environment** and using it to support emergency decisions.

---

#  What MineGuard Does

MineGuard brings together:

-  Worker monitoring
-  Worker location visualization
-  SOS and fall events
-  Hazard-zone visualization
-  Buddy-based emergency response
-  Dynamic evacuation routing
-  Alternative route calculation
-  Centralized control-room dashboard
-  Real-time event logging

Instead of simply displaying an emergency, MineGuard attempts to answer:

> **"What is the safest action that can be taken right now?"**

---

# Core Architecture

```text
                    ┌─────────────────────┐
                    │ Worker / Watch      │
                    │ Telemetry Simulator │
                    └──────────┬──────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │   MineGuard Backend │
                    │   Event Processing  │
                    └──────────┬──────────┘
                               │
              ┌────────────────┼────────────────┐
              │                │                │
              ▼                ▼                ▼
       Worker State       Hazard State      Event Log
              │                │                │
              └────────────────┼────────────────┘
                               ▼
                    ┌─────────────────────┐
                    │   Digital Twin      │
                    │ Underground Mine    │
                    └──────────┬──────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │   Risk / Routing    │
                    │      Engine         │
                    └──────────┬──────────┘
                               │
                         A* Safe Route
                               │
                               ▼
                    ┌─────────────────────┐
                    │ Buddy Trigger       │
                    │ + Evacuation Logic  │
                    └──────────┬──────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │ Control Room        │
                    │ Safety Dashboard    │
                    └─────────────────────┘
