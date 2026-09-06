# Sensor Map Visibility Update

This build keeps the 3D mine clean by default.

- All sensor markers start **hidden**.
- Click a sensor category in **Sensor Network** to show only that category.
- Click the same category again to hide it.
- UWB anchors use the same show/hide behavior.
- Hidden sensors cannot be clicked in the 3D view and their labels remain hidden.
- Worker live locations now use a dedicated **cyan/blue** visual treatment so they do not blend with green NORMAL sensors.
- Sensor status colors remain Green / Yellow / Red / Grey when a category is enabled.

This is a UI-only visibility/filtering change; sensor telemetry, emergency logic, Buddy Trigger, and A* routing are preserved.
