# D-009 · A relay-aware arc planner with a receding horizon

- **Date:** 2026-10-08
- **Status:** Accepted

21 candidates of the form "steer towards T at 12.86 °/s from the measured
angle, then hold", rolled out 10 m and scored on the memory map (collision,
clearance, unknown, route or heading, smoothness, straightness); re-planned
at 10 Hz. Planning steering *changes*, not jumps, matches the relay. Simple
arcs re-planned often give smooth, feedback-corrected paths. Chosen over
Nav2 controllers ([D-001](D-001-custom-stack-not-nav2.md)).
