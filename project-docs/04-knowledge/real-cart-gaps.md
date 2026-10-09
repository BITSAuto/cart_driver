# Real-cart gaps

What stands between this stack and the real cart (2026-10-09):

| Gap | Why it matters | Status |
| --- | --- | --- |
| **No measured speed** (`/vehicle/speed`) | speed hold, odometry, memory map and the safety threshold all need it | open; an early owner message said there are no wheel encoders ([Q-003](../06-open-questions/Q-003-real-cart-speed-source.md)) |
| **No world position fix** | route following (step 5) | open ([Q-004](../06-open-questions/Q-004-world-fix-for-routes.md)) |
| **Cart dimensions** | footprint, turning, camera position | owner to supply ([Q-001](../06-open-questions/Q-001-cart-dimensions.md)) |
| **Coast deceleration** | emergency-brake threshold | unmeasured ([Q-002](../06-open-questions/Q-002-real-coast-deceleration.md)) |
| **Throttle mapping** | open-loop byte scale unverified (PI corrects only with measured speed) | vehicle_bridge Q-001 |
| **Bridge has no command timeout** | if the driver dies, the cart keeps its throttle | vehicle_bridge Q-006 (high) |
| **Encoder calibration** | `/steering/angle` accuracy | vehicle_bridge Q-003 |
| **No launch file or config for the cart** | only `sim.launch.py` and `config/tesla_sim.yaml` exist | roadmap |
| **Never run on the Orin** | Python planner cost unknown there | [Q-009](../06-open-questions/Q-009-performance-on-the-orin.md) |
| **Campus loop route** | needs recording with a position source | [Q-005](../06-open-questions/Q-005-campus-loop-route.md) |
