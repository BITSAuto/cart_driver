# D-006 · PI speed hold on measured speed, output clamped to ≥ 0

- **Date:** 2026-10-08
- **Status:** Accepted

The real throttle is open-loop and uncalibrated, so the driver closes the
loop on measured speed (`kp` 0.5, `ki` 0.3, anti-windup, 0–7 km/h command).
Sim result: 5.13 km/h average (5.06–5.25) in step 2, 5.00 in step 5.
**Requires a measured speed**, which the real cart doesn't have yet.
