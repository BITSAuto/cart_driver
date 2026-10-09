# D-002 · Fixed 5 km/h, throttle only; slow by coasting

- **Date:** 2026-10-08
- **Status:** Accepted (owner's constraint)

The cart's brake stops it instantly and can damage it, so normal driving
never brakes. Hold 5 km/h ("so all turns are possible") with throttle;
release the throttle and let friction coast the cart to slow or stop. The
driver's speed command is always ≥ 0, so it can't express braking.
