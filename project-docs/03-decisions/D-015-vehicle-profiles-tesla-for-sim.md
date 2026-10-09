# D-015 · Vehicle profiles: the Tesla in the sim, placeholders for the cart

- **Date:** 2026-10-08
- **Status:** Accepted

All geometry comes from a `VehicleParams` profile chosen by the `vehicle`
parameter. `tesla_sim` uses the PROTO's real dimensions so collision checks
match the simulated body; `cart` has placeholders until the owner supplies
dimensions. Steering limits (±15°, 12.86 °/s) are shared.
