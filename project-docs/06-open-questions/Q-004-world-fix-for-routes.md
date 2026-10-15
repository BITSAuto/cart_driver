# Q-004 · A world position fix for route following

- **Opened:** 2026-10-08 · **Status:** Closed 2026-10-15 — not needed: recorded routes were dropped ([D-021](../03-decisions/D-021-no-recorded-routes.md))

`WorldAligner` needs world-frame positions (the sim's GPS). The cart has no
GPS publisher. BITSAuto has a `vector_nav` repository, which might be a GNSS
unit; check. Alternatives: LiDAR or visual localisation against a map.
Without it the cart can still drive with route-less heading keeping.
