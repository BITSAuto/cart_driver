# Q-004 · A world position fix for route following

- **Opened:** 2026-10-08 · **Status:** Open · **Priority:** high for step 5 on the cart

`WorldAligner` needs world-frame positions (the sim's GPS). The cart has no
GPS publisher. BITSAuto has a `vector_nav` repository, which might be a GNSS
unit; check. Alternatives: LiDAR or visual localisation against a map.
Without it the cart can still drive with route-less heading keeping.
