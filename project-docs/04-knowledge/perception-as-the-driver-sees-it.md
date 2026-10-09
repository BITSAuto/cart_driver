# Perception as the driver sees it

- **Coverage:** ~2.7–10 m ahead (bonnet hides the near field), colour FOV 69°
  (ground outside it is unknown when fusing). Grid: `camera_ground`, 0.1 m in
  the sim.
- **Latency:** semantics ~0.21 s behind the camera in the sim with the full
  stack (after traversability PR #2; 0.67 s before, which made the fused grid
  mostly unknown).
- **Start-up:** models take a minute or two to load; `/traversability/grid`
  and therefore `/cart/local_map` don't exist until then.
- **Stale semantics:** traversability publishes obstacles with ground
  *unknown*, never geometry-only free.
- **Known false positives in the sim:** the raised pedestrian crossings
  (fixed in tesla_sim) and the bonnet when traversability's `params` were
  hijacked (fixed by `cart_params`).
- **Flat clutter** (paper, paint, stains) is free: verified by scenario E
  (0.0° of steering over all three).
