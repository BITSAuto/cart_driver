# Sim facts that matter here

From tesla_sim's book, the ones this stack depends on:
- Sim time runs at ~0.55× real time; sensor stamps are wall-clock. All nodes
  use `use_sim_time`; stamps go through `ClockMap`.
- Start pose: rear axle (31.44, 47.01), yaw π (−x); GPS (front bumper) at
  ≈ (27.66, 47.01), 2 m right of the centre line, i.e. ~10 m from the
  kerb-side left lane.
- Loop A: centre line x ∈ [−45, 105], y ∈ [−105, 45], corner radius 40.5 m;
  lanes 5.375 m; kerb-side left lane 8.06 m left of the centre line → 480 m
  loop with 32.4 m corners.
- Objects spawned in the start intersection sink; spawn on `road(5)`.
- Speed 0 coasts at 0.4 m/s²; `/vehicle/emergency_brake` stops instantly.
- There's no reset: restart the sim to return to the start pose.
