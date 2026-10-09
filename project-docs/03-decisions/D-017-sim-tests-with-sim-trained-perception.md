# D-017 · Sim tests use the sim-trained perception

- **Date:** 2026-10-08
- **Status:** Accepted (owner's direction)

`sim.launch.py` includes traversability with semantics on and the
sim-trained SegFormer-B0 RGB-D student plus depth
(`student:~/traversability_data/students/b0_rgbd_sim/best.pt`). The real
cart will use the Orin's TensorRT engine instead.
