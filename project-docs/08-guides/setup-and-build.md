# Setup and build

Needs, in the same workspace: `tesla_sim`, `vehicle_bridge` (tesla_sim
imports it) and `traversability` (perception), plus the sim-trained student
at `~/traversability_data/students/b0_rgbd_sim/best.pt` for the default sim
launch (see traversability's book for where models live).

```bash
cd ~/ros2_ws/src && git clone https://github.com/BITSAuto/cart_driver.git
cd ~/ros2_ws && PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select cart_driver
cd ~/ros2_ws/src/cart_driver && python3 -m pytest test        # 18 on main, 26 with PR #4
```
New files under `routes/` need a rebuild to be installed (symlink install
covers edits to existing files).
