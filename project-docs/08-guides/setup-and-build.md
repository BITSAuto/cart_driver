# Setup and build

Needs, in the same workspace: `tesla_sim`, `vehicle_bridge` (tesla_sim
imports it) and `traversability` (perception), plus the sim-trained RGB-D
student for the default sim launch. `launch/sim.launch.py` picks whichever
form of it the machine has (`default_model()`):

1. `trt:$BITSAUTO_HOME/models/student_b0_rgbd_sim.engine` (the Orin: TensorRT, no torch);
2. `student:$BITSAUTO_HOME/data/students/b0_rgbd_sim/best.pt`;
3. `student:~/traversability_data/students/b0_rgbd_sim/best.pt` (the laptop).

`BITSAUTO_HOME` defaults to `~/BITSAuto_All_repo`. Override with `model:=...`.

## Laptop (Humble, distrobox)
```bash
cd ~/ros2_ws/src && git clone https://github.com/BITSAuto/cart_driver.git
cd ~/ros2_ws && PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select cart_driver
cd ~/ros2_ws/src/cart_driver && python3 -m pytest test        # 18 on main, 24 with the road-centre cost
```

## Orin (`~/BITSAuto_All_repo`, Jazzy)
```bash
export ROS_DOMAIN_ID=23; source ~/BITSAuto_All_repo/env.sh
cd $BITSAUTO_HOME/ros2_ws/src/cart_driver && git status      # must be clean before switching branches
git fetch && git switch <branch> && git pull
cd $BITSAUTO_HOME/ros2_ws && PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select cart_driver
```
No local edits are needed for the Orin's folder layout any more (they were
needed before the model default learnt about it, 2026-10-15).

New files (e.g. under `config/`) need a rebuild to be installed (symlink install
covers edits to existing files).
