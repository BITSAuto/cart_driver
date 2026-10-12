# Testing on the Orin in the `road_loop` world, streamed to a laptop

The `road_loop` world (7 m road with curbs) is where the road-centre cost
does something. It lives in road_follower on the Orin.

1. Check nobody else is running Webots (`pgrep -a webots`), pick a domain.
2. Copy `road_follower/tools/stack.sh` into your own scratch directory and
   change two things: the line that starts `ros2 run road_follower ...` becomes
   ```bash
   setsid nohup ros2 launch cart_driver sim.launch.py perception:=false > "$LOGDIR/cart_driver.log" 2>&1 < /dev/null &
   ```
   and the stop patterns also match `ros2 launch cart_driver` and
   `install/cart_driver/lib/`. Set `LOGDIR` to your scratch directory.
   (Perception is started by the script itself, geometry only.)
3. `./stack_cart.sh start --stream --domain 23`
4. On the laptop:
   ```bash
   ssh -N -L 8000:localhost:8000 -L 1234:localhost:1234 bitsauto@bitsauto-orin
   ```
   then open `http://localhost:8000` and press Connect.
5. Watch `ros2 topic echo --field data /cart/state` (after
   `export ROS_DOMAIN_ID=23; source ~/BITSAuto_All_repo/env.sh`): it shows the
   distance from the fitted road centre.
6. Score: `python3 road_follower/tools/eval_run.py 330 out.csv` measures the
   rear axle against the world's true centre line.
7. `./stack_cart.sh stop`, and check port 8000 is free afterwards.
