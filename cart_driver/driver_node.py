"""Driver: odometry, speed holding and steering -> /cmd_ackermann.

Speed: hold ``cruise_kmh`` (5 km/h) with a PI loop on measured speed. The
command never goes below 0 -- 0 means coast; the vehicle never brakes here.
The driver obeys /cart/safety: on ``coast`` or ``emergency_brake`` (or if the
safety monitor goes quiet for ``safety_timeout``) it commands throttle 0.

Steering: cart_driver.planner.ArcPlanner on the memory map, re-planned at
``plan_rate`` Hz; the chosen path is published as /cart/plan. If no
candidate path is collision-free the driver coasts ("blocked") -- braking,
if it comes to that, is the safety monitor's call alone.

Memory: every /traversability/grid frame is placed in a rolling local map
(cart_driver.local_map) at the camera's pose when the frame was captured, so
obstacles that leave the camera's narrow view are remembered for
``memory_s`` s. Published as /cart/local_map (odom frame) for the safety
monitor and for viewing.

Publishes /cart/odom (nav_msgs/Odometry, rear-axle pose in ``odom``) and the
TF odom -> base_link. Runs on sim time in simulation (use_sim_time); sensor
stamps (wall clock in Webots) are mapped onto it by timesync.ClockMap.
"""

import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.msg import Odometry as OdometryMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String
from tf2_ros import TransformBroadcaster

from cart_driver import control, vehicle
from cart_driver.gridview import GridView
from cart_driver.local_map import LocalMap
from cart_driver.odometry import Odometry
from cart_driver.planner import ArcPlanner
from cart_driver.timesync import ClockMap


class DriverNode(Node):
    def __init__(self):
        super().__init__('cart_driver')
        p = self.declare_parameter
        self.params = vehicle.PROFILES[p('vehicle', 'tesla_sim').value]
        self.cruise_ms = p('cruise_kmh', self.params.cruise_kmh).value / 3.6
        self.rate = p('rate', 20.0).value
        self.safety_timeout = p('safety_timeout', 0.5).value
        speed_topic = p('speed_topic', '/vehicle/speed').value
        self.speed_scale = p('speed_scale', 1.0).value      # topic units -> m/s
        imu_topic = p('imu_topic', '/vehicle/imu').value
        self.use_imu = p('use_imu', True).value
        self.memory = LocalMap(size_m=p('memory_size_m', 40.0).value, resolution=p('memory_resolution', 0.1).value,
                               memory_s=p('memory_s', 15.0).value)
        self.clock_map = ClockMap(identity=not self.get_parameter('use_sim_time').value)
        self.map_period = 1.0 / p('map_publish_rate', 2.0).value
        self.last_map_pub = -math.inf
        self.frames = 0
        self.planner = ArcPlanner(self.params, horizon_m=p('horizon_m', 10.0).value)
        self.plan_period = 1.0 / p('plan_rate', 10.0).value
        self.last_plan_t = -math.inf
        self.plan = None
        self.route = None                 # set by the route follower (step 5)
        # Route-less course keeping: a slow average of the heading (time
        # constant heading_tau s) that the planner steers back towards.
        self.heading_tau = p('heading_tau', 8.0).value
        self.ref_heading = None

        self.speed = 0.0
        self.steer_deg = 0.0
        self.yaw_rate = None
        self.safety = None
        self.safety_time = None
        self.odometry = Odometry(self.params.wheelbase)
        self.speed_ctl = control.SpeedController()
        self.steer_cmd_deg = 0.0

        self.pub_cmd = self.create_publisher(AckermannDrive, '/cmd_ackermann', 1)
        self.pub_odom = self.create_publisher(OdometryMsg, '/cart/odom', 10)
        self.pub_state = self.create_publisher(String, '/cart/state', 10)
        self.pub_map = self.create_publisher(OccupancyGrid, '/cart/local_map', 2)
        self.pub_plan = self.create_publisher(Path, '/cart/plan', 2)
        self.pub_target = self.create_publisher(Float32, '/cart/steer_target', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Float32, speed_topic, self.on_speed, 10)
        self.create_subscription(Float32, '/steering/angle', lambda m: setattr(self, 'steer_deg', m.data), 10)
        self.create_subscription(Imu, imu_topic, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(String, '/cart/safety', self.on_safety, 10)
        self.create_subscription(OccupancyGrid, '/traversability/grid', self.on_grid, 2)
        self.last_tick = None
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(f'Driver: cruise {self.cruise_ms * 3.6:.1f} km/h, throttle-only (never brakes)')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_speed(self, msg):
        self.speed = msg.data * self.speed_scale

    def on_imu(self, msg):
        if self.use_imu:
            self.yaw_rate = msg.angular_velocity.z

    def on_safety(self, msg):
        self.safety = msg.data
        self.safety_time = self.now()

    def on_grid(self, msg):
        if self.odometry.time is None:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t = self.clock_map.to_sim(stamp)
        rear = self.odometry.pose_at(t)
        cx, cy = vehicle.offset_point(rear, self.params.camera_x)
        self.memory.integrate(GridView.from_msg(msg), (cx, cy, rear[2]), t)
        self.frames += 1

    def publish_map(self, t):
        if self.memory.origin is None or t - self.last_map_pub < self.map_period:
            return
        self.last_map_pub = t
        view = self.memory.view(t)
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = float(view.res)
        msg.info.height, msg.info.width = view.data.shape
        msg.info.origin.position.x, msg.info.origin.position.y = view.origin_x, view.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = view.data.ravel().tolist()
        self.pub_map.publish(msg)

    def steering(self, t, pose):
        """Steering command, stack convention (deg, + = right). Re-plans at
        plan_rate; between plans the last target is held."""
        if self.ref_heading is None:
            self.ref_heading = pose[2]
        elif self.speed > 0.3:
            k = min(1.0, (t - self.last_plan_t) / self.heading_tau) if math.isfinite(self.last_plan_t) else 0.0
            self.ref_heading = vehicle.wrap(self.ref_heading + k * vehicle.wrap(pose[2] - self.ref_heading))
        if self.memory.origin is not None and t - self.last_plan_t >= self.plan_period:
            self.last_plan_t = t
            self.plan = self.planner.plan(self.memory.view(t), pose, self.steer_deg, self.speed,
                                          prev_target_deg=self.steer_cmd_deg, route=self.route,
                                          ref_heading=self.ref_heading)
            self.publish_plan(self.plan)
        return self.plan.target_deg if self.plan is not None else 0.0

    def publish_plan(self, plan):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        for x, y, yaw in plan.poses[::4]:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            msg.poses.append(ps)
        self.pub_plan.publish(msg)

    def tick(self):
        t = self.now()
        if t == 0.0:
            return                                   # no /clock yet
        self.clock_map.observe(time.time(), t)
        dt = 0.0 if self.last_tick is None else t - self.last_tick
        self.last_tick = t
        pose = self.odometry.update(t, self.speed, self.steer_deg, self.yaw_rate)

        safety_ok = (self.safety == control.Action.DRIVE.value and self.safety_time is not None
                     and t - self.safety_time < self.safety_timeout)
        self.steer_cmd_deg = self.steering(t, pose)
        blocked = self.plan is not None and not self.plan.valid
        target = self.cruise_ms if safety_ok and not blocked else 0.0
        speed_cmd = self.speed_ctl.update(target, self.speed, dt) if dt > 0 else 0.0

        cmd = AckermannDrive()
        cmd.speed = float(speed_cmd * 3.6)           # km/h, the stack's convention
        cmd.steering_angle = float(math.radians(self.steer_cmd_deg))
        self.pub_cmd.publish(cmd)
        self.pub_target.publish(Float32(data=float(self.steer_cmd_deg)))
        state = ('drive' if safety_ok and not blocked else
                 'coast (blocked: no clear path)' if blocked else f'coast ({self.safety or "no safety"})')
        self.pub_state.publish(String(data=state))
        self.publish_odometry(pose)
        self.publish_map(t)

    def publish_odometry(self, pose):
        x, y, yaw = pose
        stamp = self.get_clock().now().to_msg()
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        msg = OdometryMsg()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = qz, qw
        msg.twist.twist.linear.x = self.speed
        self.pub_odom.publish(msg)
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.tf.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            try:
                node.pub_cmd.publish(AckermannDrive())   # leave the vehicle coasting, never braking
                node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass                                     # context already shut down by the signal


if __name__ == '__main__':
    main()
