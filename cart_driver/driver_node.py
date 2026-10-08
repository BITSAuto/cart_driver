"""Driver: odometry, speed holding and steering -> /cmd_ackermann.

Speed: hold ``cruise_kmh`` (5 km/h) with a PI loop on measured speed. The
command never goes below 0 -- 0 means coast; the vehicle never brakes here.
The driver obeys /cart/safety: on ``coast`` or ``emergency_brake`` (or if the
safety monitor goes quiet for ``safety_timeout``) it commands throttle 0.

Steering: straight ahead (``steering: straight``) -- the local planner is
added in a later step.

Publishes /cart/odom (nav_msgs/Odometry, rear-axle pose in ``odom``) and the
TF odom -> base_link. Runs on sim time in simulation (use_sim_time).
"""

import math

import rclpy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry as OdometryMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String
from tf2_ros import TransformBroadcaster

from cart_driver import control, vehicle
from cart_driver.odometry import Odometry


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
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Float32, speed_topic, self.on_speed, 10)
        self.create_subscription(Float32, '/steering/angle', lambda m: setattr(self, 'steer_deg', m.data), 10)
        self.create_subscription(Imu, imu_topic, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(String, '/cart/safety', self.on_safety, 10)
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

    def steering(self):
        """Steering command, stack convention (deg, + = right)."""
        return 0.0

    def tick(self):
        t = self.now()
        if t == 0.0:
            return                                   # no /clock yet
        dt = 0.0 if self.last_tick is None else t - self.last_tick
        self.last_tick = t
        pose = self.odometry.update(t, self.speed, self.steer_deg, self.yaw_rate)

        safety_ok = (self.safety == control.Action.DRIVE.value and self.safety_time is not None
                     and t - self.safety_time < self.safety_timeout)
        target = self.cruise_ms if safety_ok else 0.0
        speed_cmd = self.speed_ctl.update(target, self.speed, dt) if dt > 0 else 0.0
        self.steer_cmd_deg = self.steering()

        cmd = AckermannDrive()
        cmd.speed = float(speed_cmd * 3.6)           # km/h, the stack's convention
        cmd.steering_angle = float(math.radians(self.steer_cmd_deg))
        self.pub_cmd.publish(cmd)
        self.pub_state.publish(String(data='drive' if safety_ok else f'coast ({self.safety or "no safety"})'))
        self.publish_odometry(pose)

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
