"""Safety monitor: the only node allowed to request the emergency brake.

Independent of the driver, so a planner bug can't disable it. Each tick it
predicts where the vehicle goes if it keeps its current (measured) steering,
checks that swept footprint against the latest perception grid, and decides:

  DRIVE            all clear
  COAST            perception is stale (> stale_after): throttle off, roll
                   to a stop on friction -- no brake
  EMERGENCY_BRAKE  an obstacle on the predicted path is closer than coasting
                   can stop for (coast distance + reaction + margin)

The emergency brake LATCHES: /vehicle/emergency_brake stays True until the
vehicle has stopped and someone calls ~/reset (std_srvs/Trigger). Every
engagement is logged. The decision is published on /cart/safety
(std_msgs/String: drive / coast / emergency_brake) for the driver to obey.
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

from cart_driver import control, vehicle
from cart_driver.control import Action
from cart_driver.gridview import GridView, clear_footprint, collision_distance


class SafetyNode(Node):
    def __init__(self):
        super().__init__('cart_safety')
        p = self.declare_parameter
        self.params = vehicle.PROFILES[p('vehicle', 'tesla_sim').value]
        self.coast_decel = p('coast_decel', 0.4).value
        self.stale_after = p('stale_after', 0.5).value
        self.reaction_time = p('reaction_time', 0.3).value
        self.margin = p('margin', 0.5).value
        self.footprint_margin = p('footprint_margin', 0.1).value
        self.horizon = p('horizon', 10.0).value
        # A collision must be seen on this many consecutive ticks (50 ms each)
        # before braking, so one bad perception frame can't fire the brake.
        self.confirm_ticks = p('brake_confirm_ticks', 2).value
        self.danger_ticks = 0
        speed_topic = p('speed_topic', '/vehicle/speed').value
        self.speed_scale = p('speed_scale', 1.0).value      # topic units -> m/s
        self.circles = self.params.footprint_circles(self.footprint_margin)

        self.speed = 0.0
        self.steer_deg = 0.0
        self.grid = None
        self.grid_time = None
        self.braking = False
        self.brake_count = 0
        self.last_action = None

        self.pub_brake = self.create_publisher(Bool, '/vehicle/emergency_brake', 10)
        self.pub_action = self.create_publisher(String, '/cart/safety', 10)
        self.pub_dist = self.create_publisher(Float32, '/cart/safety/collision_distance', 10)
        self.create_subscription(OccupancyGrid, '/traversability/grid', self.on_grid, 2)
        self.create_subscription(Float32, speed_topic, lambda m: setattr(self, 'speed', m.data * self.speed_scale), 10)
        self.create_subscription(Float32, '/steering/angle', lambda m: setattr(self, 'steer_deg', m.data), 10)
        self.create_service(Trigger, '~/reset', self.on_reset)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'Safety monitor: coast decel {self.coast_decel} m/s^2, brake latches until ~/reset')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_grid(self, msg):
        self.grid = msg
        self.grid_time = self.now()

    def predicted_path(self, start):
        """Rear-axle poses along the arc of the current steering, 0.1 m apart."""
        kappa = vehicle.steer_to_curvature(self.steer_deg, self.params.wheelbase)
        poses, pose = [start], start
        for _ in range(int(self.horizon / 0.1)):
            pose = vehicle.step_pose(pose, 1.0, kappa, 0.1)
            poses.append(pose)
        return np.array(poses)

    def collision_ahead(self):
        dist = math.inf
        if self.grid is not None:
            # (The grid is re-wrapped every tick; at 0.1 m cells this is cheap.)
            # Perception grid frame: on the ground under the camera, x forward.
            start = (-self.params.camera_x, 0.0, 0.0)
            view = clear_footprint(GridView.from_msg(self.grid), start, self.circles)
            dist = min(dist, collision_distance(view, self.predicted_path(start), self.circles, 0.1))
        return dist

    def tick(self):
        age = math.inf if self.grid_time is None else self.now() - self.grid_time
        dist = self.collision_ahead()
        action = control.safety_action(age, abs(self.speed), dist, coast_decel=self.coast_decel,
                                       stale_after=self.stale_after, reaction_time=self.reaction_time,
                                       margin=self.margin)
        self.danger_ticks = self.danger_ticks + 1 if action is Action.EMERGENCY_BRAKE else 0
        if action is Action.EMERGENCY_BRAKE and self.danger_ticks < self.confirm_ticks:
            action = Action.COAST            # unconfirmed: throttle off while we look again
        if action is Action.EMERGENCY_BRAKE and not self.braking:
            self.braking = True
            self.brake_count += 1
            self.get_logger().error(
                f'EMERGENCY BRAKE #{self.brake_count}: obstacle {dist:.2f} m ahead at {self.speed * 3.6:.1f} km/h '
                f'(coasting needs {control.coast_distance(abs(self.speed), self.coast_decel):.2f} m)')
        if self.braking:
            action = Action.EMERGENCY_BRAKE
        elif action is not self.last_action:
            self.get_logger().info(f'safety: {action.value}' + (f' (perception {age:.2f} s old)'
                                                                if action is Action.COAST else ''))
        self.last_action = action
        self.pub_brake.publish(Bool(data=self.braking))
        self.pub_action.publish(String(data=action.value))
        self.pub_dist.publish(Float32(data=float(min(dist, 1e3))))

    def on_reset(self, request, response):
        if abs(self.speed) > 0.05:
            response.success, response.message = False, 'vehicle still moving; not releasing the brake'
        else:
            self.braking = False
            response.success, response.message = True, 'emergency brake released'
            self.get_logger().warn('Emergency brake released by ~/reset')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            try:
                node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass                                     # context already shut down by the signal


if __name__ == '__main__':
    main()
