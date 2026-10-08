"""Vehicle geometry, steering conventions and kinematics (pure Python/numpy).

Reference point: the centre of the rear axle, on the ground. Poses are
(x, y, yaw) with yaw counter-clockwise from +x (REP 103).

Steering sign: in this stack (tesla_sim and vehicle_bridge alike) a POSITIVE
steering angle turns RIGHT -- /cmd_ackermann.steering_angle > 0 and
/steering/angle > 0 both mean right. That is the opposite of REP 103's
"positive = left", so this module converts at the boundary: everything inside
the planner uses curvature kappa > 0 = LEFT (counter-clockwise).
"""

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VehicleParams:
    wheelbase: float          # m
    width: float              # m, body
    front_overhang: float     # m, front axle -> front bumper
    rear_overhang: float      # m, rear axle -> rear bumper
    camera_x: float           # m, rear axle -> camera, along the vehicle axis
    max_steer_deg: float      # steering lock (bicycle-model angle)
    steer_rate_deg_s: float   # relay actuator rate
    cruise_kmh: float = 5.0

    @property
    def length(self):
        return self.wheelbase + self.front_overhang + self.rear_overhang

    @property
    def cruise_ms(self):
        return self.cruise_kmh / 3.6

    def footprint_circles(self, margin=0.0):
        """Circles covering the body: [(x along axis from rear axle, radius)]."""
        r = self.width / 2 + margin
        x0, x1 = -self.rear_overhang + r * 0.6, self.wheelbase + self.front_overhang - r * 0.6
        n = max(2, int(math.ceil((x1 - x0) / r)) + 1)
        return [(x, r) for x in np.linspace(x0, x1, n)]


PROFILES = {
    # Webots TeslaModel3 (tesla_sim): PROTO wheelbase/track, body ~4.7 x 1.85 m,
    # camera 1.67 m ahead of the rear axle (front sensor slot 3.79 m, camera -2.12).
    'tesla_sim': VehicleParams(wheelbase=2.94, width=1.85, front_overhang=0.85, rear_overhang=0.95,
                               camera_x=1.67, max_steer_deg=15.0, steer_rate_deg_s=12.86),
    # The real cart: steering lock and relay rate are known (cart_controller,
    # vehicle_bridge); dimensions are PLACEHOLDERS until measured.
    'cart': VehicleParams(wheelbase=1.7, width=1.2, front_overhang=0.4, rear_overhang=0.4,
                          camera_x=1.8, max_steer_deg=15.0, steer_rate_deg_s=12.86),
}


def steer_to_curvature(steer_right_deg, wheelbase):
    """Stack steering angle (deg, + = right) -> curvature (1/m, + = left)."""
    return -math.tan(math.radians(steer_right_deg)) / wheelbase


def curvature_to_steer(kappa, wheelbase):
    """Curvature (1/m, + = left) -> stack steering angle (deg, + = right)."""
    return -math.degrees(math.atan(kappa * wheelbase))


def step_pose(pose, v, kappa, dt):
    """Advance a rear-axle pose (x, y, yaw) along an arc. Exact for constant
    speed and curvature over dt."""
    x, y, yaw = pose
    ds = v * dt
    if abs(kappa) < 1e-9:
        return x + ds * math.cos(yaw), y + ds * math.sin(yaw), yaw
    dyaw = kappa * ds
    r = 1.0 / kappa
    return (x + r * (math.sin(yaw + dyaw) - math.sin(yaw)),
            y - r * (math.cos(yaw + dyaw) - math.cos(yaw)),
            yaw + dyaw)


def offset_point(pose, forward):
    """A point ``forward`` m ahead of a rear-axle pose along the vehicle axis."""
    x, y, yaw = pose
    return x + forward * math.cos(yaw), y + forward * math.sin(yaw)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))
