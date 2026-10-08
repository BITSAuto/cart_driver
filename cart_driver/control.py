"""Speed holding and the safety decision (pure Python).

The cart never brakes in normal driving: its brake stops it instantly and can
damage it. So speed is held with throttle alone (a PI loop on measured speed,
output clamped at 0 -- never a negative/braking command), slowing means
coasting (throttle 0), and the emergency brake is a separate last resort that
only the safety logic can request.
"""

import enum


class SpeedController:
    """PI on measured speed -> commanded speed (the throttle setpoint).

    The real throttle is open-loop and uncalibrated (vehicle_bridge maps km/h
    to a throttle byte by an unverified scale), so the loop trims the command
    until the *measured* speed matches. Output is clamped to [0, max_cmd]:
    0 means coast, and there is no braking range.
    """

    def __init__(self, kp=0.5, ki=0.3, max_cmd=7.0 / 3.6):
        self.kp, self.ki, self.max_cmd = kp, ki, max_cmd
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, target, measured, dt):
        if target <= 0.0:
            self.reset()
            return 0.0                      # coast
        error = target - measured
        unclamped = target + self.kp * error + self.integral + self.ki * error * dt
        cmd = min(max(unclamped, 0.0), self.max_cmd)
        if cmd == unclamped:                # integrate only when not saturated (anti-windup)
            self.integral += self.ki * error * dt
        return cmd


class Action(enum.Enum):
    DRIVE = 'drive'
    COAST = 'coast'
    EMERGENCY_BRAKE = 'emergency_brake'


def coast_distance(speed, coast_decel):
    return speed * speed / (2.0 * coast_decel)


def safety_action(perception_age, speed, collision_dist, *, coast_decel, stale_after=0.5,
                  reaction_time=0.3, margin=0.5):
    """What the safety layer allows, given:

    perception_age  s since the last perception grid (inf if none yet)
    speed           measured speed, m/s
    collision_dist  m along the currently predicted path to the first
                    collision (inf if clear)

    EMERGENCY_BRAKE only when coasting can no longer stop the vehicle before
    the collision (coast distance + reaction distance + margin). Stale
    perception only coasts: by the time it matters the cart has rolled to a
    stop on its own, without the damaging brake.
    """
    stop_dist = coast_distance(speed, coast_decel) + speed * reaction_time + margin
    if speed > 0.05 and collision_dist <= stop_dist:
        return Action.EMERGENCY_BRAKE
    if perception_age > stale_after:
        return Action.COAST
    return Action.DRIVE
