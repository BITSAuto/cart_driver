"""Sim-only helpers for cart_driver scenarios (tesla_sim / Webots).

  python3 simtools.py spawn NAME AHEAD LEFT SX SY SZ [--yaw 3.1415] [--flat]
      box obstacle AHEAD m in front of the GPS (front bumper) and LEFT m to
      the left of the car's axis, at the car's start heading; --flat makes
      drivable clutter (paper, paint) lying on the road
  python3 simtools.py remove NAME...
  python3 simtools.py monitor SECONDS OUT.csv [--until-stopped]
      record speed, safety action, e-brake, steering, GPS, route cross-track
      (sim time) and print a summary

Positions: the GPS sits in the Tesla's front sensor slot, which is the front
bumper (3.79 m ahead of the rear axle).
"""

import argparse
import csv
import math
import sys

import rclpy
import rclpy.parameter
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


def sim_node(name):
    return Node(name, parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])


def wait_for(node, cond, timeout=30.0):
    import time
    t0 = time.time()
    while not cond() and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    return cond()


def spawn(args):
    from webots_ros2_msgs.srv import SpawnNodeFromString
    n = Node('simtools_spawn')
    gps = {}
    n.create_subscription(PointStamped, '/vehicle/gps',
                          lambda m: gps.setdefault('p', (m.point.x, m.point.y, m.point.z)), 1)
    cli = n.create_client(SpawnNodeFromString, '/Ros2Supervisor/spawn_node_from_string')
    if not wait_for(n, lambda: 'p' in gps) or not cli.wait_for_service(timeout_sec=30.0):
        sys.exit('no /vehicle/gps or spawn service')
    c, s = math.cos(args.yaw), math.sin(args.yaw)
    x = gps['p'][0] + c * args.ahead - s * args.left
    y = gps['p'][1] + s * args.ahead + c * args.left
    box = f'Box {{ size {args.sx} {args.sy} {args.sz} }}'
    if args.flat:
        # Flat clutter (paper, paint): no physics, lying on the road surface (z 0.02), drivable.
        z, color, solid = 0.02 + args.sz / 2, args.color or '0.95 0.95 0.9', ''
    else:
        z, color = 0.3 + args.sz / 2, args.color or '0.8 0.45 0.1'
        solid = f'boundingObject {box} physics Physics {{ density -1 mass 20 }} '
    vrml = (f'Solid {{ name "{args.name}" translation {x:.3f} {y:.3f} {z:.3f} rotation 0 0 1 {args.yaw:.4f} '
            f'children [ Shape {{ appearance PBRAppearance {{ baseColor {color} roughness 1 metalness 0 }} '
            f'geometry {box} }} ] {solid}}}')
    fut = cli.call_async(SpawnNodeFromString.Request(data=vrml))
    rclpy.spin_until_future_complete(n, fut, timeout_sec=10.0)
    ok = fut.result() is not None and fut.result().success
    print(f'{args.name}: {"spawned" if ok else "FAILED"} at world ({x:.2f}, {y:.2f})')


def remove(args):
    n = Node('simtools_remove')
    pub = n.create_publisher(String, '/Ros2Supervisor/remove_node', 10)
    wait_for(n, lambda: pub.get_subscription_count() > 0)
    for name in args.names:
        pub.publish(String(data=name))
        rclpy.spin_once(n, timeout_sec=0.1)


def monitor(args):
    n = sim_node('simtools_monitor')
    st = {'speed': 0.0, 'safety': '', 'brake': False, 'steer': 0.0, 'gps': (math.nan, math.nan), 'dist': math.nan,
          'cross': math.nan}
    n.create_subscription(Float32, '/vehicle/speed', lambda m: st.__setitem__('speed', m.data), 10)
    n.create_subscription(String, '/cart/safety', lambda m: st.__setitem__('safety', m.data), 10)
    n.create_subscription(Bool, '/vehicle/emergency_brake', lambda m: st.__setitem__('brake', m.data), 10)
    n.create_subscription(Float32, '/steering/angle', lambda m: st.__setitem__('steer', m.data), 10)
    n.create_subscription(Float32, '/cart/safety/collision_distance', lambda m: st.__setitem__('dist', m.data), 10)
    n.create_subscription(Float32, '/cart/route/cross_track', lambda m: st.__setitem__('cross', m.data), 10)
    n.create_subscription(PointStamped, '/vehicle/gps', lambda m: st.__setitem__('gps', (m.point.x, m.point.y)), 1)

    def now():
        return n.get_clock().now().nanoseconds * 1e-9
    wait_for(n, lambda: now() > 0)
    t0, rows, last = now(), [], -1.0
    moved = False
    while now() - t0 < args.seconds:
        rclpy.spin_once(n, timeout_sec=0.02)
        t = now() - t0
        if t - last >= 0.1:
            last = t
            rows.append((round(t, 2), st['speed'], st['safety'], int(st['brake']), st['steer'], *st['gps'], st['dist'], st['cross']))
            moved |= st['speed'] > 1.0
            if args.until_stopped and moved and st['speed'] < 0.02 and t > 5:
                break
    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'speed', 'safety', 'brake', 'steer_deg', 'gps_x', 'gps_y', 'collision_dist', 'cross_track'])
        w.writerows(rows)
    summarize(rows)


def summarize(rows):
    import numpy as np
    t = np.array([r[0] for r in rows])
    v = np.array([r[1] for r in rows])
    brake = np.array([r[3] for r in rows])
    safety = [r[2] for r in rows]
    gx = np.array([r[5] for r in rows])
    gy = np.array([r[6] for r in rows])
    cruising = (np.array([s == 'drive' for s in safety])) & (t > 5)
    dist = float(np.nansum(np.hypot(np.diff(gx), np.diff(gy))))
    print(f'duration {t[-1]:.1f} s sim, distance {dist:.1f} m')
    if cruising.any():
        print(f'speed while driving (after 5 s): mean {v[cruising].mean() * 3.6:.2f} km/h, '
              f'min {v[cruising].min() * 3.6:.2f}, max {v[cruising].max() * 3.6:.2f}')
    changes = [(t[i], safety[i]) for i in range(len(safety)) if i == 0 or safety[i] != safety[i - 1]]
    print('safety states:', ', '.join(f'{s}@{tt:.1f}s' for tt, s in changes))
    engaged = np.flatnonzero(np.diff(brake) > 0)
    print(f'emergency brake engagements: {len(engaged) + int(brake[0] == 1)}')
    cross = np.array([r[8] for r in rows])
    if np.isfinite(cross).any():
        settled = np.isfinite(cross) & (np.abs(cross) < 1.0)
        after = np.arange(len(cross)) >= (np.argmax(settled) if settled.any() else len(cross))
        c = np.abs(cross[after & np.isfinite(cross)])
        if c.size:
            print(f'route cross-track once within 1 m ({t[after][0]:.1f} s on): mean {c.mean():.2f} m, '
                  f'p95 {np.percentile(c, 95):.2f}, max {c.max():.2f}')
    for i in engaged:
        print(f'  at {t[i + 1]:.1f} s, speed {v[i] * 3.6:.2f} km/h, '
              f'obstacle {rows[i][7]:.2f} m ahead on the predicted path')


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('spawn')
    for arg, kind in [('name', str), ('ahead', float), ('left', float), ('sx', float), ('sy', float), ('sz', float)]:
        s.add_argument(arg, type=kind)
    s.add_argument('--yaw', type=float, default=3.1415)
    s.add_argument('--flat', action='store_true', help='drivable flat clutter: no physics, on the road surface')
    s.add_argument('--color', help='r g b, e.g. "0.9 0.8 0.1"')
    r = sub.add_parser('remove')
    r.add_argument('names', nargs='+')
    m = sub.add_parser('monitor')
    m.add_argument('seconds', type=float)
    m.add_argument('out')
    m.add_argument('--until-stopped', action='store_true')
    args = ap.parse_args()
    rclpy.init()
    {'spawn': spawn, 'remove': remove, 'monitor': monitor}[args.cmd](args)


if __name__ == '__main__':
    main()
