"""Record a loop route in tesla_sim by following the yellow centre line.

  python3 record_route.py OUT.csv [--speed 10]

Drives the car centred on the double yellow centre line (camera colour
segmentation, proportional steering in the stack's + = right convention),
logs the GPS (front bumper) every 0.5 m, and stops when it returns within
6 m of the start after at least 150 m. make_route.py then turns this centre
line into a keep-left lane route. Sim-only; nothing else should be driving.
"""

import argparse
import csv
import math

import cv2
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDrive
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# With the camera 1.52 m up and pitched 9 deg down (931 px focal length),
# rows 400-520 see the road ~5-8 m ahead; a line straight ahead sits at the
# principal point's column (640).
ROWS, CENTRE_COL, GAIN = (400, 520), 640, 0.001        # rad of steering per pixel


class Recorder(Node):
    def __init__(self, out, speed_kmh):
        super().__init__('record_route')
        self.bridge = CvBridge()
        self.speed = speed_kmh
        self.out = open(out, 'w', newline='')
        self.writer = csv.writer(self.out)
        self.writer.writerow(['x', 'y'])
        self.pub = self.create_publisher(AckermannDrive, '/cmd_ackermann', 1)
        self.create_subscription(Image, '/vehicle/camera/image_color', self.on_image, qos_profile_sensor_data)
        self.create_subscription(PointStamped, '/vehicle/gps', self.on_gps, 10)
        self.start = self.last = None
        self.travelled = 0.0
        self.lost = 0
        self.done = False

    def on_image(self, msg):
        if self.done:
            return
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')[ROWS[0]:ROWS[1]]
        mask = cv2.inRange(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), (15, 90, 110), (40, 255, 255))
        xs = np.nonzero(mask)[1]
        cmd = AckermannDrive(speed=float(self.speed))
        if len(xs) > 80:
            self.lost = 0
            cmd.steering_angle = float(np.clip(GAIN * (np.median(xs) - CENTRE_COL), -0.26, 0.26))
        else:
            self.lost += 1           # junction without a centre line: straight on
        self.pub.publish(cmd)

    def on_gps(self, msg):
        p = (msg.point.x, msg.point.y)
        if self.start is None:
            self.start = self.last = p
            self.writer.writerow(p)
            return
        step = math.dist(p, self.last)
        if step >= 0.5:
            self.travelled += step
            self.last = p
            self.writer.writerow(p)
            if self.travelled > 150 and math.dist(p, self.start) < 6.0:
                self.done = True
        if int(self.travelled) % 50 == 0 and step >= 0.5:
            self.get_logger().info(f'{self.travelled:.0f} m recorded')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--speed', type=float, default=10.0)
    args = ap.parse_args()
    rclpy.init()
    node = Recorder(args.out, args.speed)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    for _ in range(20):
        node.pub.publish(AckermannDrive())        # coast
        rclpy.spin_once(node, timeout_sec=0.05)
    node.out.close()
    print(f'loop closed after {node.travelled:.0f} m -> {args.out}')


if __name__ == '__main__':
    main()
