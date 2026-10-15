#!/usr/bin/env python3
"""Environment health check: run it after `maze.launch.py` before handing over.

    ros2 run tb3_maze check_env [--ros-args -p duration:=5.0]

Checks /clock is advancing, /scan has 360 sane beams at ~5 Hz, /odom is
publishing, and the TF chain odom -> base_footprint -> base_scan resolves.
Exits non-zero if anything fails.
"""

import math
import sys
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class CheckEnv(Node):

    def __init__(self):
        super().__init__('check_env')
        self.declare_parameter('duration', 5.0)
        self.duration = float(self.get_parameter('duration').value)
        self.clock_first = self.clock_last = None
        self.scans, self.odoms = [], 0
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_clock(self, msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.clock_first = t if self.clock_first is None else self.clock_first
        self.clock_last = t

    def on_scan(self, msg):
        self.scans.append(msg)

    def on_odom(self, msg):
        self.odoms += 1

    def tf_ok(self, parent, child):
        try:
            self.tf_buffer.lookup_transform(parent, child, Time(), Duration(seconds=0.5))
            return True
        except Exception:
            return False


def main(args=None):
    rclpy.init(args=args)
    n = CheckEnv()
    end = time.time() + n.duration
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)

    checks = []
    sim_dt = (n.clock_last - n.clock_first) if n.clock_first is not None else 0.0
    checks.append(('/clock advancing', sim_dt > 0.5, f'{sim_dt:.2f} s sim in {n.duration:.0f} s wall'))

    rate = len(n.scans) / n.duration
    checks.append(('/scan publishing ~5 Hz', rate >= 2.0, f'{rate:.1f} Hz'))
    if n.scans:
        s = n.scans[-1]
        valid = [r for r in s.ranges if s.range_min <= r <= s.range_max and math.isfinite(r)]
        checks.append(('/scan beam count', len(s.ranges) in (360, 48),
                       f'{len(s.ranges)} beams, frame "{s.header.frame_id}"'))
        checks.append(('/scan sees walls', len(valid) > 0.5 * len(s.ranges),
                       f'{len(valid)}/{len(s.ranges)} valid, min '
                       f'{min(valid) if valid else float("nan"):.2f} m'))
    else:
        checks.append(('/scan beam count', False, 'no scans (Sensors plugin? use_sim_time?)'))

    orate = n.odoms / n.duration
    checks.append(('/odom publishing', orate >= 5.0, f'{orate:.1f} Hz'))
    checks.append(('TF odom -> base_footprint', n.tf_ok('odom', 'base_footprint'), 'DiffDrive'))
    checks.append(('TF base_footprint -> base_scan', n.tf_ok('base_footprint', 'base_scan'),
                   'robot_state_publisher'))

    width = max(len(c[0]) for c in checks)
    failed = 0
    print()
    for name, ok, detail in checks:
        failed += not ok
        print(f'  [{"PASS" if ok else "FAIL"}] {name.ljust(width)}  {detail}')
    print(f'\n  {"ALL CHECKS PASSED" if not failed else f"{failed} CHECK(S) FAILED"}\n')

    n.destroy_node()
    rclpy.try_shutdown()
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
