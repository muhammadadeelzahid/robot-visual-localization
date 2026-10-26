#!/usr/bin/env python3
"""Compare localization estimates against Gazebo ground truth.

Records the EKF output and the raw AprilTag pose alongside the simulator's true
robot pose, then on shutdown reports absolute trajectory error and writes plots.
"""

import argparse
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

import numpy as np  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402

from gazebo_msgs.msg import ModelStates  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402

from apriltag_localization_tools.geometry import (  # noqa: E402
    wrap_angle as wrap,
    yaw_from_quaternion,
)


def yaw_from_quat(q):
    return yaw_from_quaternion(q.x, q.y, q.z, q.w)


class ErrorAnalysis(Node):

    def __init__(self, model_name, output_dir):
        super().__init__('localization_error_analysis')
        self.model_name = model_name
        self.output_dir = output_dir
        self.truth = []   # (t, x, y, yaw)
        self.ekf = []
        self.tag = []

        self.create_subscription(ModelStates, '/gazebo/model_states',
                                 self.on_truth, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_ekf, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/apriltag/pose',
                                 self.on_tag, 10)
        self.get_logger().info('recording; press Ctrl-C to analyse')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_truth(self, msg):
        if self.model_name not in msg.name:
            return
        pose = msg.pose[msg.name.index(self.model_name)]
        self.truth.append((self.now(), pose.position.x, pose.position.y,
                           yaw_from_quat(pose.orientation)))

    def on_ekf(self, msg):
        p = msg.pose.pose
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.ekf.append((t, p.position.x, p.position.y, yaw_from_quat(p.orientation)))

    def on_tag(self, msg):
        p = msg.pose.pose
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.tag.append((t, p.position.x, p.position.y, yaw_from_quat(p.orientation)))

    def align(self, estimate):
        truth = np.array(self.truth)
        est = np.array(estimate)
        if len(truth) == 0 or len(est) == 0:
            return None
        pairs = []
        for row in est:
            idx = int(np.argmin(np.abs(truth[:, 0] - row[0])))
            if abs(truth[idx, 0] - row[0]) > 0.5:
                continue
            pairs.append((truth[idx], row))
        return pairs

    def stats(self, estimate, label):
        pairs = self.align(estimate)
        if not pairs:
            self.get_logger().info(f'{label}: no overlapping samples')
            return None
        trans = np.array([math.hypot(t[1] - e[1], t[2] - e[2]) for t, e in pairs])
        rot = np.array([abs(wrap(t[3] - e[3])) for t, e in pairs])
        ate = math.sqrt(float(np.mean(trans ** 2)))
        self.get_logger().info(
            f'{label}: ATE {ate:.3f} m, mean {trans.mean():.3f} m, '
            f'max {trans.max():.3f} m, yaw mean {math.degrees(rot.mean()):.2f} deg '
            f'({len(pairs)} samples)')
        return trans, rot

    def report(self):
        os.makedirs(self.output_dir, exist_ok=True)
        truth = np.array(self.truth) if self.truth else np.empty((0, 4))

        plt.figure(figsize=(7, 7))
        if len(truth):
            plt.plot(truth[:, 1], truth[:, 2], 'k-', label='ground truth')
        if self.ekf:
            e = np.array(self.ekf)
            plt.plot(e[:, 1], e[:, 2], 'b.', ms=2, label='EKF')
        if self.tag:
            g = np.array(self.tag)
            plt.plot(g[:, 1], g[:, 2], 'r.', ms=2, label='AprilTag')
        plt.axis('equal')
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.legend()
        plt.title('Trajectory')
        path = os.path.join(self.output_dir, 'trajectory.png')
        plt.savefig(path, dpi=120)
        self.get_logger().info(f'wrote {path}')

        self.stats(self.ekf, 'EKF')
        self.stats(self.tag, 'AprilTag')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='localization_bot')
    parser.add_argument('--output-dir', default='analysis_output')
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = ErrorAnalysis(args.model, args.output_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
