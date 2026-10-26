"""Integration test for the pose estimator node.

Feeds a synthetic perspective tag view together with camera intrinsics and a
static camera->base transform, then checks the published map-frame pose against
the known camera pose.
"""

import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import numpy as np
import pytest

import rclpy
from rclpy.qos import qos_profile_sensor_data

from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo

FIXTURE = os.path.join(os.path.dirname(__file__), 'fixtures', 'single_tag.yaml')

FX = FY = 554.254691
CX, CY = 320.0, 240.0
TAG_SIZE = 0.4

# Tag pose in the map frame (matches the fixture): identity rotation at (0, 0, 2).
T_MAP_TAG = np.eye(4)
T_MAP_TAG[:3, 3] = [0.0, 0.0, 2.0]


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def tag_cam_transform():
    """A clearly non-fronto-parallel tag pose in the camera optical frame."""
    T = np.eye(4)
    T[:3, :3] = rot_y(0.25) @ rot_x(0.15)
    T[:3, 3] = [0.1, 0.05, 1.5]
    return T


def quat_from_matrix(r):
    t = np.trace(r)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    else:
        i = int(np.argmax([r[0, 0], r[1, 1], r[2, 2]]))
        if i == 0:
            s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
            w = (r[2, 1] - r[1, 2]) / s
            x = 0.25 * s
            y = (r[0, 1] + r[1, 0]) / s
            z = (r[0, 2] + r[2, 0]) / s
        elif i == 1:
            s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
            w = (r[0, 2] - r[2, 0]) / s
            x = (r[0, 1] + r[1, 0]) / s
            y = 0.25 * s
            z = (r[1, 2] + r[2, 1]) / s
        else:
            s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
            w = (r[1, 0] - r[0, 1]) / s
            x = (r[0, 2] + r[2, 0]) / s
            y = (r[1, 2] + r[2, 1]) / s
            z = 0.25 * s
    return np.array([x, y, z, w])


def tag_object_points():
    h = TAG_SIZE / 2.0
    # AprilTag order: BL, BR, TR, TL in the tag frame.
    return np.array([[-h, -h, 0], [h, -h, 0], [h, h, 0], [-h, h, 0]], dtype=float)


def projected_corners(t_cam_tag):
    pts = tag_object_points()
    cam = (t_cam_tag[:3, :3] @ pts.T).T + t_cam_tag[:3, 3]
    corners = []
    for x, y, z in cam:
        corners.append((FX * x / z + CX, FY * y / z + CY))
    return corners


def expected_camera_pose(t_cam_tag):
    return T_MAP_TAG @ np.linalg.inv(t_cam_tag)


@pytest.mark.launch_test
def generate_test_description():
    pose_node = launch_ros.actions.Node(
        package='apriltag_pose_estimator',
        executable='pose_estimator_node',
        name='apriltag_pose_estimator',
        parameters=[{
            'tag_map': FIXTURE,
            'base_frame': 'base_link',
            'publish_tf': False,
            'publish_markers': False,
        }],
    )
    static_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--frame-id', 'camera_optical_frame',
            '--child-frame-id', 'base_link',
        ],
    )
    return (
        launch.LaunchDescription([
            pose_node,
            static_tf,
            launch_testing.actions.ReadyToTest(),
        ]),
        {},
    )


class TestPipeline(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('pipeline_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_pose_matches_known_camera_pose(self):
        received = []
        self.node.create_subscription(
            PoseWithCovarianceStamped, '/localization/pose',
            lambda m: received.append(m), 10)

        info_pub = self.node.create_publisher(
            CameraInfo, '/camera_info', qos_profile_sensor_data)
        det_pub = self.node.create_publisher(
            AprilTagDetectionArray, '/detections', qos_profile_sensor_data)

        t_cam_tag = tag_cam_transform()
        expected = expected_camera_pose(t_cam_tag)
        exp_t = expected[:3, 3]
        exp_q = quat_from_matrix(expected[:3, :3])

        info = CameraInfo()
        info.header.frame_id = 'camera_optical_frame'
        info.width = 640
        info.height = 480
        info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.distortion_model = 'plumb_bob'

        det_array = AprilTagDetectionArray()
        det_array.header.frame_id = 'camera_optical_frame'
        detection = AprilTagDetection()
        detection.id = 0
        detection.family = 'tag36h11'
        for i, (u, v) in enumerate(projected_corners(t_cam_tag)):
            detection.corners[i] = _point(u, v)
        detection.centre = _point(CX, CY)
        det_array.detections.append(detection)

        end = time.time() + 15.0
        while time.time() < end and not received:
            now = self.node.get_clock().now().to_msg()
            info.header.stamp = now
            det_array.header.stamp = now
            info_pub.publish(info)
            det_pub.publish(det_array)
            rclpy.spin_once(self.node, timeout_sec=0.2)

        self.assertTrue(received, 'no pose was published')
        pose = received[-1].pose.pose
        got_t = np.array([pose.position.x, pose.position.y, pose.position.z])
        got_q = np.array([pose.orientation.x, pose.orientation.y,
                          pose.orientation.z, pose.orientation.w])

        self.assertLess(np.linalg.norm(got_t - exp_t), 0.02)
        self.assertGreater(abs(float(np.dot(got_q, exp_q))), 0.999)


def _point(x, y):
    from apriltag_msgs.msg import Point
    p = Point()
    p.x = float(x)
    p.y = float(y)
    return p
