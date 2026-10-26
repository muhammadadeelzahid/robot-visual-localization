"""Headless integration test for the pose estimator node.

Feeds a synthetic fronto-parallel tag detection together with camera intrinsics
and a static camera->base transform, then checks the published map-frame pose.
No simulator is required.
"""

import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
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
TAG_Z = 2.0


def projected_corners():
    """AprilTag-order corners of the fixture tag, fronto-parallel at z = TAG_Z."""
    h = TAG_SIZE / 2.0
    object_points = [(-h, -h), (h, -h), (h, h), (-h, h)]  # BL, BR, TR, TL
    corners = []
    for x, y in object_points:
        u = FX * x / TAG_Z + CX
        v = FY * y / TAG_Z + CY
        corners.append((u, v))
    return corners


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

    def test_pose_published(self):
        received = []
        self.node.create_subscription(
            PoseWithCovarianceStamped, '/localization/pose',
            lambda m: received.append(m), 10)

        info_pub = self.node.create_publisher(
            CameraInfo, '/camera_info', qos_profile_sensor_data)
        det_pub = self.node.create_publisher(
            AprilTagDetectionArray, '/detections', qos_profile_sensor_data)

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
        for u, v in projected_corners():
            detection.corners.append(_point(u, v))
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
        self.assertAlmostEqual(pose.position.x, 0.0, delta=0.05)
        self.assertAlmostEqual(pose.position.y, 0.0, delta=0.05)
        self.assertAlmostEqual(pose.position.z, 0.0, delta=0.05)
        self.assertAlmostEqual(abs(pose.orientation.w), 1.0, delta=0.02)


def _point(x, y):
    from apriltag_msgs.msg import Point
    p = Point()
    p.x = float(x)
    p.y = float(y)
    return p
