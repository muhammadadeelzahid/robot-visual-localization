import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('apriltag_localization_bringup')
    default_tag_map = os.path.join(bringup_share, 'config', 'tags.yaml')
    default_ekf = os.path.join(bringup_share, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    tag_map = LaunchConfiguration('tag_map')
    base_frame = LaunchConfiguration('base_frame')
    use_ekf = LaunchConfiguration('use_ekf')
    publish_pose_tf = LaunchConfiguration('publish_pose_tf')

    detector = Node(
        package='apriltag_detector',
        executable='detector_node',
        name='apriltag_detector',
        namespace='apriltag',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'family': 'tag36h11',
            'quad_decimate': 1.0,
            'nthreads': 2,
            'publish_overlay': True,
        }],
        remappings=[('image', image_topic)],
    )

    pose_estimator = Node(
        package='apriltag_pose_estimator',
        executable='pose_estimator_node',
        name='apriltag_pose_estimator',
        namespace='apriltag',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'tag_map': tag_map,
            'base_frame': base_frame,
            'publish_tf': publish_pose_tf,
            'publish_markers': True,
        }],
        remappings=[
            ('camera_info', camera_info_topic),
            ('localization/pose', '/apriltag/pose'),
        ],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_map',
        output='screen',
        condition=IfCondition(use_ekf),
        parameters=[default_ekf, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('tag_map', default_value=default_tag_map),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('use_ekf', default_value='true'),
        DeclareLaunchArgument('publish_pose_tf', default_value='false'),
        detector,
        pose_estimator,
        ekf,
    ])
