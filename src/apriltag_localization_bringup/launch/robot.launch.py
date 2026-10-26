import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    bringup_share = get_package_share_directory('apriltag_localization_bringup')
    sim_share = get_package_share_directory('apriltag_localization_sim')

    camera = LaunchConfiguration('camera').perform(context)
    video_device = LaunchConfiguration('video_device').perform(context)
    camera_info_url = LaunchConfiguration('camera_info_url').perform(context)
    use_ekf = LaunchConfiguration('use_ekf')

    xacro_file = os.path.join(sim_share, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    nodes = [robot_state_publisher]

    if camera == 'realsense':
        image_topic = '/camera/color/image_raw'
        camera_info_topic = '/camera/color/camera_info'
        nodes.append(Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[{'enable_depth': False, 'rgb_camera.profile': '640x480x30'}],
        ))
    else:
        image_topic = '/camera/image_raw'
        camera_info_topic = '/camera/camera_info'
        nodes.append(Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': video_device,
                'image_width': 640,
                'image_height': 480,
                'frame_id': 'camera_optical_frame',
                'camera_name': 'camera',
                'camera_info_url': camera_info_url,
            }],
            remappings=[
                ('image_raw', image_topic),
                ('camera_info', camera_info_topic),
            ],
        ))

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'localization.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'image_topic': image_topic,
            'camera_info_topic': camera_info_topic,
            'use_ekf': use_ekf,
        }.items(),
    )
    nodes.append(localization)

    rviz_config = os.path.join(bringup_share, 'rviz', 'localization.rviz')
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', rviz_config],
    ))

    return nodes


def generate_launch_description():
    bringup_share = get_package_share_directory('apriltag_localization_bringup')
    default_info = os.path.join(bringup_share, 'config', 'camera_intrinsics.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='usb',
                              description='usb or realsense'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('camera_info_url',
                              default_value='file://' + default_info),
        DeclareLaunchArgument('use_ekf', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
