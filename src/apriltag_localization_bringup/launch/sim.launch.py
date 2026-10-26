import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('apriltag_localization_bringup')
    sim_share = get_package_share_directory('apriltag_localization_sim')

    rviz_config = os.path.join(bringup_share, 'rviz', 'localization.rviz')

    use_rviz = LaunchConfiguration('rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'localization.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'image_topic': '/camera/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'use_ekf': 'true',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        gazebo,
        localization,
        rviz,
    ])
