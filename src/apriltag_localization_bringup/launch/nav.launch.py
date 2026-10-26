import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('apriltag_localization_bringup')
    sim = get_package_share_directory('apriltag_localization_sim')

    nav2_params = os.path.join(bringup, 'config', 'nav2_params.yaml')
    map_yaml = os.path.join(sim, 'maps', 'warehouse.yaml')
    rviz_cfg = os.path.join(bringup, 'rviz', 'localization.rviz')

    use_rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')

    # Spawn facing the -x wall tag so the estimator localizes immediately.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'use_sim_time': 'true', 'gui': gui,
            'x': '-8.5', 'y': '0.0', 'yaw': '3.141593',
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'localization.launch.py')),
        launch_arguments={'use_sim_time': 'true', 'use_ekf': 'true'}.items(),
    )

    lifecycle_nodes = [
        'map_server', 'controller_server', 'planner_server',
        'behavior_server', 'bt_navigator', 'waypoint_follower',
    ]

    nav = [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[nav2_params, {'yaml_filename': map_yaml, 'use_sim_time': True}]),
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=[nav2_params]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[nav2_params]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=[nav2_params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=[nav2_params]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=[nav2_params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_nav', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': lifecycle_nodes}]),
    ]

    route = TimerAction(period=25.0, actions=[
        Node(package='apriltag_localization_tools', executable='follow_route',
             name='follow_route', output='screen',
             parameters=[{'use_sim_time': True}]),
    ])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_cfg], parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        gazebo,
        localization,
        *nav,
        route,
        rviz,
    ])
