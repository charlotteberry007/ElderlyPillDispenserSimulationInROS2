#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # This project is intentionally fixed to TurtleBot3 Burger.
    os.environ['TURTLEBOT3_MODEL'] = 'burger'

    pkg_share = get_package_share_directory('medicine_delivery_sim')
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    tb3_nav_share = get_package_share_directory('turtlebot3_navigation2')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world = os.path.join(pkg_share, 'worlds', 'hospital.world')
    map_yaml = os.path.join(pkg_share, 'maps', 'hospital_map.yaml')
    nav_params = os.path.join(
    tb3_nav_share,
    'param',
    'humble',
    'burger.yaml'
)
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_share, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_share, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x_pose': '-7.0', 'y_pose': '0.0'}.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_nav_share, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_yaml,
            'params_file': nav_params,
        }.items(),
    )

    delivery = Node(
        package='medicine_delivery_sim',
        executable='medicine_delivery.py',
        name='medicine_delivery_robot',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        TimerAction(period=3.0, actions=[navigation]),
        TimerAction(period=8.0, actions=[delivery]),
    ])
