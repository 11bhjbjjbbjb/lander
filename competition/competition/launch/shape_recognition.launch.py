#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    debug_display_arg = DeclareLaunchArgument(
        'debug_display',
        default_value='false',
        description='是否启用调试显示窗口',
    )
    
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='是否启动摄像头节点',
    )

    enable_pick_arg = DeclareLaunchArgument(
        'enable_pick',
        default_value='true',
        description='是否启动位置校正节点（底盘视觉对齐）',
    )


    camera_driver_pkg = get_package_share_directory('peripherals')
    controller_pkg = get_package_share_directory('controller')

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_driver_pkg, 'launch', 'depth_camera.launch.py'),
        ),
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_pkg, 'launch', 'controller.launch.py'),
        ),
    )

    shape_recognition_node = Node(
        package='competition',
        executable='shape_recognition',
        name='shape_recognition',
        output='screen',
        respawn=True,
        parameters=[
            {'debug_display': LaunchConfiguration('debug_display')},
        ],
    )

    automatic_pick_node = Node(
        package='competition',
        executable='automatic_pick',
        name='automatic_pick',
        output='screen',
        parameters=[
            {'debug': False},
            {'debug_display': LaunchConfiguration('debug_display')},
        ],
        condition=IfCondition(LaunchConfiguration('enable_pick')),
    )

    kinematics_node = Node(
        package='kinematics',
        executable='search_kinematics_solutions',
        name='kinematics',
        output='screen',
    )

    actions = [
        debug_display_arg,
        enable_camera_arg,
        enable_pick_arg,
        depth_camera_launch,
        controller_launch,
        shape_recognition_node,
        automatic_pick_node,
        kinematics_node,
    ]

    return LaunchDescription(actions)
