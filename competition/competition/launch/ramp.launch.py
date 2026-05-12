#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    set_need_compile = SetEnvironmentVariable(
        'need_compile',
        os.environ.get('need_compile', 'False'),
    )
    set_machine_type = SetEnvironmentVariable(
        'MACHINE_TYPE',
        os.environ.get('MACHINE_TYPE', 'ROSLander_Mecanum'),
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='是否启用坡道调试',
    )
    debug_display_arg = DeclareLaunchArgument(
        'debug_display',
        default_value='true',
        description='是否启用坡道调试显示',
    )
    broadcast_arg = DeclareLaunchArgument(
        'broadcast',
        default_value='false',
        description='是否播报状态（如 find_target）',
    )

    peripherals_pkg = get_package_share_directory('peripherals')
    controller_pkg = get_package_share_directory('controller')

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_pkg, 'launch', 'depth_camera.launch.py'),
        ),
    )
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_pkg, 'launch', 'lidar.launch.py'),
        ),
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_pkg, 'launch', 'controller.launch.py'),
        ),
    )

    ramp_node = Node(
        package='competition',
        executable='ramp',
        name='ramp',
        output='screen',
        parameters=[
            {'debug': LaunchConfiguration('debug')},
            {'broadcast': LaunchConfiguration('broadcast')},
            {'debug_display': LaunchConfiguration('debug_display')},
        ],
    )

    actions = [
        set_need_compile,   
        set_machine_type,
        debug_arg,
        debug_display_arg,
        broadcast_arg,
        depth_camera_launch,
        lidar_launch,
        controller_launch,
        ramp_node,
    ]

    return LaunchDescription(actions)
