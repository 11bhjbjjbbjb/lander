#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='是否启用调试模式',
    )

    debug_display_arg = DeclareLaunchArgument(
        'debug_display',
        default_value='true',
        description='是否启用调试显示窗口',
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/gemini_camera/rgb/image_raw',
        description='摄像头 RGB 图像话题（astra 为 /astra_camera/rgb/image_raw）',
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

    automatic_pick_node = Node(
        package='competition',
        executable='automatic_pick',
        name='position_correction',
        output='screen',
        parameters=[
            {'debug': LaunchConfiguration('debug')},
            {'debug_display': LaunchConfiguration('debug_display')},
            {'image_topic': LaunchConfiguration('image_topic')},
        ],
    )

    actions = [
        debug_arg,
        debug_display_arg,
        image_topic_arg,
        depth_camera_launch,
        controller_launch,
        automatic_pick_node,
    ]

    return LaunchDescription(actions)
