#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('competition')
    camera_driver_pkg = get_package_share_directory('peripherals')

    mic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'mic_init.launch.py')
        )
    )

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_driver_pkg, 'launch', 'depth_camera.launch.py'),
        ),
    )

    yolov5_node = Node(
        package='competition',
        executable='yolov5_node',
        name='yolov5',
        output='screen',
        parameters=[{
            'engine': os.path.join(
                pkg_share,
                'navigation_transport',
                'yolo5',
                'shape_models.engine',
            ),
            'lib': os.path.join(
                pkg_share,
                'navigation_transport',
                'yolo5',
                'shape_models_libmyplugins.so',
            ),
            'classes': ['cube', 'box', 'cylinder'],
            'debug_display': True,
        }],
    )

    return LaunchDescription([
        mic_launch,
        depth_camera_launch,
        yolov5_node,
    ])
