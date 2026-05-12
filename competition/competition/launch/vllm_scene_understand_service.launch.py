#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # 声明启动参数
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/astra_camera/rgb/image_raw',
        description='摄像头图像话题名称',
    )
    
    window_name_arg = DeclareLaunchArgument(
        'window_name',
        default_value='VLLM Scene View',
        description='预览窗口名称',
    )
    
    default_prompt_arg = DeclareLaunchArgument(
        'default_prompt',
        default_value='你是一个月球探索爱好者，帮我看看这张卡片中描述的是什么场景元素。请只回答场景元素的名称，例如：宇航员、月球陨石、太空卫星、月球车、空间站、火箭、地球等。如果卡片中没有明显的月球探索相关元素，请回答"未识别到相关元素"。',
        description='默认场景理解提示词',
    )
    
    enable_preview_arg = DeclareLaunchArgument(
        'enable_preview',
        default_value='true',
        description='是否启用摄像头预览窗口',
    )
    
    # 摄像头启动（可选）
    camera_driver_pkg = get_package_share_directory('peripherals')
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_driver_pkg, 'launch', 'depth_camera.launch.py'),
        ),
    )

    # 场景理解服务节点
    scene_node = Node(
        package='competition',
        executable='vllm_scene_understand_service',
        name='vllm_scene_understand_service',
        output='screen',
        parameters=[
            {'image_topic': LaunchConfiguration('image_topic')},
            {'window_name': LaunchConfiguration('window_name')},
            {'default_prompt': LaunchConfiguration('default_prompt')},
            {'enable_preview': LaunchConfiguration('enable_preview')},
        ],
    )

    return LaunchDescription([
        image_topic_arg,
        window_name_arg,
        default_prompt_arg,
        enable_preview_arg,
        depth_camera_launch,
        scene_node,
    ])

