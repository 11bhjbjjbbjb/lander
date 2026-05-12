#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


DEFAULT_MAP = os.path.expanduser('~/ros2_ws/src/slam/maps/map_01')


def generate_launch_description():
    competition_share = get_package_share_directory('competition')
    navigation_share = get_package_share_directory('navigation')

    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    broadcast_arg = DeclareLaunchArgument('broadcast', default_value='false')  
    slope_arg = DeclareLaunchArgument('slope_surface', default_value='true')
    map_arg = DeclareLaunchArgument('map', default_value=DEFAULT_MAP)
    sim_arg = DeclareLaunchArgument('sim', default_value='false')
    robot_arg = DeclareLaunchArgument('robot_name', default_value='')
    master_arg = DeclareLaunchArgument('master_name', default_value='')
    use_teb_arg = DeclareLaunchArgument('use_teb', default_value='true')
    map_frame_arg = DeclareLaunchArgument('map_frame', default_value='map')

    mic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(competition_share, 'launch', 'mic_init.launch.py')
        )
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'sim': LaunchConfiguration('sim'),
            'robot_name': LaunchConfiguration('robot_name'),
            'master_name': LaunchConfiguration('master_name'),
            'use_teb': LaunchConfiguration('use_teb'),
        }.items(),
    )

    vllm_scene_understand_service_node = Node(
        package='competition',
        executable='vllm_scene_understand_service',
        name='vllm_scene_understand_service',
        output='screen',
        parameters=[{
            'enable_preview': False,
        }],
    )

    kinematics_node = Node(
        package='kinematics',
        executable='search_kinematics_solutions',
        name='kinematics',
        output='screen',
    )

    yolo_model = os.path.join(
        competition_share, 'navigation_transport', 'yolo5', 'best.pt'
    )

    yolov5_node = Node(
        package='competition',
        executable='yolov5_node',
        name='yolov5',
        output='screen',
        parameters=[{
            'model_path': yolo_model,
            'conf_thresh': 0.6,
            'classes': ['cube', 'box', 'cylinder'],
            'debug_display': LaunchConfiguration('debug'),
        }],
    )

    automatic_pick_node = Node(
        package='competition',
        executable='automatic_pick',
        name='position_correction',
        output='screen',
        parameters=[{
            'debug': LaunchConfiguration('debug'),
            'broadcast': LaunchConfiguration('broadcast'),
            'debug_display': LaunchConfiguration('debug'),
        }],
    )

    ramp_node = Node(
        package='competition',
        executable='ramp',
        name='ramp',
        output='screen',
        parameters=[{
            'debug': LaunchConfiguration('debug'),
        }],
        condition=IfCondition(LaunchConfiguration('slope_surface')),
    )

    shape_recognition_node = Node(
        package='competition',
        executable='shape_recognition',
        name='shape_recognition',
        output='screen',
        respawn=True,
        parameters=[{'debug_display': LaunchConfiguration('debug')}],
    )

    voice_control_node = Node(
        package='competition',
        executable='voice_control_navigation',
        name='voice_control_navigation',
        output='screen',
        parameters=[{
            'map_frame': LaunchConfiguration('map_frame'),
            'slope_surface': LaunchConfiguration('slope_surface'),
            'debug': LaunchConfiguration('debug'),
        }],
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, 'launch', 'rviz_navigation.launch.py')
        ),
        launch_arguments={
            'sim': LaunchConfiguration('sim'),
            'robot_name': LaunchConfiguration('robot_name'),
        }.items(),
    )

    return LaunchDescription([
        debug_arg,
        broadcast_arg,
        slope_arg,
        map_arg,
        sim_arg,
        robot_arg,
        master_arg,
        use_teb_arg,
        map_frame_arg,
        mic_launch,
        navigation_launch,
        vllm_scene_understand_service_node,
        kinematics_node,
        yolov5_node,
        automatic_pick_node,
        ramp_node,
        shape_recognition_node,
        voice_control_node,
        rviz_launch,
    ])
