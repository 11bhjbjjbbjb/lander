import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'competition'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.*'))),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'navigation_transport', 'yolo5'),
            glob(os.path.join('competition', 'navigation_transport', 'yolo5', '*.engine')) +
            glob(os.path.join('competition', 'navigation_transport', 'yolo5', '*.so')) +
            glob(os.path.join('competition', 'navigation_transport', 'yolo5', '*.pt'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='1270161395@qq.com',
    description='Competition package ported to ROS2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov5_node = competition.navigation_transport.yolo5.yolov5_node:main',
            'shape_recognition = competition.navigation_transport.shape_recognition.shape_recognition_down:main',
            'automatic_pick = competition.navigation_transport.calibration_position.automatic_pick:main',
            'ramp = competition.navigation_transport.ramp.ramp:main',
            'voice_control_navigation = competition.navigation_transport.voice_control_navigation:main',
            'vllm_scene_understand_service = competition.large_models.vllm_scene_understand_service:main',
        ],
    },
)
