#coding:utf-8

import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

def includeLaunch(package_name, launch_file):
    return launch.actions.include_launch_description.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            get_package_share_directory(package_name) + launch_file))

def generate_launch_description():
    parameters = None

    manager_launch = includeLaunch('module_manager', '/launch/module_manager.launch.py')
    usb_cam_launch = includeLaunch('usb_cam', '/launch/usb_cam.launch.py')
    ball_lab_launch = includeLaunch('ball_lab_detector', '/launch/ball_lab_detector.launch.py')
    road_launch = includeLaunch('road_detector', '/launch/road_detector.launch.py')

    behavior_manager_node = launch_ros.actions.Node(
            package="behaviors", node_executable="behavior_manager",
            output="screen", parameters=parameters)

    return LaunchDescription([
        usb_cam_launch,
        ball_lab_launch,
        road_launch,
        manager_launch,
        behavior_manager_node,
    ])