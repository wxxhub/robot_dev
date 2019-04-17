#coding:utf-8

import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

def generate_launch_description():
    parameters = None

    included_manager_launch = launch.actions.include_launch_description.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            get_package_share_directory('module_manager') + '/launch/module_manager.launch.py'))

    included_usb_cam_launch = launch.actions.include_launch_description.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            get_package_share_directory('usb_cam') + '/launch/usb_cam.launch.py'))

    included_ball_lab_launch = launch.actions.include_launch_description.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            get_package_share_directory('ball_lab_detector') + '/launch/ball_lab_detector.launch.py'))

    behavior_manager_node = launch_ros.actions.Node(
            package="behaviors", node_executable="behavior_manager", node_name="hebaviour_manager_launch",
            output="screen", parameters=parameters)

    return LaunchDescription([
        included_manager_launch,
        included_usb_cam_launch,
        included_ball_lab_launch,
        behavior_manager_node,
    ])