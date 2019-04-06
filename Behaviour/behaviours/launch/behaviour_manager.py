#coding:utf-8

import os
import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

def generate_launch_description():

    parameters = None

    return LaunchDescription([
        launch_ros.actions.Node(
        package="behaviours", node_executable="behaviour_manager", node_name="hebaviour_manager",
        output="screen", parameters=parameters)
    ])