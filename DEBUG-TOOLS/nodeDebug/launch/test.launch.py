#coding:utf-8
import os

import launch.actions
import launch_ros.actions
from launch import LaunchDescription

def generate_launch_description():
	return LaunchDescription([
        # Realsense
        launch_ros.actions.Node(
            package='nodeDebug', node_executable='node_debug', node_name='node_debug_launch',
            output='screen'),
    ])
