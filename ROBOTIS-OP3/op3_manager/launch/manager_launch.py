#coding:utf-8
import os

from launch import LaunchDescription
import launch.actions
import launch_ros.actions


def generate_launch_description():
    offset_file_path = os.path.dirname(os.path.realpath(__file__))+'/config/test.yaml'
    parameters = [{'default_rviz':'default_rviz'}]
    return LaunchDescription([
        # Realsense
        launch_ros.actions.Node(
            package='op3_manager', node_executable='op3_manager',node_name='op3_manager',
            output='screen', parameters=[{'offset_file_path':offset_file_path}]),
    ])


    # parameters=[{ 'use_sim_time': use_sim_time}, { 'yaml_filename': map_yaml_file }]

