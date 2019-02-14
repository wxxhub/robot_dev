#coding:utf-8
import os

from launch import LaunchDescription
import launch.actions
import launch_ros.actions


def generate_launch_description():
    offset_file_path = os.path.dirname(os.path.realpath(__file__))+'/data/offset.yaml'
    robot_file_path = os.path.dirname(os.path.realpath(__file__))+'/config/OP3.robot'
    init_file_path = os.path.dirname(os.path.realpath(__file__))+'/config/dxl_init_OP3.yaml'
    parameters = [{'offset_file_path':offset_file_path},{'robot_file_path':robot_file_path},{'init_file_path':init_file_path},{'device_name':'/dev/ttyUSB0'}]
    return LaunchDescription([
        # Realsense
        launch_ros.actions.Node(
            package='op3_manager', node_executable='op3_manager',node_name='op3_manager',
            output='screen', parameters=parameters),
    ])


    # parameters=[{ 'use_sim_time': use_sim_time}, { 'yaml_filename': map_yaml_file }]

