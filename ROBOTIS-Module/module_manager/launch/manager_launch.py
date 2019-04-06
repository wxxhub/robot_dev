#coding:utf-8
import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

def generate_launch_description():
	module_manager_path = get_package_share_directory('module_manager')
	offset_file_path = get_package_share_directory('tuning_module') + '/data/offset.yaml'
	robot_file_path = module_manager_path+'/config/OP3.robot'
	init_file_path = module_manager_path+'/config/dxl_init_OP3.yaml'
	parameters = [{'offset_file_path':offset_file_path},{'robot_file_path':robot_file_path},{'init_file_path':init_file_path},{'device_name':'/dev/ttyUSB0'}]
	return LaunchDescription([
        # Realsense
        launch_ros.actions.Node(
            package='module_manager', node_executable='module_manager',node_name='module_manager',
            output='screen', parameters=parameters),
    ])

    # parameters=[{ 'use_sim_time': use_sim_time}, { 'yaml_filename': map_yaml_file }]
