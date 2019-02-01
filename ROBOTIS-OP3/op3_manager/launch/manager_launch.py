#coding:utf-8
import os

from launch import LaunchDescription
import launch.actions
import launch_ros.actions


def generate_launch_description():
    offset_file_path = 'default_rviz'
    parameters = [{'default_rviz':'default_rviz'}]
    return LaunchDescription([
        # Realsense
        launch_ros.actions.Node(
            package='op3_manager', node_executable='op3_manager',node_name='op3_manager',
            output='screen', kwargs=parameters),
    ])

 def _create_node(self, *, parameters=None, remappings=None):
        return launch_ros.actions.Node(
            package='demo_nodes_py', node_executable='talker_qos', output='screen',
            # The node name is required for parameter dicts.
            # See https://github.com/ros2/launch/issues/139.
            node_name='my_node', node_namespace='my_ns',
            arguments=['--number_of_cycles', '1'],
            parameters=parameters,
            remappings=remappings,
)

    # parameters=[{ 'use_sim_time': use_sim_time}, { 'yaml_filename': map_yaml_file }]

