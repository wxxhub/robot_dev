from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def generate_launch_description():
    parameters = [{'device_0_num':0},
                  {'device_0_name':'device0'},
                 ]
    return LaunchDescription([
        launch_ros.actions.Node(
            package='usb_cam', node_executable='usb_cam_node', node_name='usb_cam_node_launch', output='screen',
            parameters=parameters
        )
    ])