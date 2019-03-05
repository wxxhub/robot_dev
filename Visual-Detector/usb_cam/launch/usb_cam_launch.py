from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def generate_launch_description():
    parameters = [{'device_0_num':0},
                  {'device_0_name':'device0'},
                  {'device_1_num':1},
                  {'device_1_name':'device1'},
                  {'device_2_num':2},
                  {'device_2_name':'device2'},]
    return LaunchDescription([
        launch_ros.actions.Node(
            package='usb_cam', node_executable='usb_cam_node', node_name='usb_cam_node', output='screen',
            parameters=parameters
        )
    ])