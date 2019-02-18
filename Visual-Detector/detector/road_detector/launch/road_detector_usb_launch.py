from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='road_detector', node_executable='road_detector_form_usb', node_name='road_detector_form_usb', output='screen'
        )
    ])