from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='ball_lab_detector', node_executable='ball_lab_detector_usbcam', node_name='ball_lab_detector_usbcam_launch', output='screen'
        )
    ])