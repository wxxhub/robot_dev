import rclpy

from std_msgs.msg import String
from std_srvs.srv import SetBool


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('minimal_publisher')
    client = node.create_client(SetBool, '/road_detector/enable')

    req = SetBool.Request()
    req.data = True
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result is not None:
        node.get_logger.info('Service call success')
    else:
        node.get_logger.info('Service call failed')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()