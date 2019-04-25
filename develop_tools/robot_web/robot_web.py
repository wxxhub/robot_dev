#coding:utf-8

import rclpy
import sys
from app.app import createApp
from app.ros_node.ros_node import RosNode

def main():
    RosNode.run()
    app = createApp(sys.argv)
    app.run('0.0.0.0')

if __name__ == '__main__':
    main()