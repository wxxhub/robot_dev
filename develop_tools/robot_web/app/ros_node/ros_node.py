import threading

import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image

"""
## flask image test
class Test(object):
    print ("init")
    video = cv2.VideoCapture(0)

    # @classmethod
    # def __init__(self):
    #     self.video = cv2.VideoCapture(0)
    
    @classmethod
    def __del__(self):
        self.video.realse()

    @classmethod
    def getFrame(self):
        success, image = self.video.read()
        ret, jpeg = cv2.imencode('.jpg', image)
        return jpeg.tobytes()
"""

class RosNode(object):
    rclpy.init()

    node = rclpy.create_node('robot_web_node')
    origion_image = cv2.imread(None)

    @classmethod
    def __init__(self):
        
        pass
    
    @classmethod
    def run(self):
        queue_thread = threading.Thread(target=self.__queueThread)
        queue_thread.start()

    @classmethod
    def __queueThread(self):
        origion_image_sub = self.node.create_subscription(Image, '/usb_cam_pub/image0', self.__origionImageCallback)
        while rclpy.ok:
            rclpy.spin(self.node)

    @classmethod
    def __origionImageCallback(self, msg):
        # print (msg.data)
        np_arr = np.fromstring(msg.data, np.uint8)
        self.origion_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        if self.origion_image:
            cv2.imshow('test', self.origion_image)
            print ("ros node")
        pass

    @classmethod
    def getOrigionImage(self):
        return self.origion_image
