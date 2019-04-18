#coding:utf-8

import rclpy
from app.app import createApp

app = createApp()

def main(args=None):
    app.run()
    print ("Hello World")

if __name__ == '__main__':
    main()