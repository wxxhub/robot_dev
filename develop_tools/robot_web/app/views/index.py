from flask import Blueprint, redirect, render_template, request, url_for, Response

from app.ros_node.ros_node import RosNode
from app.views.urls import blueprint_urls, nav_items

index_bp = Blueprint(
    'index',
    __name__,
    template_folder='templates'
)

@index_bp.route('/')
def index():
    body = 'vision_control.html'
    return render_template('index.html', nav_items=nav_items, body=body)

def gen(camera):
    frame = RosNode.getOrigionImage()
    # while True:
    #     frame = RosNode.getOrigionImage()
        # 使用generator函数输出视频流， 每次请求输出的content类型是image/jpeg
        # yield (b'--frame\r\n'
        #        b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

@index_bp.route('/origion_video_feed')
def origionVideoFeed():
    return Response(gen(RosNode),mimetype='multipart/x-mixed-replace; boundary=frame')   
    pass
