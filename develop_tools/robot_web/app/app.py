#coding:utf-8
from flask import Flask, render_template
from app.views import blue_prints

def createApp(argv):
    # print (argv)
    app = Flask(__name__, static_url_path='')
    app.config['ENV'] = True
    if 'debug' in argv:
        app.config['DEBUG'] = True
    for bp in blue_prints:
        app.register_blueprint(bp)

    return app