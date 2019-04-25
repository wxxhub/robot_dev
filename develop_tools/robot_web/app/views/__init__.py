from app.views.index import index_bp
from app.views.vision_control import vision_control_bp
from app.views.urls import blueprint_urls

blue_prints = [
    index_bp,
    vision_control_bp,
]

nav_items = [
    {"VisionControl":"/vision_control"},
]