from flask import Blueprint, render_template, request, redirect, url_for

vision_control_bp = Blueprint(
    'vision_control_bp',
    __name__,
    template_folder='templates'
)

@vision_control_bp.route("/vision_control")
def vision_control():
    return "vision_control"