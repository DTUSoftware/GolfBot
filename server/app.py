from flask import Flask, Blueprint

bp = Blueprint('ev3', __name__)


@bp.route('/drive_forward')
def drive_forward():
    return 'forward'


@bp.route('/drive_backwards')
def drive_backwards():
    return 'backwards'


@bp.route("/turn_left")
def turn_left():
    return "turn left"


@bp.route("/turn_right")
def turn_right():
    return "turn right"


@bp.route("/toggle_fans")
def toggle_fans():
    return "toggle fans"

@bp.route("/stop_robot")
def stop_robot():
    return "stop robot"


app = Flask(__name__)
app.register_blueprint(bp, url_prefix='/api/v1')

if __name__ == '__main__':
    app.run()
