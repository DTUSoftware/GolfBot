from flask import Flask, Blueprint
import ev3

VERSION = "v1"

server = Blueprint('ev3', __name__)
robot: ev3.Robot


@server.route('/drive_forward')
def drive_forward():
    robot.forward()
    return 'forward'


@server.route('/drive_backwards')
def drive_backwards():
    robot.backwards()
    return 'backwards'


@server.route("/turn_left")
def turn_left():
    robot.turn_left()
    return "turn left"


@server.route("/turn_right")
def turn_right():
    robot.turn_right()
    return "turn right"


@server.route("/toggle_fans")
def toggle_fans():
    robot.toggle_fans()
    return "toggle fans"


@server.route("/stop_robot")
def stop_robot():
    robot.stop()
    return "stop robot"


app = Flask(__name__)
app.register_blueprint(server, url_prefix=f"/api/{VERSION}")

if __name__ == '__main__':
    try:
        # Setup the robot
        ev3.setup()
        # Get the robot
        global ROBOT_GLOBAL
        robot = ev3.ROBOT_GLOBAL

        # Start the app
        app.run()
    except KeyboardInterrupt:
        ev3.ROBOT_GLOBAL.stop()
        raise KeyboardInterrupt()
