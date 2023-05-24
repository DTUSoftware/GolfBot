from flask import Flask, Blueprint, request
import ev3
import math

VERSION = "v1"

server = Blueprint('ev3', __name__)
robot: ev3.Robot


@server.before_request
def check_robot_connection():
    if not robot:
        return "Robot not initialized.", 500


@server.route('/drive', methods=['POST'])
def drive():
    direction = request.args.get("direction")
    if direction:
        if "forward" in str(direction).lower():
            if robot.forward():
                return "Driving forward.", 200
            return "Failed to drive forward.", 500
        elif "back" in str(direction).lower():
            if robot.backwards():
                return "Driving backwards.", 200
            return "Failed to drive backwards.", 500
        return "Invalid direction, try 'forward' or 'backward'.", 400
    x = request.args.get("x")
    y = request.args.get("y")
    if x and y:
        if robot.drive((float(x), float(y))):
            return f"Driving to ({x}, {y}).", 200
        return f"Failed to try and drive to ({x}, {y}).", 500
    return "Please provide either 'direction' or position using 'x' and 'y'.", 400


@server.route("/turn", methods=['POST'])
def turn():
    direction = request.args.get("direction")
    if direction:
        if str(direction).lower() == "left":
            if robot.turn_left():
                return "Turning left.", 200
            return "Failed to turn left.", 500
        elif str(direction).lower() == "right":
            if robot.turn_right():
                return "Turning right.", 200
            return "Failed to turn right.", 500
        return "Invalid direction, try 'left' or 'right'.", 400
    radians = request.args.get("radians")
    degrees = request.args.get("degrees")
    if radians or degrees:
        if radians:
            if robot.turn_to_direction(float(radians)):
                return f"Turning to {radians}", 200
            return f"Failed to turn to {radians}.", 500
        elif degrees:
            radians = math.radians(float(degrees))
            if robot.turn_to_direction(radians):
                return f"Turning to {degrees} ({str(radians)} radians)", 200
            return f"Failed to turn to {degrees} ({str(radians)} radians).", 500
    return "Please provide either 'direction', 'radians' or 'degrees'.", 400


@server.route("/toggle_fans", methods=['POST'])
def toggle_fans():
    if robot.toggle_fans():
        return "Toggled the fans.", 200
    return "Failed to toggle the fans.", 500


@server.route("/stop", methods=['POST'])
def stop_robot():
    if robot.stop():
        return "Stopped the robot.", 200
    return "Failed to stop the robot. Is it already stopped?", 500


@server.route("/start", methods=['POST'])
def start_robot():
    if robot.start():
        return "Started the robot.", 200
    return "Failed to start the robot. Is it already started?", 500


@server.route("/status", methods=['GET'])
def robot_status():
    return f"Stopped: {robot.stopped}\n" \
           f"Busy: {robot.busy}\n" \
           f"Position: {robot.current_pos}\n" \
           f"Direction: {robot.direction}", \
           200


app = Flask(__name__)
app.register_blueprint(server, url_prefix=f"/api/{VERSION}")

if __name__ == '__main__':
    global robot
    try:
        # Setup the robot
        ev3.setup()
        # Get the robot
        robot = ev3.ROBOT_GLOBAL

        # Start the app
        app.run()
    except KeyboardInterrupt:
        if robot:
            robot.stop()
        raise KeyboardInterrupt()
