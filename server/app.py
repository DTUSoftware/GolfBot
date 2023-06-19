from flask import Flask, Blueprint, request
import ev3
import math

# The version of the API
VERSION = "v1"

# The server blueprint
server = Blueprint('ev3', __name__)

# The robot global variable
robot = None


@server.before_request
def check_robot_connection():
    """
    Checks if the robot is connected.
    :return: None
    """
    global robot
    if not robot:
        # Setup the robot
        ev3.setup()
        # Get the robot
        robot = ev3.ROBOT_GLOBAL

        if not robot:
            return "Robot cannot be initialized.", 500


@server.route('/drive', methods=['POST'])
def drive():
    """
    Drives the robot.
    :return: The response
    """
    speed_left = request.args.get("speed_left")
    speed_right = request.args.get("speed_right")
    if speed_left and speed_right:
        if robot.set_speed(float(speed_left), float(speed_right)):
            return "Speeds set.", 200
        return "Failed to set speed.", 500
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
        if robot.drive((int(x), int(y))):
            return "Driving to (" + x + ", " + y + ").", 200
        return "Failed to try and drive to (" + x + ", " + y + ").", 500
    return "Please provide either a speed (using 'speed_left' and 'speed_right'), a direction using 'direction' or " \
           "position using 'x' and 'y'.", 400


@server.route("/turn", methods=['POST'])
def turn():
    """
    Turns the robot.
    :return: The response
    """
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
    relative_text = request.args.get("relative")
    if not relative_text:
        relative_text = "false"
    relative = relative_text.lower() == "true"
    if radians or degrees:
        if radians:
            if robot.turn_relative(float(radians)) if relative else robot.turn_to_direction(float(radians)):
                return "Turning to " + str(math.degrees(radians)) + "deg (" + radians + ")", 200
            return "Failed to turn to " + str(math.degrees(radians)) + "deg (" + radians + ")", 500
        elif degrees:
            radians = math.radians(float(degrees))
            if robot.turn_relative(float(radians)) if relative else robot.turn_to_direction(float(radians)):
                return "Turning to " + degrees + " (" + str(radians) + " radians)", 200
            return "Failed to turn to " + degrees + " (" + str(radians) + " radians).", 500
    return "Please provide either 'direction', 'radians' or 'degrees'.", 400


@server.route("/toggle_fans", methods=['POST'])
def toggle_fans():
    """
    Toggles the fans.
    :return: The response
    """
    if robot.toggle_fans():
        return "Toggled the fans.", 200
    return "Failed to toggle the fans.", 500


@server.route("/stop", methods=['POST'])
def stop_robot():
    """
    Stops the robot.
    :return: The response
    """
    if robot.stop():
        return "Stopped the robot.", 200
    return "Failed to stop the robot. Is it already stopped?", 500


@server.route("/start", methods=['POST'])
def start_robot():
    """
    Starts the robot.
    :return: The response
    """
    if robot.start():
        return "Started the robot.", 200
    return "Failed to start the robot. Is it already started?", 500


@server.route("/direction", methods=['GET', 'POST'])
def robot_direction():
    """
    Gets or sets the direction of the robot.
    :return: The response
    """
    if request.method == "GET":
        return str(robot.direction), 200
    elif request.method == "POST":
        radians = request.args.get("radians")
        degrees = request.args.get("degrees")
        if radians or degrees:
            if radians:
                if robot.set_direction(float(radians)):
                    return "Set direction to " + radians, 200
                return "Failed to set direction to " + radians, 500
            elif degrees:
                radians = math.radians(float(degrees))
                if robot.set_direction(radians):
                    return "Set direction to " + degrees + " (" + str(radians) + " radians)", 200
                return "Failed to set direction to " + degrees + " (" + str(radians) + " radians).", 500
        return "Please provide either 'radians' or 'degrees'.", 400
    return "Invalid method.", 500


@server.route("/position", methods=['GET', 'POST'])
def robot_position():
    """
    Gets or sets the position of the robot.
    :return: The response
    """
    if request.method == "GET":
        return str(robot.current_pos), 200
    elif request.method == "POST":
        x = request.args.get("x")
        y = request.args.get("y")
        if x and y:
            if robot.set_position((int(x), int(y))):
                return "Set position to (" + x + ", " + y + ").", 200
            return "Failed to set position to (" + x + ", " + y + ").", 500
        return "Please provide position using 'x' and 'y'.", 400
    return "Invalid method.", 500


@server.route("/status", methods=['GET'])
def robot_status():
    """
    Gets the status of the robot.
    :return: The response
    """
    return ("Stopped: " + str(robot.stopped) + "\n" +
            "Busy:  " + str(robot.busy) + "\n" +
            "Position: " + str(robot.current_pos) + "\n" +
            "Direction: " + str(robot.direction)), 200


# Create the app
app = Flask(__name__)
# Register the blueprint
app.register_blueprint(server, url_prefix="/api/" + VERSION)

if __name__ == '__main__':
    try:
        # Setup the robot
        ev3.setup()

        # Get the robot
        robot = ev3.ROBOT_GLOBAL

        # Start the app
        app.run()
    except KeyboardInterrupt:
        # Stop the robot
        if robot:
            robot.stop()
        raise KeyboardInterrupt()
