# Use python 3.5.3 or similar (tested with Python 3.6.15)
import rpyc
import os
import sys
import math
from typing import Optional
from ev3dev2.button import Button
from ev3dev2.motor import MoveTank, SpeedPercent, Motor

# Variables
DRIVE_SPEED = 20  # Speed in percent
TURN_SPEED = 20  # Speed in percent
FAN_TOGGLE_SPEED = 100  # Speed in percent
FULL_TURN_TIME = 1.2  # Time it takes to spin 360 degrees, in seconds
FAN_MOTOR_DEGREES = 80  # Degrees for turning fans off and on (off, on)
ROBOT_TURN_RATIO = 2.2
IP_ADDRESS = os.environ.get('ROBOT_IP_ADDRESS', "192.168.1.240")  # The IP of the robot

# Connect to the robot and get modules
conn = None
ev3_motor = None
ev3_button = None

# Variable for robot
ROBOT_GLOBAL = None


class Robot:
    def __init__(self, buttons: Button, motors: MoveTank, fan_motor: Motor,
                 current_pos: tuple = (0, 0)) -> None:
        """
        Initialize the robot.
        :param buttons: the buttons
        :param motors: the motors
        :param fan_motor: the fan motor
        :param current_pos: the current position of the robot
        """
        self.buttons = buttons
        self.fan_motor = fan_motor
        self.motors = motors
        self.current_pos = current_pos
        self.direction = 0.0
        self.fan_state = False
        self.stopped = False
        self.busy = False

    def refresh_conn(self) -> bool:
        """
        Refresh the connection to the robot.
        :return: True if refreshed, else False
        """
        reset_conn()
        if not conn:
            return False
        self.motors = get_motors()
        self.fan_motor = get_fan_motor()
        self.buttons = get_button()
        if not self.motors or not self.fan_motor or not self.buttons:
            return False
        return True

    def forward(self) -> bool:
        """
        Drive the robot forwards.
        :return: True if successful, False otherwise
        """
        try:
            if not self.stopped and not self.busy and self.motors:
                print("going forwards")
                self.motors.on(left_speed=SpeedPercent(DRIVE_SPEED),
                               right_speed=SpeedPercent(DRIVE_SPEED))
                return True
        except (EOFError, ReferenceError):
            if self.refresh_conn():
                return self.forward()
        return False

    def backwards(self) -> bool:
        """
        Drive the robot backwards.
        :return: True if successful, False otherwise
        """
        try:
            if not self.stopped and not self.busy and self.motors:
                print("going backwards")
                self.motors.on(left_speed=SpeedPercent(-DRIVE_SPEED),
                               right_speed=SpeedPercent(-DRIVE_SPEED))
                return True
        except (EOFError, ReferenceError):
            if self.refresh_conn():
                return self.backwards()
        return False

    def turn_left(self, radians=None, busy_override=False) -> bool:
        """
        Turn the robot left.
        :param radians: the radians to turn
        :param busy_override: whether to override the busy state
        :return: True if successful, False otherwise
        """
        try:
            if not self.stopped and (not self.busy or busy_override) and self.motors:
                if radians:
                    print("Turning wheel left " + str(math.degrees(radians)) + " deg (" + str(radians) + " rad) - with turn ratio!")
                    self.motors.on_for_degrees(left_speed=SpeedPercent(-TURN_SPEED),
                                               right_speed=SpeedPercent(TURN_SPEED),
                                               degrees=math.degrees(radians))
                else:
                    print("Turning wheel left (till told otherwise)")
                    self.motors.on(left_speed=SpeedPercent(-TURN_SPEED),
                                   right_speed=SpeedPercent(TURN_SPEED))
                return True
        except (EOFError, ReferenceError):
            if self.refresh_conn():
                return self.turn_left(radians, busy_override)
        return False

    def turn_right(self, radians=None, busy_override=False) -> bool:
        """
        Turn the robot right.
        :param radians: the radians to turn
        :param busy_override: whether to override the busy state
        :return: True if successful, False otherwise
        """
        try:
            if not self.stopped and (not self.busy or busy_override) and self.motors:
                if radians:
                    print("Turning wheel right " + str(math.degrees(radians)) + " deg (" + str(radians) + " rad) - with turn ratio!")
                    self.motors.on_for_degrees(left_speed=SpeedPercent(TURN_SPEED),
                                               right_speed=SpeedPercent(-TURN_SPEED),
                                               degrees=math.degrees(radians))
                    # self.motors.left_motor.run_to_rel_pos(position_sp=math.degrees(radians), speed_sp=ev3_motor.SpeedPercent(TURN_SPEED))
                    # self.motors.left_motor.run_to_rel_pos(position_sp=-math.degrees(radians), speed_sp=ev3_motor.SpeedPercent(TURN_SPEED))
                    # self.motors.wait_until_not_moving()
                else:
                    print("Turning wheel right (till told otherwise)")
                    self.motors.on(left_speed=SpeedPercent(TURN_SPEED),
                                   right_speed=SpeedPercent(-TURN_SPEED))
                return True
        except (EOFError, ReferenceError):
            if self.refresh_conn():
                return self.turn_right(radians, busy_override)
        return False

    def toggle_fans(self) -> bool:
        """
        Toggle the fans.
        :return: True if successful, False otherwise
        """
        try:
            if not self.fan_motor:
                return False
            if self.fan_state:
                self.fan_motor.on_for_degrees(speed=SpeedPercent(FAN_TOGGLE_SPEED), degrees=FAN_MOTOR_DEGREES)
            else:
                self.fan_motor.on_for_degrees(speed=SpeedPercent(-FAN_TOGGLE_SPEED), degrees=FAN_MOTOR_DEGREES)
            self.fan_state = not self.fan_state
            return True
        except (EOFError, ReferenceError):
            if self.refresh_conn():
                return self.toggle_fans()
        return False

    # This function is blocking!
    def turn_to_direction(self, direction: float) -> bool:
        """
        Turn the robot to a direction.
        :param direction: the direction to turn to
        :return: True if successful, False otherwise
        """
        try:
            if self.stopped or self.busy or direction == self.direction or abs(direction - self.direction) == 2 * math.pi:
                return False

            self.busy = True
            # get which way to turn
            diff_in_angle = 0

            direction = direction % (2 * math.pi)
            print("Turning to direction: " + str(math.degrees(direction)) + " deg (" + str(direction) + " rad)\n" +
                  "Current direction is " + str(math.degrees(self.direction)) + " deg (" + str(self.direction) + " deg)")

            status = False
            if self.direction < direction:
                if (direction - self.direction) > math.pi:
                    # diff_in_angle = abs(direction - self.direction)
                    diff_in_angle = abs((direction - 2 * math.pi) - self.direction)
                    status = self.turn_left(diff_in_angle * ROBOT_TURN_RATIO, busy_override=True)
                else:
                    diff_in_angle = abs(direction - self.direction)
                    status = self.turn_right(diff_in_angle * ROBOT_TURN_RATIO, busy_override=True)
            else:
                if (self.direction - direction) > math.pi:
                    # diff_in_angle = abs(self.direction - direction)
                    diff_in_angle = abs((self.direction - 2 * math.pi) - direction)
                    status = self.turn_right(diff_in_angle * ROBOT_TURN_RATIO, busy_override=True)
                else:
                    diff_in_angle = abs(self.direction - direction)
                    status = self.turn_left(diff_in_angle * ROBOT_TURN_RATIO, busy_override=True)

            # Update the direction
            print("Diff in angle from current was (without ratio): " + str(math.degrees(diff_in_angle)) + " deg (" + str(diff_in_angle) + " rad)")
            self.direction = direction

            self.busy = False
            return status
        except (EOFError, ReferenceError):
            if self.refresh_conn():
                return self.turn_to_direction(direction)
        return False

    def set_speed(self, left_speed, right_speed) -> bool:
        """
        Set the speed of the robot.
        :param left_speed: the speed of the left motor
        :param right_speed: the speed of the right motor
        :return: True if successful, False otherwise
        """
        try:
            if not self.stopped and not self.busy and self.motors:
                print(str("Now driving with speed:\n" +
                          "- Left: " + str(left_speed) + "\n" +
                          "- Right: " + str(right_speed)))
                self.motors.left_motor.run_direct(duty_cycle_sp=left_speed)
                self.motors.right_motor.run_direct(duty_cycle_sp=right_speed)
                return True
        except (EOFError, ReferenceError):
            if self.refresh_conn():
                return self.set_speed(left_speed, right_speed)
        return False

    def drive(self, pos: tuple) -> bool:
        """
        Drive to a position.
        :param pos: the position to drive to
        :return: True if successful, False otherwise
        """
        try:
            if not self.stopped and not self.busy:
                # Turn to face the next node
                print("Current pos: " + str(self.current_pos))
                print("Pos for node: " + str(pos))
                angle_to_node = math.atan2(self.current_pos[1] - pos[1], self.current_pos[0] - pos[0])
                if angle_to_node == self.direction:
                    print("Robot already in correct direction, presumably... Driving to node")
                else:
                    print(
                        "Robot has to be in direction " + str(
                            angle_to_node) + " to get to the next node, current direction is " + str(
                            self.direction) + " - turning...")
                    if not self.turn_to_direction(angle_to_node):  # THIS CALL IS BLOCKING!
                        return False
                    print("Done turning, driving to the node")

                return self.forward()
        except (EOFError, ReferenceError):
            if self.refresh_conn():
                return self.drive(pos)
        return False

    def stop(self, tries=0) -> bool:
        """
        Stop the robot.
        :param tries: the amount of tries to stop the robot
        :return: True if successful, False otherwise
        """
        if not self.stopped and tries <= 10:
            try:
                self.stopped = True
                self.motors.off()
                self.busy = False
                if self.fan_state:
                    self.toggle_fans()
                return True
            except:
                self.refresh_conn()
                # keep trying pls, I don't care if it's recursive
                return self.stop(tries=tries+1)
        return False

    def start(self) -> bool:
        """
        Start the robot.
        :return: True if successful, False otherwise
        """
        if self.stopped:
            self.stopped = False
            return True
        return False

    def set_position(self, pos: tuple) -> bool:
        """
        Set the position of the robot.
        :param pos: the position to set
        :return: True if successful, False otherwise
        """
        # Recalibrate the direction / angle
        dx = self.current_pos[0] - pos[0]
        # We need the opposite of the y-axis, since we start from the top-left, and have a y-axis that goes downwards
        dy = self.current_pos[1] - pos[1]

        angle = math.atan2(dy, dx)
        if angle < 0:
            angle += 2 * math.pi

        self.set_direction(math.atan2(dy, dx) % (2 * math.pi))
        # Update position
        self.current_pos = pos

        return True

    def set_direction(self, direction: float) -> bool:
        """
        Set the direction of the robot.
        :param direction: the direction to set
        :return: True if successful, False otherwise
        """
        self.direction = direction
        return True

    def buttons_pressed(self) -> bool:
        """
        Check if any buttons are pressed.
        :return: True if any buttons are pressed, False otherwise
        """
        if self.buttons:
            return self.buttons.any()
        return False


def debug(*args, **kwargs) -> None:
    """
    Print debug messages.
    :param args: the arguments to print
    :param kwargs: the keyword arguments to print
    :return: None
    """
    print(*args, **kwargs, file=sys.stderr)


def setup(tries=0) -> None:
    """
    Setup the robot.
    :param tries: the amount of tries to setup the robot
    :return: None
    """
    # os.system('setfont Lat15-TerminusBold14')  # Sets the console font
    # print('\x1Bc', end='')  # Resets the console
    global ROBOT_GLOBAL
    ROBOT_GLOBAL = get_robot(tries=tries)


def reset_conn(tries=0):
    """
    Reset the connection to the robot.
    :param tries: the amount of tries to reset the connection
    :return: None
    """
    global conn, ev3_motor, ev3_button, ROBOT_GLOBAL
    try:
        print("Trying to reconnect to robot...")
        conn = rpyc.classic.connect(IP_ADDRESS)
        ev3_motor = conn.modules['ev3dev2.motor']
        ev3_button = conn.modules['ev3dev2.button']
        print("Reconnected!")
    except Exception as e:
        print("Failed to connect to robot: " + str(e))
        conn = None
        ev3_motor = None
        ev3_button = None

    if not conn or not ev3_motor or not ev3_button:
        # Keep trying 10 times
        if tries < 5:
            setup(tries + 1)

    return conn, ev3_motor, ev3_button


def get_button() -> Optional[Button]:
    """
    Get the buttons on the robot.
    :return: the buttons on the robot
    """
    if not conn or not ev3_button:
        return None
    return ev3_button.Button()  # Any button on the robot


def get_motors() -> Optional[MoveTank]:
    """
    Get the main wheel motors on the robot.
    :return: the wheel motors on the robot
    """
    if not conn or not ev3_motor:
        return None
    return ev3_motor.MoveTank(left_motor_port=ev3_motor.OUTPUT_A,
                              right_motor_port=ev3_motor.OUTPUT_D)  # Motor on output port A and D


def get_fan_motor() -> Optional[Motor]:
    """
    Get the fan motor on the robot.
    :return: the fan motor on the robot
    """
    if not conn or not ev3_motor:
        return None
    return ev3_motor.Motor(ev3_motor.OUTPUT_C)


def get_robot(current_pos: tuple = (0, 0), tries=0) -> Optional[Robot]:
    """
    Get the robot.
    :param current_pos: the current position of the robot
    :param tries: the amount of tries to get the robot
    :return: the robot
    """
    global conn, ev3_motor, ev3_button
    if not conn:
        conn, ev3_motor, ev3_button = reset_conn(tries)
    if not conn or not ev3_motor or not ev3_button:
        return None

    # The motors and other things on the robot
    buttons = get_button()
    motors = get_motors()
    fan_motor = get_fan_motor()

    return Robot(buttons, motors, fan_motor, current_pos)
