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
ROBOT_TURN_RATIO = 2
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
        self.buttons = buttons
        self.fan_motor = fan_motor
        self.motors = motors
        self.pos_history = []
        self.current_pos = current_pos
        self.direction = 0.0
        self.fan_state = False
        self.stopped = False
        self.busy = False

    def forward(self) -> bool:
        try:
            if not self.stopped and not self.busy:
                print("going forwards")
                self.motors.on(left_speed=SpeedPercent(DRIVE_SPEED),
                               right_speed=SpeedPercent(DRIVE_SPEED))
                return True
        except EOFError:
            reset_conn()
        return False

    def backwards(self) -> bool:
        try:
            if not self.stopped and not self.busy:
                print("going backwards")
                self.motors.on(left_speed=SpeedPercent(-DRIVE_SPEED),
                               right_speed=SpeedPercent(-DRIVE_SPEED))
                return True
        except EOFError:
            reset_conn()
        return False

    def turn_left(self, radians=None, busy_override=False) -> bool:
        try:
            if not self.stopped and (not self.busy or busy_override):
                print("turning left")
                if radians:
                    print("degrees: " + str(math.degrees(radians)))
                    self.motors.on_for_degrees(left_speed=SpeedPercent(-TURN_SPEED),
                                               right_speed=SpeedPercent(TURN_SPEED),
                                               degrees=math.degrees(radians))
                else:
                    self.motors.on(left_speed=SpeedPercent(-TURN_SPEED),
                                   right_speed=SpeedPercent(TURN_SPEED))
                return True
        except EOFError:
            reset_conn()
        return False

    def turn_right(self, radians=None, busy_override=False) -> bool:
        try:
            if not self.stopped and (not self.busy or busy_override):
                print("turning right")
                if radians:
                    print("degrees: " + str(math.degrees(radians)))
                    self.motors.on_for_degrees(left_speed=SpeedPercent(TURN_SPEED),
                                               right_speed=SpeedPercent(-TURN_SPEED),
                                               degrees=math.degrees(radians))
                    # self.motors.left_motor.run_to_rel_pos(position_sp=math.degrees(radians), speed_sp=ev3_motor.SpeedPercent(TURN_SPEED))
                    # self.motors.left_motor.run_to_rel_pos(position_sp=-math.degrees(radians), speed_sp=ev3_motor.SpeedPercent(TURN_SPEED))
                    # self.motors.wait_until_not_moving()
                else:
                    self.motors.on(left_speed=SpeedPercent(TURN_SPEED),
                                   right_speed=SpeedPercent(-TURN_SPEED))
                return True
        except EOFError:
            reset_conn()
        return False

    def toggle_fans(self) -> bool:
        try:
            if self.fan_state:
                self.fan_motor.on_for_degrees(speed=SpeedPercent(FAN_TOGGLE_SPEED), degrees=FAN_MOTOR_DEGREES)
            else:
                self.fan_motor.on_for_degrees(speed=SpeedPercent(-FAN_TOGGLE_SPEED), degrees=FAN_MOTOR_DEGREES)
            self.fan_state = not self.fan_state
            return True
        except EOFError:
            reset_conn()
        return False

    # This function is blocking!
    def turn_to_direction(self, direction: float) -> bool:
        try:
            if self.stopped or self.busy or direction == self.direction or abs(direction - self.direction) == 2 * math.pi:
                return False

            self.busy = True
            # get which way to turn
            diff_in_angle = 0

            direction = direction % (2 * math.pi)
            print("direction: " + str(direction))
            print("self.direction: " + str(self.direction))

            if self.direction < direction:
                if (direction - self.direction) > math.pi:
                    # diff_in_angle = abs(direction - self.direction)
                    diff_in_angle = abs((direction - 2 * math.pi) - self.direction)
                    self.turn_left(diff_in_angle * ROBOT_TURN_RATIO, busy_override=True)
                else:
                    diff_in_angle = abs(direction - self.direction)
                    self.turn_right(diff_in_angle * ROBOT_TURN_RATIO, busy_override=True)
            else:
                if (self.direction - direction) > math.pi:
                    # diff_in_angle = abs(self.direction - direction)
                    diff_in_angle = abs((self.direction - 2 * math.pi) - direction)
                    self.turn_right(diff_in_angle * ROBOT_TURN_RATIO, busy_override=True)
                else:
                    diff_in_angle = abs(self.direction - direction)
                    self.turn_left(diff_in_angle * ROBOT_TURN_RATIO, busy_override=True)

            # Update the direction
            print("diff: " + str(diff_in_angle))
            self.direction = direction

            self.busy = False
            return True
        except EOFError:
            reset_conn()
        return False

    def set_speed(self, left_speed, right_speed) -> bool:
        try:
            if not self.stopped and not self.busy:
                print(str("Now driving with speed:\n" +
                          "- Left: " + str(left_speed) + "\n" +
                          "- Right: " + str(right_speed)))
                self.motors.left_motor.run_direct(duty_cycle_sp=left_speed)
                self.motors.right_motor.run_direct(duty_cycle_sp=right_speed)
                return True
        except EOFError:
            reset_conn()
        return False

    def drive(self, pos: tuple) -> bool:
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
                    self.turn_to_direction(angle_to_node)  # THIS CALL IS BLOCKING!
                    print("Done turning, driving to the node")

                self.forward()
                return True
        except EOFError:
            reset_conn()
        return False

    def stop(self) -> bool:
        if not self.stopped:
            self.stopped = True
            self.motors.off()
            self.busy = False
            if self.fan_state:
                self.toggle_fans()
            return True
        return False

    def start(self) -> bool:
        if self.stopped:
            self.stopped = False
            return True
        return False

    def set_position(self, pos: tuple) -> bool:
        self.pos_history.append(pos)
        self.current_pos = pos

        # Recalibrate the direction / angle
        if len(self.pos_history) > 1:
            last_pos = self.pos_history[-2]
            new_angle = math.atan2(
                self.current_pos[1] - last_pos[1], self.current_pos[0] - last_pos[0])
            # print("New robot direction: " + str(new_angle))
            self.set_direction(new_angle)

        return True

    def set_direction(self, direction: float) -> bool:
        self.direction = direction
        return True

    def buttons_pressed(self) -> bool:
        return self.buttons.any()


def debug(*args, **kwargs) -> None:
    print(*args, **kwargs, file=sys.stderr)


def setup(tries=0) -> None:
    # os.system('setfont Lat15-TerminusBold14')  # Sets the console font
    # print('\x1Bc', end='')  # Resets the console
    global ROBOT_GLOBAL
    ROBOT_GLOBAL = get_robot(tries=tries)


def reset_conn(tries=0):
    global conn, ev3_motor, ev3_button
    try:
        conn = rpyc.classic.connect(IP_ADDRESS)
        ev3_motor = conn.modules['ev3dev2.motor']
        ev3_button = conn.modules['ev3dev2.button']
    except Exception as e:
        print("Failed to connect to robot: " + str(e))

    # Keep trying 10 times
    if tries < 10:
        setup(tries + 1)

    return conn, ev3_motor, ev3_button


def get_robot(current_pos: tuple = (0, 0), tries=0) -> Optional[Robot]:
    global conn, ev3_motor, ev3_button
    if not conn:
        conn, ev3_motor, ev3_button = reset_conn(tries)
    if not conn or not ev3_motor or not ev3_button:
        return None

    # The motors and other things on the robot
    buttons = ev3_button.Button()  # Any buton on the robot
    motors = ev3_motor.MoveTank(left_motor_port=ev3_motor.OUTPUT_A,
                                right_motor_port=ev3_motor.OUTPUT_D)  # Motor on output port A and D
    fan_motor = ev3_motor.Motor(ev3_motor.OUTPUT_C)

    return Robot(buttons, motors, fan_motor, current_pos)
