import rpyc
import os
import sys
import time
import math

# Variables
DRIVE_SPEED = 100  # Speed in percent
TURN_SPEED = 100  # Speed in percent
FAN_TOGGLE_SPEED = 100  # Speed in percent
DRIVE_DELAY = 0.1  # Delay between each loop of driving
FULL_TURN_TIME = 1.2  # Time it takes to spin 360 degrees, in seconds
FAN_MOTOR_DEGREES = 80  # Degrees for turning fans off and on (off, on)
IP_ADDRESS = "192.168.1.240"  # The IP of the robot

# Connect to the robot and get modules
conn = rpyc.classic.connect(IP_ADDRESS)
ev3_motor = conn.modules['ev3dev2.motor']
ev3_button = conn.modules['ev3dev2.button']

# Variable for robot
ROBOT_GLOBAL = None


class Robot:
    def __init__(self, buttons: ev3_button.Button, motors: ev3_motor.MoveTank, fan_motor = ev3_motor.Motor, current_pos: tuple = (0, 0)) -> None:
        self.buttons = buttons
        self.fan_motor = fan_motor
        self.motors = motors
        self.pos_history: list(tuple) = []
        self.current_pos = current_pos
        self.direction = 0.0
        self.fan_state = False
        self.stopped = False
        self.busy = False

    def forward(self) -> bool:
        if not self.stopped:
            print("going forwards")
            self.motors.on(left_speed=ev3_motor.SpeedPercent(DRIVE_SPEED), right_speed=ev3_motor.SpeedPercent(DRIVE_SPEED))
            return True
        return False

    def backwards(self) -> bool:
        if not self.stopped:
            print("going backwards")
            self.motors.on(left_speed=ev3_motor.SpeedPercent(-DRIVE_SPEED), right_speed=ev3_motor.SpeedPercent(-DRIVE_SPEED))
            return True
        return False

    def turn_left(self, radians=None) -> bool:
        if not self.stopped:
            print("turning left")
            if radians:
                self.motors.on_for_degrees(left_speed=ev3_motor.SpeedPercent(-TURN_SPEED),
                                           right_speed=ev3_motor.SpeedPercent(TURN_SPEED),
                                           degrees=math.degrees(radians))
            else:
                self.motors.on(left_speed=ev3_motor.SpeedPercent(-TURN_SPEED),
                               right_speed=ev3_motor.SpeedPercent(TURN_SPEED))
            return True
        return False

    def turn_right(self, radians=None) -> bool:
        if not self.stopped:
            print("turning right")
            if radians:
                self.motors.on_for_degrees(left_speed=ev3_motor.SpeedPercent(TURN_SPEED),
                                           right_speed=ev3_motor.SpeedPercent(-TURN_SPEED),
                                           degrees=math.degrees(radians))
            else:
                self.motors.on(left_speed=ev3_motor.SpeedPercent(TURN_SPEED),
                               right_speed=ev3_motor.SpeedPercent(-TURN_SPEED))
            return True
        return False

    def toggle_fans(self) -> bool:
        if self.fan_state:
            self.fan_motor.on_for_degrees(ev3_motor.SpeedPercent(FAN_TOGGLE_SPEED), FAN_MOTOR_DEGREES)
        else:
            self.fan_motor.on_for_degrees(ev3_motor.SpeedPercent(-FAN_TOGGLE_SPEED), FAN_MOTOR_DEGREES)
        self.fan_state = not self.fan_state
        return True

    # This function is blocking!
    def turn_to_direction(self, direction: float) -> bool:
        if not self.stopped and direction != self.direction and abs(direction - self.direction) != 2*math.pi:
            # get which way to turn
            diff_in_angle = 0
            if self.direction < direction:
                if (direction - self.direction) > 1*math.pi:
                    diff_in_angle = abs((direction - 2*math.pi)-self.direction)
                    self.turn_left(diff_in_angle)
                else:
                    diff_in_angle = direction - self.direction
                    self.turn_right(diff_in_angle)
            elif self.direction > direction:
                if (self.direction - direction) > 1*math.pi:
                    diff_in_angle = abs((self.direction - 2*math.pi)-direction)
                    self.turn_right(diff_in_angle)
                else:
                    diff_in_angle = self.direction - direction
                    self.turn_left(diff_in_angle)

            # time_to_turn = FULL_TURN_TIME * (diff_in_angle / (2*math.pi))
            #
            # print(f"Cd: {self.direction}, Nd: {direction}, diff: {diff_in_angle}, time: {time_to_turn}")
            #
            # time_taken = 0
            # if time_to_turn > 0:
            #     start_time = time.time()
            #     while time_taken < time_to_turn and not self.buttons_pressed():
            #         time_taken = time.time() - start_time
            return True
        return False

    def drive(self, pos: tuple) -> bool:
        if not self.stopped:
            # Turn to face the next node
            print(pos)
            print(self.current_pos)
            angle_to_node = math.atan2(self.current_pos[1] - pos[1], self.current_pos[0] - pos[0])
            if angle_to_node == self.direction:
                print(f"Robot already in correct direction, presumably... Driving to node")
            else:
                print(f"Robot has to be in direction {angle_to_node} to get to the next node, current direction is {self.direction} - turning...")
                self.turn_to_direction(angle_to_node)  # THIS CALL IS BLOCKING!
                print(f"Done turning, driving to the node")

            self.forward()
            return True
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
        return True
    
    def set_direction(self, direction: float) -> bool:
        self.direction = direction
        return True

    def buttons_pressed(self) -> bool:
        return self.buttons.any()


def debug(*args, **kwargs) -> None:
    print(*args, **kwargs, file=sys.stderr)


def setup() -> None:
    os.system('setfont Lat15-TerminusBold14')  # Sets the console font
    print('\x1Bc', end='')  # Resets the console
    global ROBOT_GLOBAL
    ROBOT_GLOBAL = get_robot()


def get_robot(current_pos: tuple = (0, 0)) -> Robot:
    # The motors and other things on the robot
    buttons = ev3_button.Button()  # Any buton on the robot
    motors = ev3_motor.MoveTank(left_motor_port=ev3_motor.OUTPUT_A, right_motor_port=ev3_motor.OUTPUT_D)  # Motor on output port A and D
    fan_motor = ev3_motor.Motor(ev3_motor.OUTPUT_C)

    return Robot(buttons, motors, fan_motor, current_pos)
