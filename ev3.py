import rpyc
import os
import sys
import time
import math

# Variables
DRIVE_SPEED = 100  # Speed in percent
TURN_SPEED = 100  # Speed in percent
DRIVE_DELAY = 0.1  # Delay between each loop of driving
FULL_TURN_TIME = 1.2 # Time it takes to spin 360 degrees, in seconds
IP_ADDRESS = "192.168.1.240" # The IP of the robot

# Connect to the robot and get modules
conn = rpyc.classic.connect(IP_ADDRESS)
ev3_motor = conn.modules['ev3dev2.motor']
ev3_button = conn.modules['ev3dev2.button']

# Variable for robot
ROBOT_GLOBAL = None


class Robot:
    def __init__(self, buttons: ev3_button.Button, motors: ev3_motor.MoveTank, current_pos: tuple[int, int] = (0, 0)) -> None:
        self.buttons = buttons
        self.motors = motors
        self.pos_history: list[tuple[int, int]] = []
        self.current_pos = current_pos
        self.direction = 0.0
        self.stopped = False

    def forward(self) -> None:
        if not self.stopped:
            print("going forwards")
            self.motors.on(left_speed=ev3_motor.SpeedPercent(DRIVE_SPEED), right_speed=ev3_motor.SpeedPercent(DRIVE_SPEED))

    def backwards(self) -> None:
        if not self.stopped:
            print("going backwards")
            self.motors.on(left_speed=ev3_motor.SpeedPercent(-DRIVE_SPEED), right_speed=ev3_motor.SpeedPercent(-DRIVE_SPEED))

    def turn_left(self) -> None:
        if not self.stopped:
            print("turning left")
            self.motors.on(left_speed=ev3_motor.SpeedPercent(-TURN_SPEED), right_speed=ev3_motor.SpeedPercent(TURN_SPEED))

    def turn_right(self) -> None:
        if not self.stopped:
            print("turning right")
            self.motors.on(left_speed=ev3_motor.SpeedPercent(TURN_SPEED), right_speed=ev3_motor.SpeedPercent(-TURN_SPEED))

    # This function is blocking!
    def turn_to_direction(self, direction: float) -> None:
        if not self.stopped and direction != self.direction and abs(direction - self.direction) != 2*math.pi:
            # get which way to turn
            diff_in_angle = 0
            if self.direction < direction:
                if (direction - self.direction) > 1*math.pi:
                    diff_in_angle = abs((direction - 2*math.pi)-self.direction)
                    self.turn_left()
                else:
                    diff_in_angle = direction - self.direction
                    self.turn_right()
            elif self.direction > direction:
                if (self.direction - direction) > 1*math.pi:
                    diff_in_angle = abs((self.direction - 2*math.pi)-direction)
                    self.turn_right()
                else:
                    diff_in_angle = self.direction - direction
                    self.turn_left()

            time_to_turn = FULL_TURN_TIME * (diff_in_angle / (2*math.pi))

            print(f"Cd: {self.direction}, Nd: {direction}, diff: {diff_in_angle}, time: {time_to_turn}")

            time_taken = 0
            if time_to_turn > 0:
                start_time = time.time()
                while time_taken < time_to_turn and not self.buttons_pressed():
                    time_taken = time.time() - start_time

    def drive(self, pos: tuple[int, int]) -> None:
        if not self.stopped:
            # Turn to face the next node
            print(pos)
            print(self.current_pos)
            angle_to_node = math.atan2(self.current_pos[1] - pos[1], self.current_pos[0] - pos[0])
            if angle_to_node == self.direction:
                print(f"Robot already in correct direction, presumably... Driving to node")
            else:
                print(f"Robot has to be in direction {angle_to_node} to get to the next node, current direction is {self.direction} - turning...")
                self.turn_to_direction(angle_to_node) # THIS CALL IS BLOCKING!
                print(f"Done turning, driving to the node")

            self.forward()

    def stop(self) -> None:
        self.stopped = True
        self.motors.off()

    def set_position(self, pos: tuple[int, int]) -> None:
        self.pos_history.append(pos)
        self.current_pos = pos
    
    def set_direction(self, direction: float) -> None:
        self.direction = direction

    def buttons_pressed(self) -> bool:
        return self.buttons.any()


def debug(*args, **kwargs) -> None:
    print(*args, **kwargs, file=sys.stderr)


def setup() -> None:
    os.system('setfont Lat15-TerminusBold14')  # Sets the console font
    print('\x1Bc', end='')  # Resets the console
    global ROBOT_GLOBAL
    ROBOT_GLOBAL = get_robot()


def get_robot(current_pos: tuple[int, int] = (0, 0)) -> Robot:
    # The motors and other things on the robot
    buttons = ev3_button.Button()  # Any buton on the robot
    motors = ev3_motor.MoveTank(left_motor_port=ev3_motor.OUTPUT_A, right_motor_port=ev3_motor.OUTPUT_D)  # Motor on output port A and D

    return Robot(buttons, motors, current_pos)
