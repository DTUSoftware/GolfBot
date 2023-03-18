import rpyc
import os
import sys
import time

# Variables
DRIVE_SPEED = 100  # Speed in percent
TURN_SPEED = 10  # Speed in percent
DRIVE_DELAY = 0.1  # Delay between each loop of driving
IP_ADDRESS = "192.168.1.240" # The IP of the robot

# Connect to the robot and get modules
conn = rpyc.classic.connect(IP_ADDRESS)
ev3_motor = conn.modules['ev3dev2.motor']
ev3_button = conn.modules['ev3dev2.button']

# Variable for robot
ROBOT_GLOBAL = None

class Robot:
    def __init__(self, buttons: ev3_button.Button, motors: ev3_motor.MoveTank, current_pos: tuple = (0, 0)):
        self.buttons = buttons
        self.motors = motors
        self.current_pos = current_pos
        self.stopped = False

    def forward(self):
        if not self.stopped:
            self.motors.on(left_speed=ev3_motor.SpeedPercent(-DRIVE_SPEED), right_speed=ev3_motor.SpeedPercent(-DRIVE_SPEED))

    def backwards(self):
        if not self.stopped:
            self.motors.on(left_speed=ev3_motor.SpeedPercent(DRIVE_SPEED), right_speed=ev3_motor.SpeedPercent(DRIVE_SPEED))

    def turn_left(self):
        if not self.stopped:
            self.motors.on(left_speed=ev3_motor.SpeedPercent(-TURN_SPEED), right_speed=ev3_motor.SpeedPercent(TURN_SPEED))

    def turn_right(self):
        if not self.stopped:
            self.motors.on(left_speed=ev3_motor.SpeedPercent(TURN_SPEED), right_speed=ev3_motor.SpeedPercent(-TURN_SPEED))

    def drive(self, pos: tuple):
        print("forward")
        self.forward()
        time.sleep(2)
        print("backwards")
        self.backwards()
        time.sleep(2)
        print("left")
        self.turn_left()
        time.sleep(2)
        print("right")
        self.turn_right()
        time.sleep(2)
    
    def stop(self):
        self.stopped = True
        self.motors.off()
    
    def set_position(self, pos: tuple):
        self.current_pos = pos
    
    def buttons_pressed(self) -> bool:
        return self.buttons.any()


def debug(*args, **kwargs):
    print(*args, **kwargs, file=sys.stderr)


def setup():
    os.system('setfont Lat15-TerminusBold14')  # Sets the console font
    print('\x1Bc', end='')  # Resets the console
    global ROBOT_GLOBAL
    ROBOT_GLOBAL = get_robot()


def get_robot(current_pos: tuple = (0,0)) -> Robot:
    # The motors and other things on the robot
    buttons = ev3_button.Button()  # Any buton on the robot
    motors = ev3_motor.MoveTank(left_motor_port=ev3_motor.OUTPUT_A, right_motor_port=ev3_motor.OUTPUT_D)  # Motor on output port A and D

    return Robot(buttons, motors, current_pos)
