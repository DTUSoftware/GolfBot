#!/usr/bin/env python3
from ev3dev2.motor import MoveTank, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_D, SpeedPercent
from ev3dev2.button import Button
import os
import sys
import time

# Variables
DRIVE_SPEED = 100  # Speed in percent
TURN_SPEED = 10  # Speed in percent
DRIVE_DELAY = 0.1  # Delay between each loop of driving


class Robot:
    def __init__(self, buttons: Button, motors: MoveTank, current_pos: tuple = (0, 0)):
        self.buttons = buttons
        self.motors = motors
        self.current_pos = current_pos

    def forward(self):
        motors.on(left_speed=SpeedPercent(DRIVE_SPEED), right_speed=SpeedPercent(-DRIVE_SPEED))

    def backwards(self):
        motors.on(left_speed=SpeedPercent(-DRIVE_SPEED), right_speed=SpeedPercent(DRIVE_SPEED))

    def turn_left(self):
        motors.on(left_speed=SpeedPercent(TURN_SPEED), right_speed=SpeedPercent(TURN_SPEED))

    def turn_right(self):
        motors.on(left_speed=SpeedPercent(-TURN_SPEED), right_speed=SpeedPercent(-TURN_SPEED))

    def drive(self, pos: tuple):
        debug("forward")
        forward()
        sleep(2)
        debug("backwards")
        backwards()
        sleep(2)
        debug("left")
        turn_left()
        sleep(2)
        debug("right")
        turn_right()
        sleep(2)
    
    def set_position(self, pos: tuple):
        self.current_pos = pos
    
    def buttons_pressed() -> bool:
        return self.buttons.any()


def debug(*args, **kwargs):
    print(*args, **kwargs, file=sys.stderr)


def setup():
    os.system('setfont Lat15-TerminusBold14')  # Sets the console font
    print('\x1Bc', end='')  # Resets the console


def get_robot(current_pos: tuple = (0,0)) -> Robot:
    # The motors and other things on the robot
    buttons = Button()  # Any buton on the robot
    motors = MoveTank(left_motor_port=OUTPUT_A, right_motor_port=OUTPUT_D)  # Motor on output port A and D

    return Robot(buttons, motors, current_pos)


def race():
    print("Starting race...")

    robot = get_robot()
    start_time = time.time()
    time_taken = 0

    while time_taken <= 8*60 and not robot.buttons_pressed():
        time_taken = time.time() - start_time
        try:
            # ToDo: Show the robot where to go
            new_pos = (10, 0)
            robot.drive(new_pos)
            # ToDo: get new current pos from camera instead
            robot.set_position(new_pos)
            # ToDo: Sleep, but actually instead we would just recieve input from the camera/AI, which also takes a small amount of time
            sleep(DRIVE_DELAY)
        except Exception as e:
            print("uh oh... - " + str(e))


if __name__ == '__main__':
    setup()
    race()
