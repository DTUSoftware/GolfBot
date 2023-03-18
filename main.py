#!/usr/bin/env python3
# Use python 3.5.3 or similar (tested with Python 3.6.15)
import os
import sys
import time
import ev3

def race():
    print("Starting race...")

    robot = ev3.ROBOT_GLOBAL
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
            time.sleep(DRIVE_DELAY)
        except Exception as e:
            print("uh oh... - " + str(e))


if __name__ == '__main__':
    try:
        ev3.setup()
        race()
    except KeyboardInterrupt:
        ev3.ROBOT_GLOBAL.stop()
        raise KeyboardInterrupt()
