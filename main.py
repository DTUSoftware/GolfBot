#!/usr/bin/env python3
# Use python 3.5.3 or similar (tested with Python 3.6.15)
import os
import sys
import time
import ev3
import drive_algorithm as drivealg

def test_robot_get_pos(old_pos):
    return (old_pos[0]+10, old_pos[1]+10)


def race():
    print("Starting race...")

    robot = ev3.ROBOT_GLOBAL
    track = drivealg.TRACK_GLOBAL

    start_time = time.time()
    time_taken = 0

    while time_taken <= 8*60 and not robot.buttons_pressed():
        time_taken = time.time() - start_time
        try:
            # Get current robot position and update track robot position
            # ToDo: get new current pos from camera instead
            current_pos = test_robot_get_pos(robot.current_pos)
            print(f"Current robot position: {current_pos}")
            robot.set_position(current_pos)
            track.set_robot_pos(robot.current_pos)

            # Recalibrate the direction / angle
            last_pos = robot.pos_history[-2]
            new_angle = math.atan2(robot.current_pos[1] - last_pos[1], robot.current_pos[0] - last_pos[0])
            print(f"New robot direction: {new_angle}")
            robot.set_direction(new_angle)

            # Get the path to closest ball
            track.calculate_path()
            # Get the first node on the path
            next_node = track.path[1]
            print(f"Next node pos: ({next_node.x}, {next_node.y})")

            # Tell the robot to drive towards the node
            robot.drive((next_node.x, next_node.y)) # THIS CALL IS BLOCKING!
            
            # ToDo: Sleep, but actually instead we would just recieve input from the camera/AI, which also takes a small amount of time
            time.sleep(DRIVE_DELAY)
        except Exception as e:
            print("uh oh... - " + str(e))


if __name__ == '__main__':
    try:
        # Setup the robot
        ev3.setup()
    
        # Setup the track / driving algorithm
        drivealg.setup_debug()

        # Do the race
        race()
    except KeyboardInterrupt:
        ev3.ROBOT_GLOBAL.stop()
        raise KeyboardInterrupt()
