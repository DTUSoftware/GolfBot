#!/usr/bin/env python3
# Use python 3.5.3 or similar (tested with Python 3.6.15)
import os
import sys
import time
import math
import traceback
import ev3
import drive_algorithm as drivealg


def test_robot_get_pos(old_pos: tuple[int, int]) -> tuple[int, int]:
    return old_pos[0] + 1, old_pos[1] + 1


def race() -> None:
    print("Starting race...")

    robot = ev3.ROBOT_GLOBAL
    track = drivealg.TRACK_GLOBAL

    start_time = time.time()
    time_taken = 0

    # while True:
    #     debug_turn()

    while time_taken <= 8 * 60 and not robot.buttons_pressed():
        time_taken = time.time() - start_time
        try:
            # Get current robot position and update track robot position
            # ToDo: get new current pos from camera instead
            if track.path and len(track.path) > 1:
                current_pos = (track.path[1].node.x, track.path[1].node.y)
            else:
                current_pos = test_robot_get_pos(robot.current_pos)
            print(f"Current robot position: {current_pos}")
            robot.set_position(current_pos)
            track.set_robot_pos(robot.current_pos)

            # Recalibrate the direction / angle
            if len(robot.pos_history) > 1:
                last_pos = robot.pos_history[-2]
                new_angle = math.atan2(robot.current_pos[1] - last_pos[1], robot.current_pos[0] - last_pos[0])
                print(f"New robot direction: {new_angle}")
                robot.set_direction(new_angle)

            # Get the path to closest ball
            track.calculate_path()
            # track.draw(True)
            # Get the first node on the path
            if track.path and len(track.path) > 1:
                next_node = track.path[1]
                print(f"Next node pos: ({next_node.node.x}, {next_node.node.y})")

                # Tell the robot to drive towards the node
                robot.drive((next_node.node.x, next_node.node.y))  # THIS CALL IS BLOCKING!

            # ToDo: Sleep, but actually instead we would just recieve input from the camera/AI, which also takes a small amount of time
            time.sleep(ev3.DRIVE_DELAY)
        except Exception as e:
            print("uh oh... - " + str(e))
            traceback.print_exc()


def debug_turn(robot: ev3.Robot) -> None:
    print("turning to 0")
    robot.turn_to_direction(0)
    robot.motors.off()
    robot.set_direction(0)
    time.sleep(1)
    print("turning to 0.5")
    robot.turn_to_direction(0.5 * math.pi)
    robot.motors.off()
    robot.set_direction(0.5 * math.pi)
    time.sleep(1)
    print("turning to 1")
    robot.turn_to_direction(1 * math.pi)
    robot.motors.off()
    robot.set_direction(1 * math.pi)
    time.sleep(1)
    print("turning to 1.5")
    robot.turn_to_direction(1.5 * math.pi)
    robot.motors.off()
    robot.set_direction(1.5 * math.pi)
    time.sleep(1)
    print("turning to 1")
    robot.turn_to_direction(1 * math.pi)
    robot.motors.off()
    robot.set_direction(1 * math.pi)
    time.sleep(1)
    print("turning to 0.5")
    robot.turn_to_direction(0.5 * math.pi)
    robot.motors.off()
    robot.set_direction(0.5 * math.pi)
    time.sleep(1)
    print("turning to 2")
    robot.turn_to_direction(2 * math.pi)
    robot.motors.off()
    robot.set_direction(2 * math.pi)
    time.sleep(1)


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
