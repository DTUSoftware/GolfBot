#!/usr/bin/env python3
# Use python 3.5.3 or similar (tested with Python 3.6.15)
import os
import sys
import time
import math
import traceback
import drive_algorithm as drivealg
import requests
import random


ROBOT_API_ENDPOINT = os.environ.get('API_ENDPOINT', "http://localhost:8069/api/v1")


def test_robot_get_pos(old_pos: tuple = None) -> tuple:
    if old_pos:
        return old_pos[0] + 1, old_pos[1] + 1
    else:
        return random.randrange(5, 10), random.randrange(5, 10)


def get_robot_status():
    res = requests.get(f"{ROBOT_API_ENDPOINT}/status")
    print(res.text)


def toogle_fans_debug():
    input("Press enter to flip the switch")
    requests.post(f"{ROBOT_API_ENDPOINT}/toggle_fans")


def race() -> None:
    print("Starting race...")

    track = drivealg.TRACK_GLOBAL

    start_time = time.time()
    time_taken = 0

    # while True:
    #     debug_turn()

    # while True:
    #     toogle_fans_debug()

    while time_taken <= 8 * 60:
        time_taken = time.time() - start_time
        try:
            # Get current robot position and update track robot position
            # ToDo: get new current pos from camera instead
            if track.path and len(track.path) > 1:
                current_pos = (track.path[1].node.x, track.path[1].node.y)
            else:
                current_pos = test_robot_get_pos()
            print(f"Current robot position: {current_pos}")
            track.set_robot_pos(current_pos)
            requests.post(f"{ROBOT_API_ENDPOINT}/position?x={current_pos[0]}&y={current_pos[1]}")

            # Get the path to closest ball
            track.calculate_path()
            # track.draw(True)
            # Get the first node on the path
            if track.path and len(track.path) > 1:
                next_node = track.path[1]
                print(
                    f"Next node pos: ({next_node.node.x}, {next_node.node.y})")

                # Tell the robot to drive towards the node
                # THIS CALL IS NOT BLOCKING!!!
                requests.post(f"{ROBOT_API_ENDPOINT}/drive?x={next_node.node.x}&y={next_node.node.y}")

            # ToDo: Sleep, but actually instead we would just recieve input from the camera/AI, which also takes a small amount of time
            time.sleep(5)
        except Exception as e:
            print("uh oh... - " + str(e))
            traceback.print_exc()


def debug_turn() -> None:
    radians = 0
    print(f"turning to {radians}")
    requests.post(f"{ROBOT_API_ENDPOINT}/turn?radians={radians}")
    time.sleep(1)
    radians = 0.5 * math.pi
    print(f"turning to {radians}")
    time.sleep(1)
    radians = 1 * math.pi
    print(f"turning to {radians}")
    requests.post(f"{ROBOT_API_ENDPOINT}/turn?radians={radians}")
    time.sleep(1)
    radians = 1.5 * math.pi
    print(f"turning to {radians}")
    requests.post(f"{ROBOT_API_ENDPOINT}/turn?radians={radians}")
    time.sleep(1)
    radians = 1 * math.pi
    print(f"turning to {radians}")
    requests.post(f"{ROBOT_API_ENDPOINT}/turn?radians={radians}")
    time.sleep(1)
    radians = 0.5 * math.pi
    print(f"turning to {radians}")
    requests.post(f"{ROBOT_API_ENDPOINT}/turn?radians={radians}")
    time.sleep(1)
    radians = 2 * math.pi
    print(f"turning to {radians}")
    requests.post(f"{ROBOT_API_ENDPOINT}/turn?radians={radians}")
    time.sleep(1)


if __name__ == '__main__':
    try:
        requests.post(f"{ROBOT_API_ENDPOINT}/start")

        # Setup the track / driving algorithm
        drivealg.setup_debug()

        # Do the race
        race()
    except KeyboardInterrupt:
        requests.post(f"{ROBOT_API_ENDPOINT}/stop")
        raise KeyboardInterrupt()
