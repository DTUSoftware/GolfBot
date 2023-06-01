#!/usr/bin/env python3
import os
import time
import math
import traceback
import requests
import random
# import multiprocessing
import threading
import queue
from ai.main import run_ai
from drive_algorithm import Ball, Track
from track_setup import setup_track

ROBOT_API_ENDPOINT = os.environ.get('API_ENDPOINT', "http://localhost:8069/api/v1")
GOLF_BALL_CONFIDENCE_GATE = float(os.environ.get('GOLF_BALL_CONFIDENCE_GATE', 0.45))


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


def box_confidence(box):
    return box.conf.item()


def box_to_pos(box) -> tuple:
    x1, y1, x2, y2 = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
    return int((x1 + x2) / 2), int((y1 + y2) / 2)


def race(track: Track) -> None:
    print("Starting AI thread...")

    # Create the camera and AI queues
    camera_queue = queue.Queue(maxsize=1)

    # Start the camera processing thread
    camera_thread = threading.Thread(target=run_ai, args=(camera_queue,))
    camera_thread.daemon = True  # Set the thread as daemon so it exits when the main thread ends
    camera_thread.start()

    print("Getting initial AI result.")
    ai_results = camera_queue.get()
    print("Got result, marking as ready.")
    camera_queue.task_done()
    print("AI thread ready.")

    print("Strating race...")
    start_time = time.time()
    time_taken = 0

    # while True:
    #     debug_turn()

    # while True:
    #     toogle_fans_debug()

    while time_taken <= 8 * 60:
        try:
            # Get results from AI
            # ai_results, evt = camera_queue.get(block=True, timeout=None)
            ai_results = camera_queue.get(block=False)
            print("Got results from AI!")

            robot_results = []
            golf_ball_results = []
            golden_ball_results = []
            # https://docs.ultralytics.com/modes/predict/#working-with-results
            for result in ai_results:
                boxes = result.boxes
                for box in boxes:
                    class_name = result.names[int(box.cls)]
                    if "robot" in class_name:
                        robot_results.append(box)
                    elif "golf-balls" in class_name:
                        if any(x in class_name for x in ["golden", "yellow"]):
                            golden_ball_results.append(box)
                        else:
                            golf_ball_results.append(box)

            robot_results.sort(key=box_confidence)
            golf_ball_results.sort(key=box_confidence)
            golden_ball_results.sort(key=box_confidence)

            # Get current robot position and update track robot position
            if robot_results:
                robot_box = robot_results[0]
                current_pos = box_to_pos(robot_box)
                print(f"Using robot with confidence {box_confidence(robot_box):.2f} at position ({current_pos[0]}, {current_pos[1]})")
                # if track.path and len(track.path) > 1:
                #     current_pos = (track.path[1].node.x, track.path[1].node.y)
                # else:
                #     current_pos = test_robot_get_pos()

                # print(f"Current robot position: {current_pos}")
                track.set_robot_pos(current_pos)
                requests.post(f"{ROBOT_API_ENDPOINT}/position?x={current_pos[0]}&y={current_pos[1]}")
            else:
                print("No robot on track!")

            # Add balls to track
            track.clear_balls()
            for ball_box in golf_ball_results:
                confidence = box_confidence(ball_box)
                if confidence > GOLF_BALL_CONFIDENCE_GATE:
                    ball = Ball(box_to_pos(ball_box))
                    track.add_ball(ball)
            # Add golden balls to track
            for ball_box in golden_ball_results:
                confidence = box_confidence(ball_box)
                if confidence > GOLF_BALL_CONFIDENCE_GATE:
                    ball = Ball(box_to_pos(ball_box), golden=True)
                    track.add_ball(ball)

            # Get the path to closest ball
            track.calculate_path()
            # track.draw(True)
            # Get the first node on the path
            if track.path and len(track.path) > 1:
                next_node = track.path[1]
                print(f"Next node pos: ({next_node.node.x}, {next_node.node.y})")

                # Tell the robot to drive towards the node
                # THIS CALL IS NOT BLOCKING!!!
                requests.post(f"{ROBOT_API_ENDPOINT}/drive?x={next_node.node.x}&y={next_node.node.y}")

            # Let AI know we are done with the data
            print("Marked as done with event")
            # evt.set()
            camera_queue.task_done()
        except queue.Empty:
            pass
        except Exception as e:
            print("uh oh... - " + str(e))
            traceback.print_exc()
        time_taken = time.time() - start_time


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
        while True:
            try:
                # Start robot
                requests.post(f"{ROBOT_API_ENDPOINT}/start")
                break
            except ConnectionError as e:
                pass

        # Setup the track
        track = setup_track()

        # Do the race
        race(track)
    except KeyboardInterrupt:
        requests.post(f"{ROBOT_API_ENDPOINT}/stop")
        raise KeyboardInterrupt()
    except ConnectionError:
        print("Failed to connect to robot. Is the API on?")
