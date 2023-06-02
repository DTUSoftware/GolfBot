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
DEBUG = "true" in os.environ.get('DEBUG', "True").lower()


def test_robot_get_pos(old_pos: tuple = None) -> tuple:
    if old_pos:
        return old_pos[0] + 1, old_pos[1] + 1
    else:
        return random.randrange(5, 10), random.randrange(5, 10)


def get_robot_status():
    res = requests.get(f"{ROBOT_API_ENDPOINT}/status")
    print(res.text)


def box_confidence(box):
    return box.conf.item()


def box_to_pos(box) -> tuple:
    x1, y1, x2, y2 = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
    return int((x1 + x2) / 2), int((y1 + y2) / 2)


def parse_ai_results(ai_results) -> tuple:
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

    return robot_results, golf_ball_results, golden_ball_results


def update_robot_from_ai_result(track, robot_results):
    # Get current robot position and update track robot position
    if robot_results:
        robot_box = robot_results[0]
        current_pos = box_to_pos(robot_box)
        print(
            f"Using robot with confidence {box_confidence(robot_box):.2f} at position ({current_pos[0]}, {current_pos[1]})")
        track.set_robot_pos(current_pos)
        requests.post(f"{ROBOT_API_ENDPOINT}/position?x={current_pos[0]}&y={current_pos[1]}")
    else:
        print("No robot on track!")


def update_balls_from_ai_result(track, golf_ball_results, golden_ball_results):
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


def calculate_and_adjust(track):
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


def do_race_iteration(track: Track, camera_queue: queue.Queue):
    try:
        if DEBUG:
            # Get robot status
            get_robot_status()
        
        # Get results from AI
        # ai_results, evt = camera_queue.get(block=True, timeout=None)
        ai_results = camera_queue.get(block=False)
        if DEBUG:
            print("Got results from AI!")

        # Parse the results
        robot_results, golf_ball_results, golden_ball_results = parse_ai_results(ai_results)

        # Update robot and balls
        update_robot_from_ai_result(track, robot_results)
        update_balls_from_ai_result(track, golf_ball_results, golden_ball_results)

        # Calculate track path and give the robot directions
        calculate_and_adjust(track)

        # Let AI know we are done with the data
        if DEBUG:
            print("Marked as done with event")
        # evt.set()
        camera_queue.task_done()
    except queue.Empty:
        pass
    except Exception as e:
        print("uh oh... - " + str(e))
        traceback.print_exc()


def race(track: Track) -> None:
    print("Preparing the robot and AI... Please wait!")

    # Create the camera and AI queues
    camera_queue = queue.Queue(maxsize=1)

    # Start the camera processing thread
    camera_thread = threading.Thread(target=run_ai, args=(camera_queue,))
    camera_thread.daemon = True  # Set the thread as daemon so it exits when the main thread ends
    camera_thread.start()

    if DEBUG:
        print("Getting initial AI result.")
    camera_queue.get()
    if DEBUG:
        print("Got result, marking as ready.")
    camera_queue.task_done()
    if DEBUG:
        print("AI thread ready.")

    input("Ready! Press Enter to start race!")
    start_time = time.time()
    time_taken = 0

    while time_taken <= 8 * 60:
        do_race_iteration(track, camera_queue)
        time_taken = time.time() - start_time


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
        # Do the race
        race(setup_track())
    except KeyboardInterrupt:
        requests.post(f"{ROBOT_API_ENDPOINT}/stop")
        raise KeyboardInterrupt()
    except ConnectionError:
        print("Failed to connect to robot. Is the API on?")
