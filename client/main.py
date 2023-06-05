#!/usr/bin/env python3
import os
import time
import math
import traceback
from typing import Optional

import requests
import random
import asyncio
import aiohttp
from torch import multiprocessing
import queue
from ai.main import run_ai
from drive_algorithm import Ball, Track, Node, NodeData
from track_setup import setup_track

ROBOT_API_ENDPOINT = os.environ.get('API_ENDPOINT', "http://localhost:8069/api/v1")
GOLF_BALL_CONFIDENCE_GATE = float(os.environ.get('GOLF_BALL_CONFIDENCE_GATE', 0.45))
DISABLE_LOGGING = "true" in os.environ.get('DISABLE_LOGGING', "False").lower()
DEBUG = ("true" in os.environ.get('DEBUG', "True").lower()) and not DISABLE_LOGGING

# PID constants
KP = float(os.environ.get('PID_KP', 5))  # Proportional gain
KI = float(os.environ.get('PID_KI', 0.1))  # Integral gain - 0.1
KD = float(os.environ.get('PID_KD', 0.05))  # Derivative gain - 0.05

# Robot parameters
WHEEL_RADIUS = (float(os.environ.get('WHEEL_DIAMETER', 68.8)) / 2) / 10  # Radius of the robot's wheels in cm
DIST_BETWEEN_WHEELS = float(
    os.environ.get('DIST_BETWEEN_WHEELS', 83.0 * 2)) / 10  # Distance between the robot's wheels in cm
ROBOT_BASE_SPEED = float(os.environ.get('ROBOT_BASE_SPEED', 50.0))

# PID variables
integral = 0
previous_error = 0
last_target_node = None


def test_robot_get_pos(old_pos: tuple = None) -> tuple:
    if old_pos:
        return old_pos[0] + 1, old_pos[1] + 1
    else:
        return random.randrange(5, 10), random.randrange(5, 10)


async def get_robot_status(session: aiohttp.ClientSession):
    print("Getting robot status")
    async with session.get(f"{ROBOT_API_ENDPOINT}/status") as response:
        print(await response.text())


def box_confidence(box):
    return box.conf.item()


def box_to_pos(box) -> tuple:
    x1, y1, x2, y2 = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
    return int((x1 + x2) / 2), int((y1 + y2) / 2)


async def parse_ai_results(ai_results) -> tuple:
    # if DEBUG:
    #     print("Parsing AI results")
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

    # if DEBUG:
    #     print("Done parsing AI results.")
    return robot_results, golf_ball_results, golden_ball_results


async def update_robot_from_ai_result(track, robot_results, session: aiohttp.ClientSession):
    # if DEBUG:
    #     print("Updating robot from AI results")
    # Get current robot position and update track robot position
    if robot_results:
        robot_box = robot_results[0]
        current_pos = box_to_pos(robot_box)
        if DEBUG:
            print(
                f"Using robot with confidence {box_confidence(robot_box):.2f} at position ({current_pos[0]}, {current_pos[1]})")
        track.set_robot_pos(current_pos)
        async with session.post(f"{ROBOT_API_ENDPOINT}/position?x={current_pos[0]}&y={current_pos[1]}") as response:
            if DEBUG:
                print(response.status)
    else:
        if DEBUG:
            print("No robot on track!")


async def update_balls_from_ai_result(track, golf_ball_results, golden_ball_results):
    # if DEBUG:
    #     print("Update balls from AI result")
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


# TODO: rewrite this
def pick_target(track: Track) -> Optional[NodeData]:
    if not track.path or len(track.path) <= 1:
        return None

    # If small path, go straight for goal
    if len(track.path) <= 20:
        return track.path[-1]

    # If bigger path, get the path which is on 1/4 of the way
    return track.path[math.ceil(len(track.path)/4)]

    # If nothing else, get the first node on the path
    # return track.path[1]


async def calculate_and_adjust(track, session: aiohttp.ClientSession):
    # if DEBUG:
    #     print("Calculating path and adjusting speed")

    # Get the path to closest ball
    track.calculate_path()
    # track.draw(True)

    # Get the node to go to
    next_node = pick_target(track)
    if next_node:
        if DEBUG:
            print(f"Next node pos: ({next_node.node.x}, {next_node.node.y})")

        # Tell the robot to drive towards the node
        # drive_to_coordinates(next_node)
        await adjust_speed_using_pid(track, next_node.node, session)
    # if DEBUG:
    #     print("Done calculating path and adjusting speed")


# Inspired by https://en.wikipedia.org/wiki/PID_controller and
# https://www.sciencedirect.com/science/article/pii/S0967066101000661
async def adjust_speed_using_pid(track: Track, target_node: Node, session: aiohttp.ClientSession):
    # Get current robot position
    current_position = track.robot_pos

    if (current_position[0] == 0 and current_position[1] == 0) or not target_node:
        if DEBUG:
            print("Current position is (0,0), stopping speed adjustment")
        return

    # Calculate target heading to the next path point
    target_heading = target_node.get_heading(current_position)

    # Get the current heading
    current_heading = track.robot_direction

    # Calculate heading error
    error = target_heading - current_heading

    # Adjust error to be within -pi to pi range
    if error > math.pi:
        error -= 2 * math.pi
    elif error < -math.pi:
        error += 2 * math.pi

    # Reset integral and previous error if target position is very different
    global integral, previous_error, last_target_node
    if last_target_node and is_target_different(track, target_node, last_target_node):
        integral = 0
        previous_error = 0

    # Update integral term
    integral += error

    # Anti-windup - Limit the integral term
    integral = max(min(integral, 100), -100)

    # Update derivative term
    derivative = error - previous_error

    # Calculate PID output
    output = KP * error + KI * integral + KD * derivative

    if DEBUG:
        print(f"PID Output:\n"
              f"- Output: {output}\n"
              f"- Error: {error} (previous error {previous_error}) - Pi: {KP}\n"
              f"- Integral: {integral} - KI: {KI}\n"
              f"- Derivative: {derivative} - KD: {KD}")

    # Update previous error
    previous_error = error

    # Calculate wheel speeds based on PID output
    # speed_left = (2 * output * DIST_BETWEEN_WHEELS + error) / (2 * WHEEL_RADIUS)
    # speed_right = (2 * output * DIST_BETWEEN_WHEELS - error) / (2 * WHEEL_RADIUS)
    speed_left = max(min(-output + ROBOT_BASE_SPEED, 100), -100)
    speed_right = max(min(output + ROBOT_BASE_SPEED, 100), -100)

    if DEBUG:
        print(f"Speed: L:{speed_left} R:{speed_right}")

    # Set motor speeds
    async with session.post(
            f"{ROBOT_API_ENDPOINT}/drive?speed_left={speed_left}&speed_right={speed_right}") as response:
        if response.status != 200:
            print(f"Error on adjusting speed: {response.status}")

    # Update last target node
    last_target_node = target_node


def is_target_different(track: Track, target_node: Node, other_node: Node) -> bool:
    # Define a threshold for difference based on your requirements
    # Assume a difference of 1.0 cm is significant, we use pixels though, so it depends on the distance and camera
    position_threshold = 15.0

    # Calculate the position difference
    position_diff = math.sqrt((target_node.x - other_node.x)**2 + (target_node.y - other_node.y)**2)

    # Calculate the direction difference
    direction_diff = abs(target_node.get_heading(track.robot_pos) - other_node.get_heading(track.robot_pos))

    # Check if target is significantly different from the last target
    if position_diff > position_threshold or direction_diff > math.pi/4:
        return True

    return False


async def drive_to_coordinates(node: Node, session: aiohttp.ClientSession):
    async with session.post(f"{ROBOT_API_ENDPOINT}/drive?x={node.x}&y={node.y}") as response:
        if DEBUG:
            print(response.status)


async def do_race_iteration(track: Track, camera_queue: multiprocessing.JoinableQueue, session: aiohttp.ClientSession):
    try:
        # Get results from AI
        # if DEBUG:
        #     print("Trying to get results from queue")
        ai_results = camera_queue.get_nowait()
        # if DEBUG:
        #     print("Got results from AI!")

        # if DEBUG:
        #     # Get robot status
        #     get_robot_status()

        # Parse the results
        robot_results, golf_ball_results, golden_ball_results = await parse_ai_results(ai_results)

        # Update robot and balls
        await update_robot_from_ai_result(track, robot_results, session)
        await update_balls_from_ai_result(track, golf_ball_results, golden_ball_results)

        # Calculate track path and give the robot directions
        await calculate_and_adjust(track, session)

        # Let AI know we are done with the data
        # if DEBUG:
        #     print("Marked as done with event")
        camera_queue.task_done()
    except queue.Empty:
        pass
    except Exception as e:
        print("uh oh... - " + str(e))
        traceback.print_exc()


async def race(camera_queue: multiprocessing.JoinableQueue, track: Track, session: aiohttp.ClientSession) -> None:
    if DEBUG:
        print("Getting initial AI result.")
    while True:
        try:
            camera_queue.get_nowait()
            break
        except queue.Empty:
            await asyncio.sleep(0)
    if DEBUG:
        print("Got result, marking as ready.")
    camera_queue.task_done()
    if DEBUG:
        print("AI thread ready.")

    # input("Ready! Press Enter to start race!")
    print("Starting race!")
    start_time = time.time()
    time_taken = 0

    while time_taken <= 8 * 60:
        await do_race_iteration(track, camera_queue, session)
        time_taken = time.time() - start_time
        # Never remove this sleep
        await asyncio.sleep(0)
    print("Done with race!")


async def main(camera_queue: multiprocessing.JoinableQueue, track):
    print("Running main...")
    try:
        async with aiohttp.ClientSession() as session:
            if DEBUG:
                print("Starting robot...")
            while True:
                try:
                    # Start robot
                    async with session.post(f"{ROBOT_API_ENDPOINT}/start") as response:
                        if DEBUG:
                            print(response.status)
                            print(await response.text())
                        break
                except ConnectionError as e:
                    pass

            # Do the race
            await race(camera_queue, track, session)
    except KeyboardInterrupt:
        requests.post(f"{ROBOT_API_ENDPOINT}/stop")
        raise KeyboardInterrupt()
    except ConnectionError:
        print("Failed to connect to robot. Is the API on?")


def main_entrypoint(camera_queue: multiprocessing.JoinableQueue):
    print("Running main entrypoint")

    # Setup the track
    print("Setting up track...")
    track = setup_track()
    camera_queue.put("Done!")

    # Run main task in async
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    asyncio.run(main(camera_queue, track))


if __name__ == '__main__':
    try:
        print("Preparing the robot and AI... Please wait!")

        multiprocessing.set_start_method('spawn', force=True)

        # Initialize joinable queue to share data between processes
        # After many hours of troubleshooting, finally got help from https://stackoverflow.com/a/74190530/12418245
        queue = multiprocessing.JoinableQueue(maxsize=1)

        # Create a process for both the AI producer and the main consumer.
        # Run the AI as the "main thread" and the consumer in the background.
        ai_producer = multiprocessing.Process(target=run_ai, args=(queue,))
        main_consumer = multiprocessing.Process(target=main_entrypoint, args=(queue,), daemon=True)

        # First we start the consumer, since it needs to initialize the track
        main_consumer.start()

        # Wait for the track to be initialized before starting the AI
        if DEBUG:
            print("[main] Waiting for track to be initialized...")
        queue.get()
        queue.task_done()
        if DEBUG:
            print("[main] Track initialized! Starting AI producer...")

        # Now we start the AI
        # run_ai(queue)
        ai_producer.start()

        if DEBUG:
            print("[main] AI producer started!")

        # It shouldn't join, unless it ends somehow, but we put it here anyway
        ai_producer.join()

        if DEBUG:
            print("[main] AI producer joined!? Stopping race!")
    except KeyboardInterrupt:
        pass
    requests.post(f"{ROBOT_API_ENDPOINT}/stop")
