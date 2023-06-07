#!/usr/bin/env python3
import asyncio
import math
import os
import random
import time
import traceback
from typing import Optional

import aiohttp
import requests
from torch import multiprocessing

from Utils.math_helpers import calculate_distance
from ai.main import run_ai
from drive_algorithm import Ball, Track, Node, NodeData
from track_setup import setup_track

ROBOT_API_ENDPOINT = os.environ.get('API_ENDPOINT', "http://localhost:8069/api/v1")
GOLF_BALL_CONFIDENCE_GATE = float(os.environ.get('GOLF_BALL_CONFIDENCE_GATE', 0.45))
DISABLE_LOGGING = "true" in os.environ.get('DISABLE_LOGGING', "False").lower()
DEBUG = ("true" in os.environ.get('DEBUG', "True").lower()) and not DISABLE_LOGGING

# PID constants
KP = float(os.environ.get('PID_KP', 5))  # Proportional gain  3.04
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
last_target_path = None


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
            elif "orange" in class_name:
                golden_ball_results.append(box)
            elif "white" in class_name:
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
            if response.status != 200:
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
    return track.path[math.ceil(len(track.path) / 4)]

    # If nothing else, get the first node on the path
    # return track.path[1]


async def calculate_and_adjust(track, path_queue: multiprocessing.JoinableQueue, session: aiohttp.ClientSession):
    if DEBUG:
        print("Calculating path and adjusting speed")

    # Get the path to closest ball
    await track.calculate_path()
    # track.draw(True)

    if not track.path:
        if DEBUG:
            print("No node to travel to, setting speed to 0!")
        await set_speeds(session, 0, 0)

        # if not path_queue.full():
        #     try:
        #         path_queue.put([])
        #     except:
        #         pass
        return

    # Get the node to go to
    if collapse_path(track, path_queue) and last_target_path and isinstance(last_target_path, list):
        # Tell the robot to drive towards the node
        # await drive_to_coordinates(next_node.node, session)
        await adjust_speed_using_pid(track, last_target_path[0].node, session)
    else:
        print("No node")

    # next_node = pick_target(track)
    # if next_node:
    #     if DEBUG:
    #         print(f"Next node pos: ({next_node.node.x}, {next_node.node.y})")
    #
    #     # Tell the robot to drive towards the node
    #     # await drive_to_coordinates(next_node.node, session)
    #     await adjust_speed_using_pid(track, next_node.node, session)
    # else:
    #     if DEBUG:
    #         print("No next node?")

    # if DEBUG:
    #     print("Done calculating path and adjusting speed")


async def set_speeds(session, speed_left, speed_right):
    async with session.post(
            f"{ROBOT_API_ENDPOINT}/drive?speed_left={speed_left}&speed_right={speed_right}") as response:
        if response.status != 200:
            print(f"Error on adjusting speed: {response.status}")


def collapse_path(track: Track, path_queue: multiprocessing.JoinableQueue) -> bool:
    global last_target_path, integral, previous_error, last_target_node
    if DEBUG:
        print(
            f"current last target path: {[(nodedata.node.x, nodedata.node.y) for nodedata in last_target_path] if last_target_path else []}\n"
            f"new path target: {(track.path[-1].node.x, track.path[-1].node.y) if track.path else []}")
    # If not set already, set and reset
    # Check if the new path target is different than before
    if last_target_path and isinstance(last_target_path, list) and not is_target_different(track,
                                                                                           last_target_path[-1].node,
                                                                                           track.path[-1].node):
        if DEBUG:
            print("No change in target, keeping current path.")

            # If robot is at target, pop
            if not is_target_different(track, last_target_path[0].node, track.graph.get_node(track.robot_pos)):
                if DEBUG:
                    print("Robot reached target, popping")
                last_target_path.pop(0)
                last_target_node = track.graph.get_node(track.robot_pos)
            # If we passed the target, pop
            has_passed_result = has_passed_target(last_target_path[0].node, track.graph.get_node(track.robot_pos),
                                                  last_target_node)
            if has_passed_result:
                if DEBUG:
                    print("Robot passed target, popping")
                last_target_path.pop(0)
                last_target_node = track.graph.get_node(track.robot_pos)
            elif has_passed_result is None:
                if DEBUG:
                    print("Robot went completely wrong way?!?? Removing path!")
                last_target_node = None
                last_target_path = None
                # if not path_queue.full():
                #     try:
                #         path_queue.put([])
                #     except:
                #         pass

            if DEBUG:
                print(
                    f"Current optimized path: {[(nodedata.node.x, nodedata.node.y) for nodedata in last_target_path]}")
            if last_target_path:
                # Call adjust to adjust for error, and to clear point from list if we reach the target
                return True
            else:
                # if not path_queue.full():
                #     try:
                #         path_queue.put([])
                #     except:
                #         pass
                pass
    if DEBUG:
        print("New target, making new path")

    if track.path[0].node.x == 0 and track.path[0].node.y == 0:
        if DEBUG:
            print("Robot pos is 0,0, not making new path")
            return False

    integral = 0
    previous_error = 0

    # "Summarize" path into good points for targets
    new_path = []
    current_from_node = track.path[0]

    if len(track.path) <= 1:
        # If under 1, do not make any path
        if DEBUG:
            print("Under or equal to 1 (only robot), skips")
        new_path = []
    elif len(track.path) == 2:
        # Only given two points, make last point the target
        if DEBUG:
            print("Only two points, setting to last element")
        new_path.append(track.path[-1])
    else:
        # Get the summarization
        for i in range(2, len(track.path)):
            current_check_node = track.path[i]
            if current_check_node == current_from_node:
                if DEBUG:
                    print("Same as current from node, skipping")
                continue

            # If last node, add it
            if i == (len(track.path) - 1):
                if DEBUG:
                    print("Last node appending")
                new_path.append(current_check_node)
                break

            # Check if node is on same straight line
            if current_check_node.node.x == current_from_node.node.x:
                # if DEBUG:
                #     print("Same x")
                continue
            if current_check_node.node.y == current_from_node.node.y:
                # if DEBUG:
                #     print("Same y")
                continue

            # If heading is same as last node, skip
            heading_diff_tolerance = 15  # degrees, not radians
            current_from_pos = (current_from_node.node.x, current_from_node.node.y)
            heading_of_check_node = current_check_node.node.get_heading(
                (track.path[i - 1].node.x, track.path[i - 1].node.y))
            # The line below 'should not' fuck up, but it can give a value of 0 if, somehow, the track path's last node
            # is the same as the current from pos. But we, in the case of a new direction, assign the last variable,
            # so the current from pos should always be different than the last node!
            # We also start at index 2 (third value) to get around this, and for other reasons.
            heading_of_last_node = track.path[i - 1].node.get_heading(current_from_pos)
            heading_diff = abs(heading_of_check_node - heading_of_last_node)
            # if DEBUG:
            #     print(f"current from: {current_from_pos}\n"
            #           f"current check pos: {current_check_node.node.x}, {current_check_node.node.y}\n"
            #           f"last check pos: {track.path[i-1].node.x}, {track.path[i-1].node.y}\n"
            #           f"Heading diff: (check) {heading_of_check_node} - (last) {heading_of_last_node} = {heading_diff}")
            if heading_diff <= math.radians(heading_diff_tolerance):
                # if DEBUG:
                #     print("Is same direction, skipping")
                continue

            # We have a new direction, add the LAST POINT (not current!) to the new path, as it's where the turn happens
            if DEBUG:
                print(f"{track.path[i - 1].node.x}, {track.path[i - 1].node.y} is on not on same path, appending")
            new_path.append(track.path[i - 1])
            current_from_node = track.path[i - 1]

    last_target_path = new_path
    if DEBUG:
        print(f"Current target optimized path: {[(nodedata.node.x, nodedata.node.y) for nodedata in last_target_path]}")

    # Adjust and go to
    if last_target_path:
        full_path = [track.robot_pos] + [(nodedata.node.x, nodedata.node.y) for nodedata in last_target_path]

        if not path_queue.full():
            try:
                path_queue.put(full_path)
            except:
                pass
        last_target_node = track.graph.get_node(track.robot_pos)
        return True


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
    global integral, previous_error, KI
    # if last_target_node and is_target_different(track, target_node, last_target_node):
    #     integral = 0
    #     previous_error = 0

    # Update integral term
    integral += error

    # Anti-windup - Limit the integral term
    integral = max(min(integral, 100), -100)

    # Update derivative term
    derivative = error - previous_error

    # DEBUG Testing of finding good variables
    KI += 0.0001
    print(f"=============================================================\n"
          f"USING A KI VALUE OF {KI}\n"
          f"=============================================================")

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
    await set_speeds(session, speed_left, speed_right)

    # Update last target node
    # last_target_node = target_node


def is_target_different(track: Track, target_node: Node, other_node: Node) -> bool:
    # Define a threshold for difference based on your requirements
    # Assume a difference of 1.0 cm is significant, we use pixels though, so it depends on the distance and camera
    position_threshold = 25.0

    # Calculate the position difference
    position_diff = calculate_distance(target_node.get_position(), other_node.get_position())
    # Calculate the direction difference
    direction_diff = abs(target_node.get_heading(track.robot_pos) - other_node.get_heading(track.robot_pos))

    # Check if target is significantly different from the last target
    if position_diff > position_threshold or direction_diff > math.pi / 4:
        return True

    return False


def has_passed_target(target_node: Node, current_node: Node, from_node: Node):
    length_to_obtain = calculate_distance(target_node.get_position(), from_node.get_position())
    current_length = calculate_distance(target_node.get_position(), current_node.get_position())
    position_threshold = 10
    if current_length - length_to_obtain >= position_threshold:
        return True
    return False


async def drive_to_coordinates(node: Node, session: aiohttp.ClientSession):
    async with session.post(f"{ROBOT_API_ENDPOINT}/drive?x={node.x}&y={node.y}") as response:
        if response.status != 200:
            print(response.status)
        else:
            # Let the robot drive a lil' bit
            await asyncio.sleep(1)


async def do_race_iteration(track: Track, ai_queue: multiprocessing.JoinableQueue,
                            path_queue: multiprocessing.JoinableQueue, session: aiohttp.ClientSession):
    try:
        # Get results from AI
        # if DEBUG:
        #     print("Trying to get results from queue")
        if ai_queue.empty():
            # if DEBUG:
            #     print("AI queue empty")
            return

        ai_results = ai_queue.get_nowait()
        if DEBUG:
            print("Got results from AI!")

        # if DEBUG:
        #     # Get robot status
        #     get_robot_status()

        # Parse the results
        robot_results, golf_ball_results, golden_ball_results = await parse_ai_results(ai_results)

        # Update robot and balls
        await update_robot_from_ai_result(track, robot_results, session)
        await update_balls_from_ai_result(track, golf_ball_results, golden_ball_results)

        # Calculate track path and give the robot directions
        await calculate_and_adjust(track, path_queue, session)

        # Let AI know we are done with the data
        # if DEBUG:
        #     print("Marked as done with event")
        ai_queue.task_done()
    except Exception as e:
        print("uh oh... - " + str(e))
        traceback.print_exc()
    except:
        pass


async def race(ai_queue: multiprocessing.JoinableQueue, path_queue: multiprocessing.JoinableQueue, track: Track,
               session: aiohttp.ClientSession) -> None:
    if DEBUG:
        print("Getting initial AI result.")
    while True:
        if not ai_queue.empty():
            try:
                ai_queue.get_nowait()
                break
            except:
                pass
        await asyncio.sleep(0)
    if DEBUG:
        print("Got result, marking as ready.")
    ai_queue.task_done()
    if DEBUG:
        print("AI thread ready.")

    # input("Ready! Press Enter to start race!")
    print("Starting race!")
    start_time = time.time()
    time_taken = 0

    print("Toggling fans!")
    await toggle_fans(session)

    print("Racing!")
    while time_taken <= 8 * 60:
        await do_race_iteration(track, ai_queue, path_queue, session)
        time_taken = time.time() - start_time
        # Never remove this sleep
        await asyncio.sleep(0)
    print("Done with race!")


async def toggle_fans(session):
    async with session.post(
            f"{ROBOT_API_ENDPOINT}/toggle_fans") as response:
        if response.status != 200:
            print(f"Error on toggling fans: {response.status}")


async def main(ai_queue: multiprocessing.JoinableQueue, path_queue: multiprocessing.JoinableQueue, track):
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
            await race(ai_queue, path_queue, track, session)
    except KeyboardInterrupt:
        requests.post(f"{ROBOT_API_ENDPOINT}/stop")
        raise KeyboardInterrupt()
    except ConnectionError:
        print("Failed to connect to robot. Is the API on?")


def main_entrypoint(ai_queue: multiprocessing.JoinableQueue, path_queue: multiprocessing.JoinableQueue):
    print("Running main entrypoint")

    # Setup the track
    print("Setting up track...")
    track = setup_track()
    ai_queue.put("Done!")

    # Run main task in async
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    asyncio.run(main(ai_queue, path_queue, track))


if __name__ == '__main__':
    try:
        print("Preparing the robot and AI... Please wait!")

        multiprocessing.set_start_method('spawn', force=True)

        # Initialize joinable queue to share data between processes
        # After many hours of troubleshooting, finally got help from https://stackoverflow.com/a/74190530/12418245
        ai_queue = multiprocessing.JoinableQueue(maxsize=1)
        path_queue = multiprocessing.JoinableQueue(maxsize=10)

        # Create a process for both the AI producer and the main consumer.
        # Run the AI as the "main thread" and the consumer in the background.
        ai_producer = multiprocessing.Process(target=run_ai, args=(ai_queue, path_queue))
        main_consumer = multiprocessing.Process(target=main_entrypoint, args=(ai_queue, path_queue), daemon=True)

        # First we start the consumer, since it needs to initialize the track
        main_consumer.start()

        # Wait for the track to be initialized before starting the AI
        if DEBUG:
            print("[main] Waiting for track to be initialized...")
        ai_queue.get()
        ai_queue.task_done()
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
