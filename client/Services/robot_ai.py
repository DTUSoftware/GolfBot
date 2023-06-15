import logging
import os
import sys
from threading import Event
from typing import Tuple, Dict

import aiohttp
from torch import multiprocessing

from Services import robot_api
from Utils import path_algorithm, math_helpers
import ai

# If logging should be disabled
DISABLE_LOGGING = "true" in os.environ.get('DISABLE_LOGGING', "False").lower()
# If debugging should be enabled
DEBUG = ("true" in os.environ.get('DEBUG', "True").lower()) and not DISABLE_LOGGING
# The confidence gate for the robot deciding to go for a golf ball
GOLF_BALL_CONFIDENCE_GATE = float(os.environ.get('GOLF_BALL_CONFIDENCE_GATE', 0.45))

logger = logging.getLogger(__name__)
# logger.addHandler(logging.StreamHandler(sys.stdout))
if DEBUG:
    logger.setLevel(logging.DEBUG)


def box_confidence(box) -> float:
    """
    Get confidence of box
    :param box: The box to get confidence from
    :return: The confidence of the box
    """
    return box.conf.item()


def box_to_pos(box) -> Tuple[int, int]:
    """
    Get the position of the box
    :param box: The box to get position from
    :return: The position of the box
    """
    x1, y1, x2, y2 = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
    return int((x1 + x2) / 2), int((y1 + y2) / 2)


def start_ai(camera_queue: multiprocessing.JoinableQueue, path_queue: multiprocessing.JoinableQueue, ai_event: Event):
    """
    Start the AI
    :param camera_queue: The queue to send the AI results from AI to robot
    :param path_queue: The queue to send the path from robot to AI
    :param ai_event: The event to let the AI know that the robot has processed the results
    :return: None
    """
    ai.run_ai(camera_queue, path_queue, ai_event)


async def parse_ai_results(ai_results) -> Tuple[Dict[str, list], list, list]:
    """
    Parse the AI results
    :param ai_results: The results to parse
    :return: The parsed results
    """
    # if DEBUG:
    #     print("Parsing AI results")
    robot_rear_results = []
    robot_front_results = []
    robot_results = []
    golf_ball_results = []
    golden_ball_results = []
    # https://docs.ultralytics.com/modes/predict/#working-with-results
    for result in ai_results:
        boxes = result.boxes
        for box in boxes:
            class_name = result.names[int(box.cls)]
            if all(x in class_name for x in ["robot", "front"]):
                robot_front_results.append(box)
            elif all(x in class_name for x in ["robot", "rear"]):
                robot_rear_results.append(box)
            elif "robot" in class_name:
                robot_results.append(box)
            elif "orange" in class_name:
                golden_ball_results.append(box)
            elif "white" in class_name:
                golf_ball_results.append(box)

    # Sort results by confidence
    robot_rear_results.sort(key=box_confidence)
    robot_front_results.sort(key=box_confidence)
    robot_results.sort(key=box_confidence)
    golf_ball_results.sort(key=box_confidence)
    golden_ball_results.sort(key=box_confidence)

    # if DEBUG:
    #     print("Done parsing AI results.")
    return {"robot": robot_results, "front": robot_front_results, "rear": robot_rear_results}, golf_ball_results, golden_ball_results


async def update_robot_from_ai_result(track: path_algorithm.Track, robot_results: Dict[str, list],
                                      session: aiohttp.ClientSession) -> None:
    """
    Update the robot from the AI results
    :param track: the track to update the robot on
    :param robot_results: the results to update the robot from
    :param session: the session to use for the robot api
    :return: None
    """
    # if DEBUG:
    #     print("Updating robot from AI results")
    # Get current robot position and update track robot position
    if robot_results:
        robot_pos = (0, 0)
        robot_direction = 0.0
        robot_front_pos = None
        robot_rear_pos = None
        # Extract values
        if robot_results["robot"]:
            robot_box = robot_results["robot"][0]
            robot_pos = box_to_pos(robot_box)
            # Calculate direction using past position and new position
            robot_direction = math_helpers.calculate_direction(to_pos=robot_pos, from_pos=track.robot_pos)
            # logger.debug(f"Using robot with confidence {box_confidence(robot_box):.2f} at position ({robot_pos[0]}, {robot_pos[1]})")
        if robot_results["front"]:
            robot_front_box = robot_results["front"][0]
            robot_front_pos = box_to_pos(robot_front_box)
        if robot_results["rear"]:
            robot_rear_box = robot_results["rear"][0]
            robot_rear_pos = box_to_pos(robot_rear_box)

        # Update robot position and direction
        if robot_front_pos and robot_rear_pos:
            robot_pos = math_helpers.get_middle_between_two_points(robot_front_pos, robot_rear_pos)
            robot_direction = math_helpers.calculate_direction(to_pos=robot_front_pos, from_pos=robot_rear_pos)
        elif robot_front_pos and robot_pos != (0, 0):
            robot_pos = math_helpers.get_middle_between_two_points(robot_front_pos, robot_pos)
            robot_direction = math_helpers.calculate_direction(to_pos=robot_front_pos, from_pos=robot_pos)
        elif robot_rear_pos and robot_pos != (0, 0):
            robot_pos = math_helpers.get_middle_between_two_points(robot_pos, robot_rear_pos)
            robot_direction = math_helpers.calculate_direction(to_pos=robot_pos, from_pos=robot_rear_pos)

        # Update values
        if robot_pos != (0, 0):
            track.set_robot_pos(middle=robot_pos, front=robot_front_pos, rear=robot_rear_pos)
            await robot_api.set_robot_position(session, x=robot_pos[0], y=robot_pos[1])
            track.set_robot_direction(robot_direction)
            await robot_api.set_robot_direction(session, direction=robot_direction)
        else:
            logger.debug("Couldn't set position and direction???")
    else:
        logger.debug("No robot on track!")


async def update_balls_from_ai_result(track: path_algorithm.Track, golf_ball_results, golden_ball_results) -> None:
    """
    Update the balls from the AI results
    :param track: the track to update the balls on
    :param golf_ball_results: the results to update the golf balls from
    :param golden_ball_results: the results to update the golden golf balls from
    :return: None
    """
    # if DEBUG:
    #     print("Update balls from AI result")
    # Add balls to track
    track.clear_balls()
    for ball_box in golf_ball_results:
        confidence = box_confidence(ball_box)
        if confidence > GOLF_BALL_CONFIDENCE_GATE:
            ball = path_algorithm.Ball(box_to_pos(ball_box))
            track.add_ball(ball)
    # Add golden balls to track
    for ball_box in golden_ball_results:
        confidence = box_confidence(ball_box)
        if confidence > GOLF_BALL_CONFIDENCE_GATE:
            ball = path_algorithm.Ball(box_to_pos(ball_box), golden=True)
            track.add_ball(ball)
