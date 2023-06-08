import os
from threading import Event

import aiohttp
from torch import multiprocessing

from client.Services import robot_api
from client.Utils import path_algorithm
from client.ai.main import run_ai

DISABLE_LOGGING = "true" in os.environ.get('DISABLE_LOGGING', "False").lower()
DEBUG = ("true" in os.environ.get('DEBUG', "True").lower()) and not DISABLE_LOGGING
GOLF_BALL_CONFIDENCE_GATE = float(os.environ.get('GOLF_BALL_CONFIDENCE_GATE', 0.45))


def box_confidence(box):
    return box.conf.item()


def box_to_pos(box) -> tuple:
    x1, y1, x2, y2 = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
    return int((x1 + x2) / 2), int((y1 + y2) / 2)


def start_ai(camera_queue: multiprocessing.JoinableQueue, path_queue: multiprocessing.JoinableQueue, ai_event: Event):
    run_ai(camera_queue, path_queue, ai_event)


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
        await robot_api.set_robot_position(session, x=current_pos[0], y=current_pos[1])
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
            ball = path_algorithm.Ball(box_to_pos(ball_box))
            track.add_ball(ball)
    # Add golden balls to track
    for ball_box in golden_ball_results:
        confidence = box_confidence(ball_box)
        if confidence > GOLF_BALL_CONFIDENCE_GATE:
            ball = path_algorithm.Ball(box_to_pos(ball_box), golden=True)
            track.add_ball(ball)
