import asyncio
import logging
import os
import time
import traceback
from multiprocessing import Event
from typing import Tuple, List
import aiohttp
from torch import multiprocessing

from ..Services import robot_api, robot_ai
from . import path_algorithm, driving_algorithm

# If logging should be disabled
DISABLE_LOGGING = "true" in os.environ.get('DISABLE_LOGGING', "False").lower()
# If debugging should be enabled
DEBUG = ("true" in os.environ.get('DEBUG', "True").lower()) and not DISABLE_LOGGING

logger = logging.getLogger(__name__)
# logger.addHandler(logging.StreamHandler(sys.stdout))
if DEBUG:
    logger.setLevel(logging.DEBUG)

seen_ball_queue = []


async def calculate_and_adjust(track: path_algorithm.Track, path_queue: multiprocessing.JoinableQueue,
                               session: aiohttp.ClientSession,
                               objects_to_navigate_to: List[List[Tuple[int, int]]]) -> None:
    """
    Calculates the path and adjusts the speed
    Args:
        track: The track to use
        path_queue: the queue to put the path in
        session: the session to use
        objects_to_navigate_to: the objects to navigate to

    Returns:
        None
    """
    logger.debug("Calculating path and adjusting speed")

    # Get the path to closest ball
    start_time = 0.0
    if DEBUG:
        start_time = time.time()
    await track.calculate_path(objects_to_navigate_to)
    logger.debug(f"Got all paths in {time.time() - start_time} seconds!")
    # track.draw(True)

    if not track.path:
        logger.debug("No node to travel to, setting speed to 0!")
        await robot_api.set_speeds(session, 0, 0)

        # if not path_queue.full():
        #     try:
        #         path_queue.put([])
        #     except:
        #         pass
        return

    # Get the node to go to
    if await path_algorithm.check_new_path(path_queue, session) and track.last_target_path and \
            isinstance(track.last_target_path, list):
        # Tell the robot to drive towards the node
        # await drive_to_coordinates(next_node.node, session)
        await driving_algorithm.drive_decision(target_position=track.last_target_path[0].node.get_position(),
                                               session=session)
        # await driving_algorithm.adjust_speed_using_pid(track, track.last_target_path[0].node, session)
    else:
        logger.debug("No node")

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


async def do_race_iteration(track: path_algorithm.Track, ai_queue: multiprocessing.JoinableQueue,
                            path_queue: multiprocessing.JoinableQueue, ai_event: Event, session: aiohttp.ClientSession, time_taken):
    try:
        # Get results from AI
        # if DEBUG:
        #     print("Trying to get results from queue")
        if ai_queue.empty():
            # if DEBUG:
            #     print("AI queue empty")
            return

        ai_results = ai_queue.get_nowait()
        # if DEBUG:
        #     print("Got results from AI!")

        # if DEBUG:
        #     # Get robot status
        #     get_robot_status()

        # Parse the results
        robot_results, golf_ball_results, golden_ball_results = await robot_ai.parse_ai_results(ai_results)

        # Update robot and balls
        await robot_ai.update_robot_from_ai_result(track, robot_results, session)
        await robot_ai.update_balls_from_ai_result(track, golf_ball_results, golden_ball_results)

        # Update queue with results
        if len(seen_ball_queue) == 10:
            seen_ball_queue.pop(0)
        if track.balls:
            seen_ball_queue.append(True)
        else:
            seen_ball_queue.append(False)

        objects_to_navigate_to: List[List[Tuple[int, int]]] = []
        if track.balls and (
                len(seen_ball_queue) < 10 or len([seen_ball for seen_ball in seen_ball_queue if seen_ball]) >= 4) and time_taken <= 6 * 60:
            # Get every ball that's not golden
            objects_to_navigate_to = [ball.drivePath for ball in track.balls if
                                      not ball.golden and ball.get_drive_path()]
            # If no balls that aren't golden
            if not objects_to_navigate_to:
                logger.debug("Couldn't get a drive path for any white balls, trying the golden ball")
                # Include the golden ball
                objects_to_navigate_to = [ball.drivePath for ball in track.balls if ball.get_drive_path()]
        else:
            goal_path = await track.small_goal.deliver_path()
            if goal_path:
                objects_to_navigate_to = [goal_path]

        if objects_to_navigate_to:
            # Calculate track path and give the robot directions
            await calculate_and_adjust(track, path_queue, session, objects_to_navigate_to)
        else:
            logger.debug("No objects to navigate to")

        # Let AI know we are done with the data
        # if DEBUG:
        #     print("Marked as done with event")
        ai_queue.task_done()
        ai_event.set()
    except Exception as e:
        logging.error("uh oh... - " + str(e))
        traceback.print_exc()
    except:
        pass


async def race(ai_queue: multiprocessing.JoinableQueue, path_queue: multiprocessing.JoinableQueue, ai_event: Event,
               track: path_algorithm.Track, session: aiohttp.ClientSession) -> None:
    logger.debug("Getting initial AI result.")
    while True:
        if not ai_queue.empty():
            try:
                ai_queue.get_nowait()
                break
            except:
                pass
        await asyncio.sleep(0)
    logger.debug("Got result, marking as ready.")
    ai_queue.task_done()
    ai_event.set()
    logger.debug("AI thread ready.")

    # input("Ready! Press Enter to start race!")
    logging.info("Starting race!")
    start_time = time.time()
    time_taken = 0

    logging.info("Toggling fans!")
    await robot_api.toggle_fans(session)

    logging.info("Racing!")
    while time_taken <= 8 * 60:
        await do_race_iteration(track, ai_queue, path_queue, ai_event, session, time_taken)
        time_taken = time.time() - start_time
        # Never remove this sleep
        await asyncio.sleep(0)
    logging.info("Done with race!")
    robot_api.set_robot_stop()
