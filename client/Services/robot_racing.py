import asyncio
import os
import time
import traceback
from multiprocessing import Event
import aiohttp
from torch import multiprocessing

from client.Services import robot_api, robot_ai
from client.Utils import path_algorithm, driving_algorithm

# If logging should be disabled
DISABLE_LOGGING = "true" in os.environ.get('DISABLE_LOGGING', "False").lower()
# If debugging should be enabled
DEBUG = ("true" in os.environ.get('DEBUG', "True").lower()) and not DISABLE_LOGGING


async def calculate_and_adjust(track: path_algorithm.Track, path_queue: multiprocessing.JoinableQueue, session: aiohttp.ClientSession):
    if DEBUG:
        print("Calculating path and adjusting speed")

    # Get the path to closest ball
    start_time = 0.0
    if DEBUG:
        start_time = time.time()
    await track.calculate_path()
    if DEBUG:
        print(f"Got all paths in {time.time() - start_time} seconds!")
    # track.draw(True)

    if not track.path:
        if DEBUG:
            print("No node to travel to, setting speed to 0!")
        await robot_api.set_speeds(session, 0, 0)

        # if not path_queue.full():
        #     try:
        #         path_queue.put([])
        #     except:
        #         pass
        return

    # Get the node to go to
    if await path_algorithm.check_new_path(path_queue) and track.last_target_path and \
            isinstance(track.last_target_path, list):
        # Tell the robot to drive towards the node
        # await drive_to_coordinates(next_node.node, session)
        await driving_algorithm.drive_decision(robot_position=track.robot_pos, robot_direction=track.robot_direction,
                                               target_position=track.last_target_path[0].node.get_position(),
                                               session=session)
        # await driving_algorithm.adjust_speed_using_pid(track, track.last_target_path[0].node, session)
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


async def do_race_iteration(track: path_algorithm.Track, ai_queue: multiprocessing.JoinableQueue,
                            path_queue: multiprocessing.JoinableQueue, ai_event: Event, session: aiohttp.ClientSession):
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

        # Calculate track path and give the robot directions
        await calculate_and_adjust(track, path_queue, session)

        # Let AI know we are done with the data
        # if DEBUG:
        #     print("Marked as done with event")
        ai_queue.task_done()
        ai_event.set()
    except Exception as e:
        print("uh oh... - " + str(e))
        traceback.print_exc()
    except:
        pass


async def race(ai_queue: multiprocessing.JoinableQueue, path_queue: multiprocessing.JoinableQueue, ai_event: Event,
               track: path_algorithm.Track, session: aiohttp.ClientSession) -> None:
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
    ai_event.set()
    if DEBUG:
        print("AI thread ready.")

    # input("Ready! Press Enter to start race!")
    print("Starting race!")
    start_time = time.time()
    time_taken = 0

    print("Toggling fans!")
    await robot_api.toggle_fans(session)

    print("Racing!")
    while time_taken <= 8 * 60:
        await do_race_iteration(track, ai_queue, path_queue, ai_event, session)
        time_taken = time.time() - start_time
        # Never remove this sleep
        await asyncio.sleep(0)
    print("Done with race!")
