#!/usr/bin/env python3
import asyncio
import os
import time
import traceback
from threading import Event

import aiohttp
from torch import multiprocessing

from Services import robot_api, robot_ai
from Utils import driving_algorithm, path_algorithm, track_setup

DISABLE_LOGGING = "true" in os.environ.get('DISABLE_LOGGING', "False").lower()
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


async def main(ai_queue: multiprocessing.JoinableQueue, path_queue: multiprocessing.JoinableQueue, ai_event: Event, track):
    # Update TRACK_GLOBAL in path algorithm since we just joined an async loop
    path_algorithm.TRACK_GLOBAL = track
    print("Running main...")
    try:
        async with aiohttp.ClientSession() as session:
            if DEBUG:
                print("Starting robot...")
            while True:
                try:
                    # Start robot
                    await robot_api.set_robot_start(session)
                    break
                except Exception as e:
                    print(f"Failed to start robot with exception {e}.\nRetrying in 5 seconds...")
                    await asyncio.sleep(5)

            # Do the race
            await race(ai_queue, path_queue, ai_event, track, session)
    except KeyboardInterrupt:
        robot_api.set_robot_stop()
        raise KeyboardInterrupt()
    except ConnectionError:
        print("Failed to connect to robot. Is the API on?")


def main_entrypoint(ai_queue: multiprocessing.JoinableQueue, path_queue: multiprocessing.JoinableQueue, ai_event: Event):
    print("Running main entrypoint")

    # Setup the track
    print("Setting up track...")
    track = track_setup.setup_track()
    path_queue.put("Done!")

    # Run main task in async
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    asyncio.run(main(ai_queue, path_queue, ai_event, track))


if __name__ == '__main__':
    try:
        print("Preparing the robot and AI... Please wait!")

        multiprocessing.set_start_method('spawn', force=True)

        # Initialize joinable queue to share data between processes
        # After many hours of troubleshooting, finally got help from https://stackoverflow.com/a/74190530/12418245
        ai_queue = multiprocessing.JoinableQueue(maxsize=1)
        path_queue = multiprocessing.JoinableQueue(maxsize=10)

        ai_queue_event = multiprocessing.Event()
        ai_queue_event.set()  # set it initially, we clear it when sending stuff through the queue

        # Create a process for both the AI producer and the main consumer.
        # Run the AI as the "main thread" and the consumer in the background.
        ai_producer = multiprocessing.Process(target=robot_ai.start_ai, args=(ai_queue, path_queue, ai_queue_event))
        main_consumer = multiprocessing.Process(target=main_entrypoint, args=(ai_queue, path_queue, ai_queue_event),
                                                daemon=True)

        # First we start the consumer, since it needs to initialize the track
        main_consumer.start()

        # Wait for the track to be initialized before starting the AI
        if DEBUG:
            print("[main] Waiting for track to be initialized...")
        path_queue.get()
        path_queue.task_done()
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
    robot_api.set_robot_stop()
