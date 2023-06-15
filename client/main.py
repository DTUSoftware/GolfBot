#!/usr/bin/env python3
import asyncio
import os
import sys
from threading import Event
import aiohttp
import logging
from torch import multiprocessing

from Services import robot_api, robot_ai, robot_racing
from Utils import path_algorithm, track_setup

# If logging should be disabled
DISABLE_LOGGING = "true" in os.environ.get('DISABLE_LOGGING', "False").lower()
# If debugging should be enabled
DEBUG = ("true" in os.environ.get('DEBUG', "True").lower()) and not DISABLE_LOGGING

logger = logging.getLogger(__name__)
logger.addHandler(logging.StreamHandler(sys.stdout))
if DEBUG:
    logger.setLevel(logging.DEBUG)


async def main(ai_queue: multiprocessing.JoinableQueue, path_queue: multiprocessing.JoinableQueue, ai_event: Event,
               track):
    """
    The main function
    :param ai_queue:  The queue to get the AI results from AI to robot
    :param path_queue: The queue to send the path from robot to AI
    :param ai_event: The event to let the AI know that the robot has processed the results
    :param track: The track to run on
    :return: None
    """
    # Update TRACK_GLOBAL in path algorithm since we just joined an async loop
    path_algorithm.TRACK_GLOBAL = track
    logger.info("Running main...")
    try:
        async with aiohttp.ClientSession() as session:
            logger.debug("Starting robot...")
            while True:
                try:
                    # Start robot
                    await robot_api.set_robot_start(session)
                    break
                except Exception as e:
                    logging.error(f"Failed to start robot with exception {e}.\nRetrying in 5 seconds...")
                    await asyncio.sleep(5)

            # Do the race
            await robot_racing.race(ai_queue, path_queue, ai_event, track, session)
    except KeyboardInterrupt:
        robot_api.set_robot_stop()
        raise KeyboardInterrupt()
    except ConnectionError:
        logging.error("Failed to connect to robot. Is the API on?")


def main_entrypoint(ai_queue: multiprocessing.JoinableQueue, path_queue: multiprocessing.JoinableQueue,
                    ai_event: Event):
    """
    The main entrypoint
    :param ai_queue: The queue to get the AI results from AI to robot
    :param path_queue: The queue to send the path from robot to AI
    :param ai_event: The event to let the AI know that the robot has processed the results
    :return: None
    """
    logger.info("Running main entrypoint")

    # Setup the track
    logger.info("Setting up track...")
    track = track_setup.setup_track()
    path_queue.put("Done!")

    # Run main task in async
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    asyncio.run(main(ai_queue, path_queue, ai_event, track))


if __name__ == '__main__':
    ai_producer = None
    main_consumer = None
    try:
        logger.info("Preparing the robot and AI... Please wait!")

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
        logger.debug("Waiting for track to be initialized...")
        path_queue.get()
        path_queue.task_done()
        logger.debug("Track initialized! Starting AI producer...")

        # Now we start the AI
        # run_ai(queue)
        ai_producer.start()

        logger.debug("AI producer started!")

        # It shouldn't join, unless it ends somehow, but we put it here anyway
        ai_producer.join()

        logger.debug("AI producer joined!? Stopping race!")
    except KeyboardInterrupt:
        pass

    logger.debug("Killing processes.")
    if main_consumer:
        main_consumer.terminate()
    if ai_producer:
        ai_producer.terminate()

    logger.debug("Stopping robot.")
    robot_api.set_robot_stop()
