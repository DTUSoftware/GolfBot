import asyncio
import math
import os
import aiohttp
import requests
import logging

# The endpoint of the robot API
ROBOT_API_ENDPOINT = os.environ.get('API_ENDPOINT', "http://localhost:8069/api/v1")

# If logging should be disabled
DISABLE_LOGGING = "true" in os.environ.get('DISABLE_LOGGING', "False").lower()
# If debugging should be enabled
DEBUG = ("true" in os.environ.get('DEBUG', "True").lower()) and not DISABLE_LOGGING
DRIVE_DELAY = 0
TURN_DELAY = 0

logger = logging.getLogger(__name__)
if DEBUG:
    logger.setLevel(logging.DEBUG)


async def set_robot_start(session: aiohttp.ClientSession) -> bool:
    """
    Starts the robot.
    :param session: the session
    :return: True if the robot started, False otherwise
    """
    logger.debug("Setting robot start")
    async with session.post(f"{ROBOT_API_ENDPOINT}/start") as response:
        logger.debug(await response.text())
        if response.status != 200:
            logger.debug(response.status)
            return False
    return True


def set_robot_stop() -> bool:
    """
    Stops the robot.
    :return: True if the robot stopped, False otherwise
    """
    response = requests.post(f"{ROBOT_API_ENDPOINT}/stop")
    logger.debug(response.text)
    if response.status_code != 200:
        logger.debug(response.status_code)
        return False
    return True


async def get_robot_status(session: aiohttp.ClientSession) -> bool:
    """
    Gets the robot status.
    :param session: the session
    :return: True if the robot is running, False otherwise
    """
    logger.debug("Getting robot status")
    async with session.get(f"{ROBOT_API_ENDPOINT}/status") as response:
        if response.status != 200:
            logger.debug(response.status)
            return False
        logger.debug(await response.text())
    return True


async def set_robot_position(session: aiohttp.ClientSession, x: int, y: int) -> bool:
    """
    Sets the robot position.
    :param session: the session
    :param x: The x coordinate
    :param y: The y coordinate
    :return: True if the robot position was set, False otherwise
    """
    logger.debug("Setting robot position")
    async with session.post(f"{ROBOT_API_ENDPOINT}/position?x={x}&y={y}") as response:
        logger.debug(await response.text())
        if response.status != 200:
            logger.debug(response.status)
            return False
    return True


async def set_robot_direction(session: aiohttp.ClientSession, direction: float) -> bool:
    """
    Sets the robot direction.
    NEEDS TO BE CALLED AFTER THE POSITION GETS SET!!!

    :param session: the session
    :param direction: The direction
    :return: True if the robot direction was set, False otherwise
    """
    logger.debug("Setting robot direction")
    async with session.post(f"{ROBOT_API_ENDPOINT}/direction?radians={(direction % (math.pi * 2))}") as response:
        logger.debug(await response.text())
        if response.status != 200:
            logger.debug(response.status)
            return False
    return True


async def turn_robot(session: aiohttp.ClientSession, direction: float) -> bool:
    """
    Turns the robot.
    THIS CALL IS BLOCKING UNTIL THE ROBOT HAS FULLY TURNED!
    :param session:  the session
    :param direction: the wanted direction
    :return: None
    """
    async with session.post(f"{ROBOT_API_ENDPOINT}/turn?radians={direction}") as response:
        logger.debug(await response.text())
        if response.status != 200:
            logger.debug(response.status)
            return False
        else:
            # Let the robot drive a lil' bit
            await asyncio.sleep(TURN_DELAY)
    return True


async def set_speeds(session: aiohttp.ClientSession, speed_left: float, speed_right: float) -> bool:
    """
    Adjusts the speed of the robot.
    :param session: the session
    :param speed_left: the speed of the left wheel
    :param speed_right: the speed of the right wheel
    :return: True if the speed was adjusted, False otherwise
    """
    async with session.post(
            f"{ROBOT_API_ENDPOINT}/drive?speed_left={max(min(speed_left, 100), -100)}&speed_right={max(min(speed_right, 100), -100)}") as response:
        if response.status != 200:
            logger.debug(f"Error on adjusting speed: {response.status}")
            return False
        else:
            # Let the robot drive a lil' bit
            await asyncio.sleep(DRIVE_DELAY)
    return True


async def toggle_fans(session: aiohttp.ClientSession) -> bool:
    """
    Toggles the fans.
    :param session: the session
    :return: True if the fans were toggled, False otherwise
    """
    async with session.post(
            f"{ROBOT_API_ENDPOINT}/toggle_fans") as response:
        if response.status != 200:
            logger.debug(f"Error on toggling fans: {response.status}")
            return False
    return True
