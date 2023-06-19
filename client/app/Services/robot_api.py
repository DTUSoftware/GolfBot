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

logger = logging.getLogger(__name__)
# logger.addHandler(logging.StreamHandler(sys.stdout))
if DEBUG:
    logger.setLevel(logging.DEBUG)


async def set_robot_start(session: aiohttp.ClientSession) -> bool:
    """
    Starts the robot.
    :param session: the session
    :return: True if the robot started, False otherwise
    """
    try:
        logger.debug("Setting robot start")
        async with session.post(f"{ROBOT_API_ENDPOINT}/start") as response:
            res = await response.text()
            if response.status != 200:
                logger.debug(f"Robot failed to start with response {res}, code {response.status}")
                return False
            logger.debug(f"Robot started with response: {res}")
        return True
    except Exception as e:
        logger.error(f"Failed to start robot with exception {e}")
        return False


def set_robot_stop() -> bool:
    """
    Stops the robot.
    :return: True if the robot stopped, False otherwise
    """
    try:
        response = requests.post(f"{ROBOT_API_ENDPOINT}/stop")
        res = response.text
        if response.status_code != 200:
            logger.debug(f"Robot failed to stop with response {res}, code {response.status_code}")
            return False
        logger.debug(f"Robot stopped with response: {res}")
        return True
    except Exception as e:
        logger.error(f"Failed to stop robot with exception {e}")
        return False


async def get_robot_status(session: aiohttp.ClientSession) -> bool:
    """
    Gets the robot status.
    :param session: the session
    :return: True if the robot is running, False otherwise
    """
    try:
        logger.debug("Getting robot status")
        async with session.get(f"{ROBOT_API_ENDPOINT}/status") as response:
            res = await response.text()
            if response.status != 200:
                logger.debug(f"Robot failed to get status with response {res}, code {response.status}")
                return False
            logger.debug(f"Robot status is: {res}")
        return True
    except Exception as e:
        logger.error(f"Failed to get robot status with exception {e}")
        return False


async def set_robot_position(session: aiohttp.ClientSession, x: int, y: int) -> bool:
    """
    Sets the robot position.
    :param session: the session
    :param x: The x coordinate
    :param y: The y coordinate
    :return: True if the robot position was set, False otherwise
    """
    try:
        logger.debug("Setting robot position")
        async with session.post(f"{ROBOT_API_ENDPOINT}/position?x={x}&y={y}") as response:
            res = await response.text()
            if response.status != 200:
                logger.debug(f"Robot failed to set position with response {res}, code {response.status}")
                return False
            logger.debug(f"Robot position set with response: {res}")
        return True
    except Exception as e:
        logger.error(f"Failed to set robot position with exception {e}")
        return False


async def set_robot_direction(session: aiohttp.ClientSession, direction: float) -> bool:
    """
    Sets the robot direction.
    NEEDS TO BE CALLED AFTER THE POSITION GETS SET!!!

    :param session: the session
    :param direction: The direction
    :return: True if the robot direction was set, False otherwise
    """
    try:
        logger.debug("Setting robot direction")
        async with session.post(f"{ROBOT_API_ENDPOINT}/direction?radians={(direction % (math.pi * 2))}") as response:
            res = await response.text()
            if response.status != 200:
                logger.debug(f"Robot failed to set direction with response {res}, code {response.status}")
                return False
            logger.debug(f"Robot direction set with response: {res}")
        return True
    except Exception as e:
        logger.error(f"Failed to set robot direction with exception {e}")
        return False


async def turn_robot(session: aiohttp.ClientSession, direction: float, relative=False) -> bool:
    """
    Turns the robot.
    THIS CALL IS BLOCKING UNTIL THE ROBOT HAS FULLY TURNED!
    :param session:  the session
    :param direction: the wanted direction
    :param relative: if the direction is relative to the current direction
    :return: None
    """
    try:
        async with session.post(f"{ROBOT_API_ENDPOINT}/turn?radians={direction}&relative={relative}") as response:
            res = await response.text()
            if response.status != 200:
                logger.debug(f"Robot failed to turn with response {res}, code {response.status}")
                return False
            logger.debug(f"Robot turned with response: {res}")
        return True
    except Exception as e:
        logger.error(f"Failed to turn robot with exception {e}")
        return False


async def set_speeds(session: aiohttp.ClientSession, speed_left: float, speed_right: float) -> bool:
    """
    Adjusts the speed of the robot.
    :param session: the session
    :param speed_left: the speed of the left wheel
    :param speed_right: the speed of the right wheel
    :return: True if the speed was adjusted, False otherwise
    """
    try:
        async with session.post(
                f"{ROBOT_API_ENDPOINT}/drive?speed_left={max(min(speed_left, 100), -100)}&speed_right={max(min(speed_right, 100), -100)}") as response:
            res = await response.text()
            if response.status != 200:
                logger.debug(f"Robot failed to set speed with response {res}, code {response.status}")
                return False
            logger.debug(f"Robot speed set with response: {res}")
        return True
    except Exception as e:
        logger.error(f"Failed to set robot speed with exception {e}")
        return False


async def toggle_fans(session: aiohttp.ClientSession) -> bool:
    """
    Toggles the fans.
    :param session: the session
    :return: True if the fans were toggled, False otherwise
    """
    try:
        async with session.post(
                f"{ROBOT_API_ENDPOINT}/toggle_fans") as response:
            res = await response.text()
            if response.status != 200:
                logger.debug(f"Robot failed to toggle fans with response {res}, code {response.status}")
                return False
            logger.debug(f"Robot fans toggled with response: {res}")
        return True
    except Exception as e:
        logger.error(f"Failed to toggle robot fans with exception {e}")
        return False
