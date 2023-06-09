import asyncio
import os
import aiohttp
import requests

# The endpoint of the robot API
ROBOT_API_ENDPOINT = os.environ.get('API_ENDPOINT', "http://localhost:8069/api/v1")


async def set_robot_start(session: aiohttp.ClientSession) -> bool:
    """
    Starts the robot.
    :param session: the session
    :return: True if the robot started, False otherwise
    """
    print("Setting robot start")
    async with session.post(f"{ROBOT_API_ENDPOINT}/start") as response:
        print(await response.text())
        if response.status != 200:
            print(response.status)
            return False
    return True


def set_robot_stop() -> bool:
    """
    Stops the robot.
    :return: True if the robot stopped, False otherwise
    """
    response = requests.post(f"{ROBOT_API_ENDPOINT}/stop")
    print(response.text)
    if response.status_code != 200:
        print(response.status_code)
        return False
    return True


async def get_robot_status(session: aiohttp.ClientSession) -> bool:
    """
    Gets the robot status.
    :param session: the session
    :return: True if the robot is running, False otherwise
    """
    print("Getting robot status")
    async with session.get(f"{ROBOT_API_ENDPOINT}/status") as response:
        if response.status != 200:
            print(response.status)
            return False
        print(await response.text())
    return True


async def set_robot_position(session: aiohttp.ClientSession, x: int, y: int) -> bool:
    """
    Sets the robot position.
    :param session: the session
    :param x: The x coordinate
    :param y: The y coordinate
    :return: True if the robot position was set, False otherwise
    """
    print("Setting robot position")
    async with session.post(f"{ROBOT_API_ENDPOINT}/position?x={x}&y={y}") as response:
        print(await response.text())
        if response.status != 200:
            print(response.status)
            return False
        else:
            # Let the robot drive a lil' bit
            await asyncio.sleep(1)
    return True


async def set_robot_direction(session: aiohttp.ClientSession, direction: float) -> bool:
    """
    Turns the robot.
    THIS CALL IS BLOCKING UNTIL THE ROBOT HAS FULLY TURNED!
    :param session:  the session
    :param direction: the wanted direction
    :return: None
    """
    async with session.post(f"{ROBOT_API_ENDPOINT}/turn?radians={direction}") as response:
        print(await response.text())
        if response.status != 200:
            print(response.status)
            return False
        else:
            # Let the robot drive a lil' bit
            await asyncio.sleep(0.5)
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
            f"{ROBOT_API_ENDPOINT}/drive?speed_left={speed_left}&speed_right={speed_right}") as response:
        if response.status != 200:
            print(f"Error on adjusting speed: {response.status}")
            return False
        else:
            # Let the robot drive a lil' bit
            await asyncio.sleep(1)
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
            print(f"Error on toggling fans: {response.status}")
            return False
    return True
