import logging

import aiohttp
import math
import os
from typing import Tuple

from Utils import math_helpers, path_algorithm
from Services import robot_api

# If logging should be disabled
DISABLE_LOGGING = "true" in os.environ.get('DISABLE_LOGGING', "False").lower()
# If debugging should be enabled
DEBUG = ("true" in os.environ.get('DEBUG', "True").lower()) and not DISABLE_LOGGING

# PID constants
KP = float(os.environ.get('PID_KP', 5))  # Proportional gain  3.04
KI = float(os.environ.get('PID_KI', 0.1))  # Integral gain - 0.1
KD = float(os.environ.get('PID_KD', 0.05))  # Derivative gain - 0.05

# Distance and direction tolerance
DISTANCE_TOLERANCE = float(os.environ.get('DISTANCE_TOLERANCE', 10.0))
DIRECTION_TOLERANCE = float(os.environ.get('DIRECTION_TOLERANCE', 5.0))  # degrees

# Robot parameters
WHEEL_RADIUS = (float(os.environ.get('WHEEL_DIAMETER', 68.8)) / 2) / 10  # Radius of the robot's wheels in cm
DIST_BETWEEN_WHEELS = float(
    os.environ.get('DIST_BETWEEN_WHEELS', 83.0 * 2)) / 10  # Distance between the robot's wheels in cm
ROBOT_BASE_SPEED = float(os.environ.get('ROBOT_BASE_SPEED', 40.0))

logger = logging.getLogger(__name__)
if DEBUG:
    logger.setLevel(logging.DEBUG)


async def drive_decision(robot_position: Tuple[int, int], robot_direction: float, target_position: Tuple[int, int],
                         session: aiohttp.ClientSession) -> None:
    """
    The main decision-tree model for the driving of the robot.

    :param robot_position: The robot's current position.
    :param robot_direction: The robot's current direction.
    :param target_position: The target position of the robot.
    :param session: The AIOHTTP session used to send requests.
    :return: None
    """

    logger.debug(f"Trying to get robot from {robot_position} to {target_position}. Robot currently has a direction of "
                  f"{math.degrees(robot_direction)} deg ({robot_direction} rad)")

    # If robot position is invalid, return
    if robot_position == (0, 0):
        # Stop robot
        await robot_api.set_speeds(session=session, speed_left=0, speed_right=0)
        return

    # If the robot is about to drive into a wall or other obstacle, stop the robot
    # if math_helpers.is_about_to_collide_with_obstacle(robot_position, robot_direction):
    #     if DEBUG:
    #         print("Robot is about to collide with an obstacle, driving robot backwards")
    #     await robot_api.set_speeds(session=session, speed_left=-ROBOT_BASE_SPEED, speed_right=-ROBOT_BASE_SPEED)
    #     return

    # Get the distance between the two targets
    distance = math_helpers.calculate_distance(position1=robot_position, position2=target_position)

    # If the distance is above distance tolerance
    if distance >= DISTANCE_TOLERANCE:
        # Get difference in direction, if any
        new_direction = math_helpers.calculate_direction(position1=target_position, position2=robot_position)

        logger.debug(f"The distance between the robot and the target is {distance} units, and the angle from robot to "
                      f"target is {math.degrees(new_direction)} deg ({new_direction} rad)\n"
                      f"The adjustment in direction needed is {math.degrees(new_direction - robot_direction)} deg "
                      f"({new_direction - robot_direction} rad)")

        # Turn if needed
        if abs(robot_direction - new_direction) >= math.radians(DIRECTION_TOLERANCE):
            logger.debug("Turning robot")
            await robot_api.set_robot_direction(session=session, direction=new_direction)
        else:
            logger.debug("Robot is in the correct heading (within tolerance), will not turn.")

        logger.debug("Driving robot forward.")
        # Drive forward with base speed
        await robot_api.set_speeds(session=session, speed_left=ROBOT_BASE_SPEED, speed_right=ROBOT_BASE_SPEED)
    else:
        logger.debug("Robot has reached the target (within tolerance), stopping robot.")
        # Stop robot
        await robot_api.set_speeds(session=session, speed_left=0, speed_right=0)

    # if DEBUG:
    #     logger.debug("Wait to read results")
    #     await asyncio.sleep(2)
    #     await robot_api.set_speeds(session=session, speed_left=0, speed_right=0)
    #     await asyncio.sleep(5)
    #     logger.debug("Wait done")


async def adjust_speed_using_pid(track: path_algorithm.Track, target_node: path_algorithm.Node,
                                 session: aiohttp.ClientSession):
    """
    An experimental PID controller for the robot.
    Inspired by https://en.wikipedia.org/wiki/PID_controller and
    https://www.sciencedirect.com/science/article/pii/S0967066101000661

    :param track: the track
    :param target_node: the target node
    :param session: the AIOHTTP session to send requests through
    :return: None
    """
    # Get current robot position
    current_position = track.robot_pos

    if (current_position[0] == 0 and current_position[1] == 0) or not target_node:
        logger.debug("Current position is (0,0), stopping speed adjustment")
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
    global KI
    # if last_target_node and is_target_different(track, target_node, last_target_node):
    #     integral = 0
    #     previous_error = 0

    # Update integral term
    track.integral += error

    # Anti-windup - Limit the integral term
    integral = max(min(track.integral, 100), -100)

    # Update derivative term
    derivative = error - track.previous_error

    # DEBUG Testing of finding good variables
    KI += 0.0001
    logger.debug(f"=============================================================\n"
                  f"USING A KI VALUE OF {KI}\n"
                  f"=============================================================")

    # Calculate PID output
    output = KP * error + KI * integral + KD * derivative

    logger.debug(f"PID Output:\n"
                  f"- Output: {output}\n"
                  f"- Error: {error} (previous error {track.previous_error}) - Pi: {KP}\n"
                  f"- Integral: {integral} - KI: {KI}\n"
                  f"- Derivative: {derivative} - KD: {KD}")

    # Update previous error
    track.previous_error = error

    # Calculate wheel speeds based on PID output
    # speed_left = (2 * output * DIST_BETWEEN_WHEELS + error) / (2 * WHEEL_RADIUS)
    # speed_right = (2 * output * DIST_BETWEEN_WHEELS - error) / (2 * WHEEL_RADIUS)
    speed_left = max(min(-output + ROBOT_BASE_SPEED, 100), -100)
    speed_right = max(min(output + ROBOT_BASE_SPEED, 100), -100)

    logger.debug(f"Speed: L:{speed_left} R:{speed_right}")

    # Set motor speeds
    await robot_api.set_speeds(session, speed_left, speed_right)
