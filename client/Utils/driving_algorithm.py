import logging
import sys
import aiohttp
import math
import os
from typing import Tuple

import asyncio

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
DISTANCE_TOLERANCE = float(os.environ.get('DISTANCE_TOLERANCE', 1.0))  # in units
DIRECTION_TOLERANCE = float(os.environ.get('DIRECTION_TOLERANCE', 45.0))  # degrees
DIRECTION_TOLERANCE_NEW = float(os.environ.get('DIRECTION_TOLERANCE_NEW', 15.0))  # degrees

# Robot parameters
WHEEL_RADIUS = (float(os.environ.get('WHEEL_DIAMETER', 68.8)) / 2) / 10  # Radius of the robot's wheels in cm
DIST_BETWEEN_WHEELS = float(
    os.environ.get('DIST_BETWEEN_WHEELS', 83.0 * 2)) / 10  # Distance between the robot's wheels in cm
ROBOT_BASE_SPEED = float(os.environ.get('ROBOT_BASE_SPEED', 25.0))

logger = logging.getLogger(__name__)
# logger.addHandler(logging.StreamHandler(sys.stdout))
if DEBUG:
    logger.setLevel(logging.DEBUG)

current_target_pos = (0, 0)


def do_smooth_turn(current_direction: float, new_direction: float, reverse=False) -> Tuple[float, float]:
    """
    Smoothly turns the robot to the new direction
    Args:
        current_direction: the current direction of the robot
        new_direction: the new direction of the robot
        reverse: if the robot should turn in reverse

    Returns:
        the new speed, first right then left
    """
    if reverse:
        robot_speed_right, robot_speed_left = (-ROBOT_BASE_SPEED, -ROBOT_BASE_SPEED)
    else:
        robot_speed_right, robot_speed_left = (ROBOT_BASE_SPEED, ROBOT_BASE_SPEED)

    current_direction = current_direction % (2 * math.pi)
    new_direction = new_direction % (2 * math.pi)

    # Adjust the robot speed depending on the direction difference, adding a small offset to the speed
    # to make sure the robot is always moving towards the correct angle

    # We make the direction diff (which can be at max math.pi * 2) into a value between 0 and 1
    # and multiply it by a multiplier to get a speed correction value between 0 and speed_correction_multiplier
    speed_correction_multiplier = 100

    # If the robot is to the left of the target, turn right by decreasing the right wheel speed
    # If the robot is to the right of the target, turn left by decreasing the left wheel speed

    right = True

    if current_direction < new_direction:
        if (new_direction - current_direction) > math.pi:
            diff_in_angle = abs((new_direction - 2 * math.pi) - current_direction)
            right = False
        else:
            diff_in_angle = abs(new_direction - current_direction)
            right = True
    else:
        if (current_direction - new_direction) > math.pi:
            diff_in_angle = abs((current_direction - 2 * math.pi) - new_direction)
            right = True
        else:
            diff_in_angle = abs(current_direction - new_direction)
            right = False

    speed_correction = (diff_in_angle / (math.pi * 2)) * speed_correction_multiplier
    if not reverse:
        speed_correction = -speed_correction

    if right:
        robot_speed_right = robot_speed_right + speed_correction
    else:
        robot_speed_left = robot_speed_right + speed_correction

    return robot_speed_right, robot_speed_left


async def drive_decision(target_position: Tuple[int, int], session: aiohttp.ClientSession) -> None:
    """
    The main decision-tree model for the driving of the robot.

    :param target_position: The target position of the robot.
    :param session: The AIOHTTP session used to send requests.
    :return: None
    """

    track = path_algorithm.TRACK_GLOBAL
    robot_direction = track.robot_direction

    robot_speed_left = ROBOT_BASE_SPEED
    robot_speed_right = ROBOT_BASE_SPEED

    logger.debug(f"Trying to get robot from {track.get_front_position()} to {target_position}. "
                 f"Robot currently has a direction of {math.degrees(robot_direction)} deg ({robot_direction} rad)")

    # If robot position is invalid, return
    if track.get_front_position() == (0, 0) or track.get_middle_position() == (0, 0) or target_position == (0, 0):
        # Stop robot
        await robot_api.set_speeds(session=session, speed_left=0, speed_right=0)
        return

    if path_algorithm.TRACK_GLOBAL.small_goal.is_in_delivery_distance():
        logger.debug("Robot is in delivery distance")

        if not path_algorithm.TRACK_GLOBAL.small_goal.is_in_delivery_direction():
            logger.debug("Robot is not in delivery direction, moving robot to delivery direction")
            await robot_api.turn_robot(session=session, direction=path_algorithm.TRACK_GLOBAL.small_goal.get_angle_to_middle())
            return

        logger.info("Robot is in delivery position, stopping robot and fans.")
        await robot_api.set_speeds(session=session, speed_left=0, speed_right=0)
        await robot_api.toggle_fans(session=session)
        logger.debug("Sleeping for 15 seconds to finish delivery")
        await asyncio.sleep(15)
        logger.debug("Done sleeping, driving backwards and starting fans again in case there are more balls")
        # Start fans again after driving backwards for 1 second
        await robot_api.set_speeds(session=session, speed_left=-ROBOT_BASE_SPEED, speed_right=-ROBOT_BASE_SPEED)
        await asyncio.sleep(1)
        await robot_api.set_speeds(session=session, speed_left=0, speed_right=0)
        await robot_api.toggle_fans(session=session)
        return

    # If the robot is about to drive into a wall or other obstacle, stop the robot
    if await math_helpers.is_about_to_collide_with_obstacle(track.get_front_position(), robot_direction):
        logger.debug("Robot is about to collide with an obstacle in front, driving robot backwards")
        await robot_api.set_speeds(session=session, speed_left=-ROBOT_BASE_SPEED, speed_right=-ROBOT_BASE_SPEED)
        # TODO: evaluate this sleep
        await asyncio.sleep(0.4)
        return

    # If the robot is about to drive into a wall or other obstacle, stop the robot
    if await math_helpers.is_about_to_collide_with_obstacle(track.get_middle_position(), (robot_direction + math.pi) % (2 * math.pi)):
        logger.debug("Robot is about to collide with an obstacle behind, driving robot forward")
        await robot_api.set_speeds(session=session, speed_left=ROBOT_BASE_SPEED, speed_right=ROBOT_BASE_SPEED)
        # TODO: evaluate this sleep
        await asyncio.sleep(0.4)
        return

    # Get the distance between the two targets
    # We use the turn position (rear) of the robot as the starting point,
    # as that is our turning point, so we turn at that point
    distance = math_helpers.calculate_distance(position1=track.get_front_position(), position2=target_position)

    logger.debug(f"The distance between the robot and the target is {distance} units")

    # If the distance is above distance tolerance
    if distance >= DISTANCE_TOLERANCE:
        # Get difference in direction, if any
        new_direction = math_helpers.calculate_direction(to_pos=target_position, from_pos=track.get_front_position())

        direction_diff = abs(robot_direction - new_direction)
        logger.debug(f"The angle from robot to target is {math.degrees(new_direction)} deg ({new_direction} rad). "
                     f"The adjustment in direction needed is {direction_diff} deg ({direction_diff} rad)")

        # Turn if needed
        global current_target_pos
        direction_tolerance = DIRECTION_TOLERANCE
        if target_position != current_target_pos:
            current_target_pos = target_position
        #     direction_tolerance = DIRECTION_TOLERANCE_NEW
        #
        #
        # if direction_diff >= math.radians(direction_tolerance):
        #     direction = new_direction
        #
        #     logger.debug(f"Turning robot {math.degrees(direction)} deg ({direction} rad) - Originally wanted to "
        #                  f"turn to {math.degrees(new_direction)} deg ({new_direction} rad), with diff being "
        #                  f"{math.degrees(direction_diff)} deg ({direction_diff} rad)")
        #     await robot_api.turn_robot(session=session, direction=direction)
        # else:
        #     logger.debug("Robot is in the correct heading (within tolerance), will not stop-turn.")

        # If we go backwards, go back
        if abs(direction_diff - math.radians(180)) < math.radians(45):
            robot_speed_right, robot_speed_left = do_smooth_turn((robot_direction + math.pi) % (2 * math.pi), new_direction, reverse=True)
        else:
            # Adjust the robot speed depending on the direction difference, adding a small offset to the speed
            # to make sure the robot is always moving towards the correct angle
            robot_speed_right, robot_speed_left = do_smooth_turn(robot_direction, new_direction)

        logger.debug("Driving robot forward.")
        # Drive forward with base speed
        await robot_api.set_speeds(session=session, speed_left=robot_speed_left, speed_right=robot_speed_right)
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
    current_position = track.get_middle_position()

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
    speed_left = -output + ROBOT_BASE_SPEED
    speed_right = output + ROBOT_BASE_SPEED

    logger.debug(f"Speed: L:{speed_left} R:{speed_right}")

    # Set motor speeds
    await robot_api.set_speeds(session, speed_left, speed_right)
