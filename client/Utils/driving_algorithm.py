import aiohttp
import math
import os
from typing import Tuple

from client.Utils import math_helpers, path_algorithm
from client.Services import robot_api

DISABLE_LOGGING = "true" in os.environ.get('DISABLE_LOGGING', "False").lower()
DEBUG = ("true" in os.environ.get('DEBUG', "True").lower()) and not DISABLE_LOGGING

# PID constants
KP = float(os.environ.get('PID_KP', 5))  # Proportional gain  3.04
KI = float(os.environ.get('PID_KI', 0.1))  # Integral gain - 0.1
KD = float(os.environ.get('PID_KD', 0.05))  # Derivative gain - 0.05

# Robot tolerance
DISTANCE_TOLERANCE = float(os.environ.get('DISTANCE_TOLERANCE', 10.0))
DIRECTION_TOLERANCE = float(os.environ.get('DIRECTION_TOLERANCE', 5.0))  # degrees

# Robot parameters
WHEEL_RADIUS = (float(os.environ.get('WHEEL_DIAMETER', 68.8)) / 2) / 10  # Radius of the robot's wheels in cm
DIST_BETWEEN_WHEELS = float(
    os.environ.get('DIST_BETWEEN_WHEELS', 83.0 * 2)) / 10  # Distance between the robot's wheels in cm
ROBOT_BASE_SPEED = float(os.environ.get('ROBOT_BASE_SPEED', 50.0))

# PID variables
integral = 0
previous_error = 0


async def drive_decision(robot_position: Tuple[int, int], robot_direction: float, target_position: Tuple[int, int],
                         session: aiohttp.ClientSession) -> None:
    """
    The main decision-tree model for the driving of the robot.

    :param robot_position: The robot's current position.
    :param robot_direction: The robot's current direction.
    :param target_position: The target position of the robot.
    :return: None
    """

    # If robot position is invalid, return
    if robot_position == (0, 0):
        return

    # Get the distance between the two targets
    distance = math_helpers.calculate_distance(position1=robot_position, position2=target_position)

    # If the distance is above distance tolerance
    if distance >= DISTANCE_TOLERANCE:
        # Get difference in direction, if any
        new_direction = math_helpers.calculate_direction(position1=robot_position, position2=target_position)

        # Turn if needed
        if abs(robot_direction - new_direction) >= math.radians(DIRECTION_TOLERANCE):
            # turn robot-
            pass

        # Drive forward with base speed
        await robot_api.set_speeds(session, ROBOT_BASE_SPEED, ROBOT_BASE_SPEED)


async def adjust_speed_using_pid(track: path_algorithm.Track, target_node: path_algorithm.Node, session: aiohttp.ClientSession):
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
        if DEBUG:
            print("Current position is (0,0), stopping speed adjustment")
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
    global integral, previous_error, KI
    # if last_target_node and is_target_different(track, target_node, last_target_node):
    #     integral = 0
    #     previous_error = 0

    # Update integral term
    integral += error

    # Anti-windup - Limit the integral term
    integral = max(min(integral, 100), -100)

    # Update derivative term
    derivative = error - previous_error

    # DEBUG Testing of finding good variables
    KI += 0.0001
    print(f"=============================================================\n"
          f"USING A KI VALUE OF {KI}\n"
          f"=============================================================")

    # Calculate PID output
    output = KP * error + KI * integral + KD * derivative

    if DEBUG:
        print(f"PID Output:\n"
              f"- Output: {output}\n"
              f"- Error: {error} (previous error {previous_error}) - Pi: {KP}\n"
              f"- Integral: {integral} - KI: {KI}\n"
              f"- Derivative: {derivative} - KD: {KD}")

    # Update previous error
    previous_error = error

    # Calculate wheel speeds based on PID output
    # speed_left = (2 * output * DIST_BETWEEN_WHEELS + error) / (2 * WHEEL_RADIUS)
    # speed_right = (2 * output * DIST_BETWEEN_WHEELS - error) / (2 * WHEEL_RADIUS)
    speed_left = max(min(-output + ROBOT_BASE_SPEED, 100), -100)
    speed_right = max(min(output + ROBOT_BASE_SPEED, 100), -100)

    if DEBUG:
        print(f"Speed: L:{speed_left} R:{speed_right}")

    # Set motor speeds
    await robot_api.set_speeds(session, speed_left, speed_right)
