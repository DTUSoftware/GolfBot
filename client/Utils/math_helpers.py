import math
from typing import Tuple

from Utils import path_algorithm


def calculate_direction(position1: Tuple[int, int], position2: Tuple[int, int]) -> float:
    """
    Calculates the heading between two points.

    Args:
        position1 (Tuple[int, int]): The first position.
        position2 (Tuple[int, int]): The second position.

    Returns:
        float: The heading in radians.
    """
    dx = position2[0] - position1[0]
    # We need the opposite of the y-axis, since we start from the top-left, and have a y-axis that goes downwards
    dy = position2[1] - position1[1]

    angle = math.atan2(dy, dx)
    if angle < 0:
        angle += 2 * math.pi

    return angle % (2 * math.pi)


def calculate_direction_difference(from_position: Tuple[int, int], middle_position: Tuple[int, int],
                                   to_position: Tuple[int, int]) -> float:
    """
    Calculates the direction difference between three positions.

    Args:
        from_position: The starting position.
        middle_position: The middle position.
        to_position: The ending position.

    Returns:
        float: The direction difference in radians.
    """
    direction1 = calculate_direction(from_position, middle_position)
    direction2 = calculate_direction(middle_position, to_position)
    direction_diff = abs(direction2 - direction1)
    # print(f"direction1: {direction1} (from {from_position} to {middle_position}\n"
    #       f"direction2: {direction2} (from {middle_position} to {to_position}\n"
    #       f"diff: {direction_diff}")
    return direction_diff


def calculate_distance(position1: Tuple[int, int], position2: Tuple[int, int]) -> float:
    """
    Calculates the distance between two points.

    Args:
        position1 (Tuple[int, int]): The first position.
        position2 (Tuple[int, int]): The second position.

    Returns:
        float: The distance between the two points.
    """
    return math.sqrt(pow(position1[0] - position2[0], 2) + pow(position1[1] - position2[1], 2))


def is_on_same_line(position1: Tuple[int, int], position2: Tuple[int, int]) -> bool:
    """
    Checks if two nodes are on the same line, either horizontally or vertically.

    Args:
        position1 (Tuple[int, int]): The first position.
        position2 (Tuple[int, int]): The second position.

    Returns:
        bool: True if the points are on the same line, False otherwise.
    """
    return position1[0] == position2[0] or position1[1] == position2[1]


def has_passed_target(frompos: Tuple[int, int], target: Tuple[int, int], currentpos: Tuple[int, int]) -> bool:
    """
    Checks if you pass a target.
    :param frompos: the original from position
    :param target: the target position
    :param currentpos: your new position
    :return: True if passed, else False
    """
    length_to_obtain = calculate_distance(target, frompos)
    current_length = calculate_distance(target, currentpos)
    position_threshold = 10
    if current_length - length_to_obtain >= position_threshold:
        return True
    return False


def is_about_to_collide_with_obstacle(pos: Tuple[int, int], direction: float):
    """
    Checks if you are about to collide with an obstacle.
    :param pos: your current position
    :param direction: your current direction
    :return: True if you are about to collide, else False
    """
    obstacles = path_algorithm.TRACK_GLOBAL.obstacles
    for obstacle in obstacles:
        if obstacle.is_about_to_collide(pos, direction):
            return True


if __name__ == "__main__":
    target_position = (487, 272)
    robot_pos = (487, 301)
    res = calculate_direction(position1=robot_pos, position2=target_position)
    print(res)
    print(math.degrees(res))
