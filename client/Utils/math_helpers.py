import math
from typing import Tuple


def calculate_direction(position1: Tuple[int, int], position2: Tuple[int, int]) -> float:
    """
    Calculates the heading between two points.

    Args:
        position1 (Tuple[int, int]): The first position.
        position2 (Tuple[int, int]): The second position.

    Returns:
        float: The heading in radians.
    """
    dx = position2[1] - position1[1]
    dy = position2[0] - position1[0]
    return math.atan2(dy, dx)


def calculate_direction_difference(from_position: Tuple[int, int], middle_position: Tuple[int, int], to_position: Tuple[int, int]) -> float:
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
    return math.sqrt(pow(position1[0] + position2[0], 2) + pow(position1[1] + position2[1], 2))


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
