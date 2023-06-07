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
