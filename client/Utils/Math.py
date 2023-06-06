import math


def calculate_new_direction(old_position: tuple[int, int], new_position: tuple[int, int]) -> float:
    return math.atan2(new_position[1] - old_position[1], new_position[0] - old_position[0])


def calculate_distance(old_position: tuple[int, int], new_position: tuple[int, int]) -> float:
    return math.sqrt(pow(old_position[0] + new_position[0], 2) + pow(old_position[1] + new_position[1], 2))