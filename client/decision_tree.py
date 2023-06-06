import math


def drive_decision(robot_position: tuple[int, int], robot_direction: float, target_position: tuple[int, int]):
    new_direction = calculate_new_direction(robot_position=robot_position, target_position=target_position)
    distance = calculate_distance(robot_position=robot_position, target_position=target_position)
    # maybe make som kind of tolerance
    if abs(robot_direction - new_direction) > 0:
        # turn robot-
        pass
    if distance > 0:
        # robot, drive distance
        pass


def calculate_new_direction(robot_position: tuple[int, int], target_position: tuple[int, int]) -> float:
    return math.atan2(target_position[1] - robot_position[1], target_position[0] - robot_position[0])


def calculate_distance(robot_position: tuple[int, int], target_position: tuple[int, int]) -> float:
    return math.sqrt(pow(robot_position[0] + target_position[0], 2) + pow(robot_position[1] + target_position[1], 2))
