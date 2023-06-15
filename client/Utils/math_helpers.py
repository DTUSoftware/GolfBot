import math
from typing import Tuple, List

from Utils import path_algorithm


def calculate_direction(to_pos: Tuple[int, int], from_pos: Tuple[int, int]) -> float:
    """
    Calculates the heading between two points.

    Args:
        to_pos (Tuple[int, int]): The first position.
        from_pos (Tuple[int, int]): The second position.

    Returns:
        float: The heading in radians.
    """
    dx = from_pos[0] - to_pos[0]
    # We need the opposite of the y-axis, since we start from the top-left, and have a y-axis that goes downwards
    dy = from_pos[1] - to_pos[1]

    angle = math.atan2(dy, dx)
    if angle < 0:
        angle += 2 * math.pi

    return angle % (2 * math.pi)


def get_middle_between_two_points(position1: Tuple[int, int], position2: Tuple[int, int]) -> Tuple[int, int]:
    """
    Calculates the middle position between two points.
    Args:
        position1: the first position
        position2: the second position

    Returns:
        Tuple[int, int]: the middle position
    """
    return math.ceil((position1[0] + position2[0]) / 2), math.ceil((position1[1] + position2[1]) / 2)


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


def calculate_shortest_turn(start_angle: float, end_angle: float) -> float:
    """
    Calculates the shortest turn between two angles.
    :param start_angle:
    :param end_angle:
    :return the shortest turn between two angles:
    """
    absolute_difference = abs(end_angle - start_angle)

    if absolute_difference <= math.pi:
        shortest_turn = absolute_difference
    else:
        shortest_turn = 2 * math.pi - absolute_difference

    return shortest_turn


def calculate_longest_turn(start_angle: float, end_angle: float) -> float:
    """
    Calculates the longest turn between two angles.
    :param start_angle:
    :param end_angle:
    :return the longest turn between two angles:
    """
    absolute_difference = abs(end_angle - start_angle)

    if absolute_difference <= math.pi:
        longest_turn = 2 * math.pi - absolute_difference
    else:
        longest_turn = absolute_difference

    return longest_turn


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


def is_about_to_collide_with_obstacle(pos: Tuple[int, int], direction: float) -> bool:
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
    return False


def is_same_positions_as_obstacle(pos: Tuple[int, int]) -> bool:
    """
    Checks if you are on the same position as an obstacle.
    :param pos: your current position
    :return: True if you are on the same position, else False
    """
    obstacles = path_algorithm.TRACK_GLOBAL.obstacles
    for obstacle in obstacles:
        if obstacle.is_same_position(pos):
            return True
    return False


def calculate_xn_and_yn(radius: float, angle: float, pos: Tuple[int, int]) -> Tuple[int, int]:
    """
    Calculates the x and y position of a point on a circle.
    :param radius: the radius of the circle
    :param angle: the angle of the point on the circle
    :param pos: the center position of the circle
    :return: the x and y position of the point on the circle
    """
    xn = math.ceil(pos[0] + radius * math.cos(angle))
    yn = math.ceil(pos[1] + radius * math.sin(angle))
    return xn, yn


def calculate_angle_to_turn(radius: float, current_angle: float, angle_to_turn: float,
                            post: Tuple[int, int], ) -> float:
    """
    Calculates the angle to turn.
    :param radius:
    :param current_angle:
    :param angle_to_turn:
    :param post:
    :return the angle to turn returns 0 if the turn is not possible:
    """
    angle_array1: List[float]
    angle_array2: List[float]
    if current_angle < angle_to_turn:
        angle_array1 = [x for x in range(current_angle, angle_to_turn, math.pi / 18)]
        angle_array2 = [x for x in range(angle_to_turn, current_angle, -math.pi / 18)]
    else:
        angle_array1 = [x for x in range(current_angle, angle_to_turn, -math.pi / 18)]
        angle_array2 = [x for x in range(angle_to_turn, current_angle, math.pi / 18)]
    short_angle_array: List[float]
    long_angle_array: List[float]
    if len(angle_array1) > len(angle_array2):
        short_angle_array = angle_array2
        long_angle_array = angle_array1
    else:
        short_angle_array = angle_array1
        long_angle_array = angle_array2
    distance_array: List[int] = [x for x in range(1, radius, 1)]
    break_flag: bool = False
    for angle in short_angle_array:
        for distance in distance_array:
            xn, yn = calculate_xn_and_yn(distance, angle, post)
            if is_same_positions_as_obstacle((xn, yn)):
                break_flag = True
                break
        if break_flag:
            break
    else:
        # something here i haven't figured out yet
        return calculate_shortest_turn(current_angle, angle_to_turn)
    break_flag = False
    for angle in long_angle_array:
        for distance in distance_array:
            xn, yn = calculate_xn_and_yn(distance, angle, post)
            if is_same_positions_as_obstacle((xn, yn)):
                break_flag = True
                break
        if break_flag:
            break
    else:
        return calculate_longest_turn(current_angle, angle_to_turn)
    return 0


if __name__ == "__main__":
    target_position = (487, 272)
    robot_pos = (487, 301)
    res = calculate_direction(to_pos=robot_pos, from_pos=target_position)
    print(res)
    print(math.degrees(res))
