from Utils.math_helpers import calculate_direction, calculate_distance


def drive_decision(robot_position: tuple[int, int], robot_direction: float, target_position: tuple[int, int]):
    new_direction = calculate_direction(position1=robot_position, position2=target_position)
    distance = calculate_distance(position1=robot_position, position2=target_position)
    # maybe make som kind of tolerance
    if abs(robot_direction - new_direction) > 0:
        # turn robot-
        pass
    if distance > 0:
        # robot, drive distance
        pass
