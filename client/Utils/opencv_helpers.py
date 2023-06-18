from typing import Tuple
import cv2

from ..core import path_algorithm


def draw_object(img, object_type, path):
    """
    Draws a path on an image.
    :param img: The image to draw on.
    :param object_type: The type of object to draw.
    :param path: The path to draw.
    :return: None
    """
    color = (0, 0, 0)
    if object_type == "obstacle":
        color = (0, 0, 255)
    elif object_type == "small_goal":
        color = (255, 0, 0)
    elif object_type == "big_goal":
        color = (0, 255, 0)

    for i in range(len(path)):
        point = path[i]

        # Add a dot at the point
        cv2.circle(img, point, 3, color)

        # Add a line
        if i >= 1:
            cv2.line(img, path[i - 1], point, color)


def get_frame_size() -> Tuple[int, int]:
    """
    Gets the size of the frame.
    :return: The size of the frame.
    """
    return len(path_algorithm.TRACK_GLOBAL.graph.nodes[0]), len(path_algorithm.TRACK_GLOBAL.graph.nodes)


def opencv_position_to_graph_position(position: Tuple[int, int]) -> Tuple[int, int]:
    """
    Converts a position from OpenCV's coordinate system to the graph's coordinate system.
    :param position: The position to convert.
    :return: The converted position.
    """
    frame_size = get_frame_size()
    pos = int(position[0]), int(frame_size[1] - position[1])
    # print(f"Converting {position} to graph position with frame size {frame_size}. Output: {pos}")
    return pos


def graph_position_to_opencv_position(position: Tuple[int, int]) -> Tuple[int, int]:
    """
    Converts a position from the graph's coordinate system to OpenCV's coordinate system.
    :param position: The position to convert.
    :return: The converted position.
    """
    frame_size = get_frame_size()
    # yes, it's the same conversion
    return int(position[0]), int(frame_size[1] - position[1])
