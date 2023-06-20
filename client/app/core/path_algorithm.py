import asyncio
import heapq
import logging
import math
import os
from typing import Any, Optional, List, Tuple, Set, Dict, Union

import aiohttp
import matplotlib.pyplot as plt
import numpy as np
from colorama import Fore
from colorama import Style
from colorama import init as colorama_init
from torch import multiprocessing

from ..Services import robot_api
from ..Utils import math_helpers, opencv_helpers

# If debugging should be enabled
DEBUG = "true" in os.environ.get('DEBUG', "True").lower()
# The timeout for calculating a path
TIMEOUT_GET_PATH = 5  # in seconds
# in units, this is where the balls should max be from the borders for robot width
PATH_OBSTACLE_DISTANCE = 100
DELIVERY_DISTANCE_FAR = 150  # in units
# in units, this is where the middle of the robot is when delivering (241 - 113)
DELIVERY_DISTANCE = 110
DELIVERY_DISTANCE_X_DIFF = 15  # in units
DELIVERY_DISTANCE_Y = 30  # in units
SAFETY_LENGTH = 150  # in units
SAFETY_LENGTH_CORNER = 200  # in units
HEADING_DIFFERENCE_DELIVERY = 1  # in degrees
COLLISION_DISTANCE = 30  # in units (pixels)
DIRECTION_DIFFERENCE = 250  # in degrees
TARGET_DIFFERENT_POSITION_DIFF_THRESHOLD = 20.0

# Initialize colorama
colorama_init()
# Calculate the distance across a square
distance_across = math.sqrt(1 ** 2 + 1 ** 2)

logger = logging.getLogger(__name__)
# logger.addHandler(logging.StreamHandler(sys.stdout))
if DEBUG:
    logger.setLevel(logging.DEBUG)


class Node:
    def __init__(self, coordinates: tuple) -> None:
        """
        Initializes a new instance of the Node class.

        Args:
            coordinates (tuple): The coordinates of the node as a tuple of (x, y).

        Returns:
            None
        """
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.neighbours: List[Dict[str, Union['Node', float]]] = []

    def __lt__(self, other: Any) -> bool:
        """
        Less than comparison operator.

        Args:
            other (Any): The other object to compare.

        Returns:
            bool: Always returns True.
        """
        return True

    def __le__(self, other: Any) -> bool:
        """
        Less than or equal to comparison operator.

        Args:
            other (Any): The other object to compare.

        Returns:
            bool: Always returns True.
        """
        return True

    def get_position(self) -> Tuple[int, int]:
        """
        Returns the position of the node as a tuple of (x, y) coordinates.

        Returns:
            Tuple[int, int]: The position of the node.
        """
        return self.x, self.y

    def add_neighbour(self, node: 'Node', weight: float = None) -> None:
        """
        Adds a neighbour node with an optional weight to the current node.

        Args:
            node (Node): The neighbour node to add.
            weight (float, optional): The weight of the edge connecting the nodes. If not provided,
                the weight is calculated based on the Euclidean distance between the nodes.

        Returns:
            None
        """
        if not weight:
            if self.x == node.x or self.y == node.y:
                weight = max(abs(self.x - node.x), abs(self.y - node.y))
            else:
                weight = math_helpers.calculate_distance(
                    self.get_position(), node.get_position())
        self.neighbours.append({"node": node, "weight": weight})

    def remove_neighbour(self, node: 'Node') -> None:
        """
        Removes a neighbour node from the current node.

        Args:
            node (Node): The neighbour node to remove.

        Returns:
            None
        """
        for i in range(len(self.neighbours)):
            if self.neighbours[i]["node"] == node:
                self.neighbours.pop(i)
                return

    def get_neighbour_nodes(self) -> List['Node']:
        """
        Returns a list of neighbour nodes.

        Returns:
            List[Node]: A list of neighbour nodes.
        """
        return [neighbour["node"] for neighbour in self.neighbours]

    def get_heading(self, from_position: tuple) -> float:
        """
        Returns the heading from a given position to the current node.

        Args:
            from_position (tuple): The starting position as a tuple of (x, y) coordinates.

        Returns:
            float: The heading in radians.
        """
        return math_helpers.calculate_direction(from_pos=from_position, to_pos=self.get_position())


class Ball:
    def __init__(self, pos: Tuple[int, int], golden: bool = False) -> None:
        """
        Initializes a new instance of the Ball class.

        Args:
            pos (tuple): The position of the ball as a tuple of (x, y) coordinates.
            golden (bool, optional): Indicates whether the ball is golden. Defaults to False.

        Returns:
            None
        """
        self.x = pos[0]
        self.y = pos[1]
        self.golden = golden
        self.drivePath: List[Tuple[int, int]] = []

    def get_position(self) -> Tuple[int, int]:
        """
        Returns the position of the ball as a tuple of (x, y) coordinates.

        Returns:
            Tuple[int, int]: The position of the ball.
        """
        return self.x, self.y

    def get_drive_path(self) -> List[Tuple[int, int]]:
        """
        Returns the drive path of the ball as a list of (x, y) coordinates.
        Returns:
            List[Tuple[int, int]]: The drive path of the ball.
        """
        # obstacle_angle_array = []
        x_node: Tuple[Tuple[int, int], float, float] = ((0, 0), math.inf, 0)
        y_node: Tuple[Tuple[int, int], float, float] = ((0, 0), math.inf, 0)
        self_pos = self.get_position()
        # for each obstacle in the track we want to find the closest node to the ball
        for obstacle in TRACK_GLOBAL.obstacles:
            for node in obstacle.path:
                node_pos = node.get_position()
                if (node_pos[0] != self_pos[0] and node_pos[1] != self_pos[1]) or node_pos == self_pos:
                    continue

                # Get the distance and direction to the node
                distance = math_helpers.calculate_distance(self_pos, node_pos)
                if distance < PATH_OBSTACLE_DISTANCE:
                    direction = math_helpers.calculate_direction(
                        from_pos=node_pos, to_pos=self_pos)

                    # If same x, then it's a vertical line, so we want the closest node
                    if node_pos[0] == self_pos[0]:
                        if distance < y_node[1]:
                            y_node = (node_pos, distance, direction)
                            # obstacle_angle_array.append((direction, node_pos))
                    # If same y, then it's a horizontal line, so we want the closest node
                    if node_pos[1] == self_pos[1]:
                        if distance < x_node[1]:
                            x_node = (node_pos, distance, direction)
                            # obstacle_angle_array.append((direction, node_pos))

                    logger.debug(
                        f"Ball distance from obstacle node {node_pos} is {distance} with direction {math.degrees(direction)} deg")
        if x_node[0] == (0, 0) and y_node[0] == (0, 0):
            logger.debug("Ball isn't close to an obstacle.")
            # We need to return two points to make a line, even if it's the same
            self.drivePath = [self_pos, self_pos]
            return self.drivePath
        logger.debug("Ball is close to an obstacle.")

        # Get the path to the balls while avoiding obstacles

        angle = 0.0
        # avg_angles = np.mean([elem[0] for elem in obstacle_angle_array])
        # angle = avg_angles % (2 * math.pi)

        safety_length = SAFETY_LENGTH

        # We get the angle of the ball to the obstacle
        if x_node[0] != (0, 0) and y_node[0] != (0, 0):
            safety_length = SAFETY_LENGTH_CORNER
            if x_node[0][0] < self_pos[0] and y_node[0][1] < self_pos[1]:
                # bottom left
                angle = math.radians(45)
            elif x_node[0][0] < self_pos[0] and y_node[0][1] > self_pos[1]:
                # top left
                angle = math.radians(315)
            elif x_node[0][0] > self_pos[0] and y_node[0][1] < self_pos[1]:
                # bottom right
                angle = math.radians(135)
            elif x_node[0][0] > self_pos[0] and y_node[0][1] > self_pos[1]:
                # top right
                angle = math.radians(225)
        elif x_node[0] != (0, 0):
            if x_node[0][0] < self_pos[0]:
                # left
                angle = math.radians(0)
            elif x_node[0][0] > self_pos[0]:
                # right
                angle = math.radians(180)
        elif y_node[0] != (0, 0):
            if y_node[0][1] < self_pos[1]:
                # bottom
                angle = math.radians(90)
            elif y_node[0][1] > self_pos[1]:
                # top
                angle = math.radians(270)

        # We get the angle of the ball to the obstacle
        dx = math.cos(angle) * safety_length
        dy = math.sin(angle) * safety_length
        x1 = int(self_pos[0] + dx)
        y1 = int(self_pos[1] + dy)
        self.drivePath = [(x1, y1), self_pos]
        logger.debug(
            f"Ball drive path: {self.drivePath} - angle is {math.degrees(angle)} deg")
        return self.drivePath


class Obstacle:
    def __init__(self, path: List[Node], points: Optional[List[Tuple[int, int]]] = None) -> None:
        """
        Initializes a new instance of the Obstacle class.

        Args:
            path (list): The path of the obstacle.
            points (list, optional): The points marking the corners of the obstacle. Used to draw the path.

        Returns:
            None
        """
        self.path: List[Node] = path
        self.points: Optional[List[Tuple[int, int]]] = points

    async def get_distance(self, position: Tuple[int, int]) -> Tuple[float, Node]:
        """
        Returns the distance from the given position to the obstacle.

        Args:
            position (tuple): The position as a tuple of (x, y) coordinates.

        Returns:
            Tuple[float, Node]: The distance to the obstacle and the closest node.
        """
        min_distance = math.inf
        closest_node = None
        for node in self.path:
            distance = math_helpers.calculate_distance(
                position, node.get_position())
            if distance < min_distance:
                min_distance = distance
                closest_node = node
            await asyncio.sleep(0)
        return min_distance, closest_node

    async def is_about_to_collide(self, position: Tuple[int, int], heading: float) -> bool:
        """
        Checks whether the robot is about to collide with the obstacle.
        :param position: the position of the robot
        :param heading: the heading of the robot
        :return: True if the robot is about to collide with the obstacle, False otherwise
        """
        distance, node = await self.get_distance(position)
        if distance < COLLISION_DISTANCE:
            logger.debug("Robot is within collision distance of obstacle.")
            return True
            # direction = math_helpers.calculate_direction(to_pos=node.get_position(), from_pos=position)
            # if abs(direction - heading) < math.radians(DIRECTION_DIFFERENCE):
            #     logger.debug("Robot is about to collide with obstacle.")
            #     return True
        return False

    def is_same_position(self, pos: Tuple[int, int]) -> bool:
        """
        Checks whether the given position is the same as the obstacle's position.

        Args:
            pos (tuple): The position as a tuple of (x, y) coordinates.

        Returns:
            bool: True if the position is the same as the obstacle's position, False otherwise.
        """
        return self.path[0].get_position() == pos


class Goal:
    def __init__(self, path: List[Node], points: Optional[List[Tuple[int, int]]] = None, small=False) -> None:
        """
        Initializes a new instance of the Goal class.

        Args:
            path (list): The path of the goal.
            points (list, optional): The points marking the corners of the goal. Used to draw the path.
            small (bool, optional): Indicates whether the goal is small or big. Defaults to False == Big goal.

        Returns:
            None
        """
        if len(points) != 2:
            raise ValueError("Points must be a list of two tuples for a goal.")

        self.path: List[Node] = path
        self.small = small
        self.points: Optional[List[Tuple[int, int]]] = points

    def get_middle_and_angle(self) -> Tuple[Tuple[int, int], float]:
        """
        Gets the middle of the goal and the angle to it.
        Returns:
            Tuple[Tuple[int, int], float]: The middle of the goal and the angle to it.
        """
        # Get middle of the goal
        middle = math_helpers.get_middle_between_two_points(
            self.points[0], self.points[1])

        if self.points[0][1] < self.points[1][1]:
            angle = math_helpers.calculate_direction(
                to_pos=self.points[1], from_pos=self.points[0])
        else:
            angle = math_helpers.calculate_direction(
                to_pos=self.points[0], from_pos=self.points[1])

        # Get angle to middle of the goal
        if middle[0] < TRACK_GLOBAL.bounds["x"] / 2:
            angle = (angle - (math.pi/2)) % (2 * math.pi)
        else:
            angle = (angle + (math.pi/2)) % (2 * math.pi)

        return middle, angle

    async def deliver_path(self) -> List[Tuple[int, int]]:
        """
        Delivers the balls to the goal.

        Returns:
            List[Tuple[int, int]]: The points to drive to.
        """
        # Get the middle and the angle
        middle, angle = self.get_middle_and_angle()

        # Get points to drive to
        point_1 = (int(middle[0] + math.cos(angle) * DELIVERY_DISTANCE_FAR),
                   int(middle[1] + math.sin(angle) * DELIVERY_DISTANCE_FAR))
        point_2 = (int(middle[0] + math.cos(angle) * DELIVERY_DISTANCE),
                   int(middle[1] + math.sin(angle) * DELIVERY_DISTANCE))

        return [point_1, point_2]

    def is_in_delivery_distance_min(self) -> bool:
        """
        Checks whether the robot far enough from the delivery distance.
        We don't want the robot too close to the goal!
        :return: True if the robot is in the delivery distance, False otherwise
        """
        # Get the middle and the angle
        middle, _ = self.get_middle_and_angle()

        # Get the distance to the middle
        distance = math_helpers.calculate_distance(
            TRACK_GLOBAL.get_middle_position(), middle)

        # Check if robot is close enough to the middle
        if distance < DELIVERY_DISTANCE - (DELIVERY_DISTANCE_X_DIFF / 2):
            return False

        return True

    def is_in_delivery_distance_max(self) -> bool:
        """
        Checks whether the robot is close enough to the delivery distance.
        We don't want the robot too far from the goal!
        :return: True if the robot is in the delivery distance, False otherwise
        """
        # Get the middle and the angle
        middle, _ = self.get_middle_and_angle()

        # Get the distance to the middle
        distance = math_helpers.calculate_distance(
            TRACK_GLOBAL.get_middle_position(), middle)

        # Check if robot is close enough to the middle
        if distance > DELIVERY_DISTANCE + (DELIVERY_DISTANCE_X_DIFF / 2):
            return False

        return True

    def is_in_delivery_distance(self) -> bool:
        """
        Checks whether the robot is in the delivery distance.
        :return: True if the robot is in the delivery distance, False otherwise
        """
        # Check if robot is close enough to the middle
        if not self.is_in_delivery_distance_min():
            return False

        if not self.is_in_delivery_distance_max():
            return False

        return True

    def get_angle_to_middle(self) -> float:
        """
        Gets the angle to the middle of the goal.
        :return: The angle to the middle of the goal.
        """
        # Get the middle and the angle
        middle, angle = self.get_middle_and_angle()

        angle_to_middle = (angle + math.pi) % (2*math.pi)

        # Get the angle to the middle
        # angle_to_middle = math_helpers.calculate_direction(to_pos=middle, from_pos=TRACK_GLOBAL.get_middle_position())

        # angle = math_helpers.calculate_direction(self.points[0], self.points[1])

        return angle_to_middle

    def delivery_direction_diff(self) -> float:
        """
        Gets the difference between the robot's heading and the delivery direction.
        :return: The difference between the robot's heading and the delivery direction.
        """
        # Get the difference between the robot's heading and the delivery direction
        return self.get_angle_to_middle() - TRACK_GLOBAL.robot_direction

    def is_in_delivery_direction(self) -> bool:
        """
        Checks whether the robot is in the delivery direction.
        :return: True if the robot is in the delivery direction, False otherwise
        """
        # Check if robot is facing the middle
        if abs(self.delivery_direction_diff()) > math.radians(HEADING_DIFFERENCE_DELIVERY):
            return False

        return True

    def is_in_delivery_height(self) -> bool:
        # Get the middle
        middle, _ = self.get_middle_and_angle()
        robot_pos = TRACK_GLOBAL.get_middle_position()
        if not robot_pos:
            robot_pos = TRACK_GLOBAL.get_front_position()
        if not robot_pos:
            logger.debug(
                "Robot pos is none??? (￣ε(#￣)☆╰╮o(￣皿￣///)")
            return False

        if abs(middle[1] - robot_pos[1]) > DELIVERY_DISTANCE_Y:
            return False

        return True

    def is_in_delivery_position(self) -> bool:
        """
        Checks whether the robot is in the delivery position.
        :return: True if the robot is in the delivery position, False otherwise
        """
        # Check if robot is close enough to the middle
        if not self.is_in_delivery_distance():
            return False

        # Check if robot is facing the middle
        if not self.is_in_delivery_direction():
            return False

        # Check if robot is on the correct y-coordinate
        if not self.is_in_delivery_height():
            return False

        return True


class NodeData:
    def __init__(self, node: Node, g: float = math.inf, h: float = math.inf, parent: Optional[Node] = None) -> None:
        """
        Initializes a new instance of the NodeData class.

        Args:
            node (Node): The node associated with the data.
            g (float): The cost from the start node to the current node.
            h (float): The estimated cost from the current node to the goal node.
            parent (Optional[Node]): The parent node. Defaults to None.

        Returns:
            None
        """
        self.node = node
        self.g = g
        self.h = h
        self.parent = parent

        self.f = self.g + self.h

    def __lt__(self, other: Any) -> bool:
        """
        Less than comparison operator.

        Args:
            other (Any): The other object to compare.

        Returns:
            bool: Always returns True.
        """
        return True

    def __le__(self, other: Any) -> bool:
        """
        Less than or equal to comparison operator.

        Args:
            other (Any): The other object to compare.

        Returns:
            bool: Always returns True.
        """
        return True


class Graph:
    def __init__(self, size_x: int = 500, size_y: int = 200) -> None:
        """
        Initializes a new instance of the Graph class.
        :param size_x: The width of the graph.
        :param size_y: The height of the graph.
        """
        if size_x and size_y:
            range_x = range(size_x)
            range_y = range(size_y)
            self.nodes: np.ndarray[Node] = np.empty(
                (size_y, size_x), dtype=Node)

            # Create nodes
            for y in range_y:
                for x in range_x:
                    pos = (x, y)
                    node = Node(pos)
                    self.nodes[y, x] = node

            # Connect neighbors
            rows, cols = self.nodes.shape

            for y in range(rows):
                for x in range(cols):
                    node = self.nodes[y, x]

                    # Check left and right neighbors
                    if x - 1 >= 0:
                        left = self.nodes[y, x - 1]
                        node.add_neighbour(left, 1)

                        if y - 1 >= 0:
                            left_up = self.nodes[y - 1, x - 1]
                            node.add_neighbour(left_up, distance_across)

                        if y + 1 < rows:
                            left_down = self.nodes[y + 1, x - 1]
                            node.add_neighbour(left_down, distance_across)

                    if x + 1 < cols:
                        right = self.nodes[y, x + 1]
                        node.add_neighbour(right, 1)

                        if y - 1 >= 0:
                            right_up = self.nodes[y - 1, x + 1]
                            node.add_neighbour(right_up, distance_across)

                        if y + 1 < rows:
                            right_down = self.nodes[y + 1, x + 1]
                            node.add_neighbour(right_down, distance_across)

                    # Check top and bottom neighbors
                    if y - 1 >= 0:
                        top = self.nodes[y - 1, x]
                        node.add_neighbour(top, 1)

                    if y + 1 < rows:
                        bottom = self.nodes[y + 1, x]
                        node.add_neighbour(bottom, 1)

    def add_node(self, coordinates: tuple) -> None:
        """
        Adds a node to the graph.
        :param coordinates: The coordinates of the node.
        :return: None
        """
        np.append(self.nodes, Node(coordinates))

    def get_node(self, pos: tuple) -> Optional[Node]:
        """
        Gets the node at the specified position.
        :param pos: The position of the node.
        :return: The node at the specified position.
        """
        if 0 <= pos[0] < len(self.nodes[0]) and 0 <= pos[1] < len(self.nodes):
            return self.nodes[pos[1]][pos[0]]
        else:
            logger.debug(
                f"Node with position {pos} not found, not within x and y bounds = ({len(self.nodes[0])}, {len(self.nodes)})")
            return None

    def get_nodes_in_path(self, path: list) -> list:
        """
        Gets all nodes in the specified path.
        :param path: The path to get the nodes from.
        :return: A list of nodes in the path.
        """
        node_plusminus = 1
        nodes_in_path = []
        for i in range(len(path)):
            pos = path[i]
            node = self.get_node(pos)
            if node:
                logger.debug(f"Adding {pos[0]}, {pos[1]}")
                nodes_in_path.append(node)
                if i >= 1:
                    # Get all nodes between the current node and the previous node
                    # Time for good-ol high school algebra for equation for line between two points, yaaay
                    y1 = float(path[i - 1][1])
                    y2 = float(pos[1])
                    x1 = float(path[i - 1][0])
                    x2 = float(pos[0])

                    # Are they even apart by more than 1?
                    if abs(x1 - x2) > 1:
                        slope = (y1 - y2) / (x1 - x2)
                        y_intercept = (x1 * y2 - x2 * y1) / (x1 - x2)
                        x_min = min(x1, x2)
                        x_max = max(x1, x2)
                        for x in range(int(x_min + 1), int(x_max)):
                            y = int(slope * float(x) + y_intercept)
                            logger.debug(
                                f"Adding {x}+-{node_plusminus}, {y}+-{node_plusminus}")
                            for j in range(-node_plusminus, node_plusminus + 1):
                                pos = (x + j, y + j)
                                if pos not in path:
                                    node = self.get_node(pos)
                                    if node:
                                        nodes_in_path.append(node)
                    if abs(y1 - y2) > 1:
                        slope = (x1 - x2) / (y1 - y2)
                        x_intercept = (y1 * x2 - y2 * x1) / (y1 - y2)
                        y_min = min(y1, y2)
                        y_max = max(y1, y2)
                        for y in range(int(y_min + 1), int(y_max)):
                            x = int(slope * float(y) + x_intercept)
                            logger.debug(
                                f"Adding {x}+-{node_plusminus}, {y}+-{node_plusminus}")
                            for j in range(-node_plusminus, node_plusminus + 1):
                                pos = (x + j, y + j)
                                if pos not in path:
                                    node = self.get_node(pos)
                                    if node:
                                        nodes_in_path.append(node)

        return list(set(nodes_in_path))

    def add_edge(self, node_1: Node, node_2: Node) -> None:
        """
        Adds an edge between two nodes.
        :param node_1: The first node.
        :param node_2: The second node.
        :return: None
        """
        if node_1 is not node_2 and node_1 and node_2:
            if node_2 not in node_1.get_neighbour_nodes():
                node_1.add_neighbour(node_2)
            if node_1 not in node_2.get_neighbour_nodes():
                node_2.add_neighbour(node_1)

    def remove_edge(self, node_1: Node, node_2: Node) -> None:
        """
        Removes an edge between two nodes.
        :param node_1: The first node.
        :param node_2: The second node.
        :return: None
        """
        if node_1 is not node_2 and node_1 and node_2:
            if node_2 in node_1.get_neighbour_nodes():
                node_1.remove_neighbour(node_2)
            if node_1 in node_2.get_neighbour_nodes():
                node_2.remove_neighbour(node_1)

    # Manhattan Distance heuristic for A*
    def h(self, start_node: Node, dst_node: Node) -> float:
        """
        The heuristic function for A*. This is the Manhattan Distance.
        :param start_node: The start node.
        :param dst_node: The destination node.
        :return: The heuristic value.
        """
        dx = abs(start_node.x - dst_node.x)
        dy = abs(start_node.y - dst_node.y)
        return dx + dy

    # Get path and cost using A*
    async def get_path(self, start_node: Node, dst_node: Node) -> List[NodeData]:
        """
        Gets the path between two nodes using A*.
        :param start_node: The start node.
        :param dst_node: The destination node.
        :return: A list of nodes in the path.
        """
        logger.debug(
            f"Getting path between {start_node.x}, {start_node.y} and {dst_node.x}, {dst_node.y}")

        # Initialize the start node data
        start_node_data = NodeData(
            start_node, 0, self.h(start_node, dst_node), None)

        # Create open and closed sets
        open_set: Set[Node] = {start_node}
        closed_set: Dict[Node, NodeData] = {}

        # A dictionary to store the best known cost from start to each node
        g_scores: Dict[Node, float] = {start_node: 0}

        # A dictionary to store the estimated total cost from start to each node
        f_scores: Dict[Node, float] = {start_node: start_node_data.f}

        # A priority queue to efficiently extract the node with the lowest f-score
        open_queue: List[Tuple[float, Node]] = [
            (f_scores[start_node], start_node)]

        # A counter to track the number of iterations
        iteration = 0

        # The current node
        current_node: Node
        while open_queue:
            iteration += 1
            if iteration % 500 == 0:
                # if DEBUG:
                #     print(f"Iteration: {iteration}")
                await asyncio.sleep(0)

            _, current_node = heapq.heappop(open_queue)

            closed_set[current_node] = NodeData(
                node=current_node,
                g=g_scores[current_node],
                h=self.h(current_node, dst_node),
                parent=closed_set.get(current_node, None).parent if closed_set.get(
                    current_node, None) else None
            )

            if current_node is dst_node:
                # Destination reached, construct and return the final path
                final_path: List[NodeData] = []
                while current_node:
                    final_path.insert(0, closed_set[current_node])
                    current_node = closed_set[current_node].parent
                return final_path

            for neighbour in current_node.neighbours:
                neighbour_node: Node = neighbour["node"]
                neighbour_weight: float = neighbour["weight"]

                if neighbour_node in closed_set:
                    continue

                neighbour_g_score = g_scores[current_node] + neighbour_weight

                if neighbour_node not in open_set:
                    # Add the neighbour to the open set
                    neighbour_data = NodeData(
                        node=neighbour_node,
                        g=neighbour_g_score,
                        h=self.h(neighbour_node, dst_node),
                        parent=current_node
                    )
                    open_set.add(neighbour_node)
                    closed_set[neighbour_node] = neighbour_data
                    g_scores[neighbour_node] = neighbour_g_score
                    f_scores[neighbour_node] = neighbour_data.f
                    heapq.heappush(
                        open_queue, (f_scores[neighbour_node], neighbour_node))
                elif neighbour_g_score < g_scores[neighbour_node]:
                    # Update the neighbour's scores and parent if a better path is found
                    neighbour_data = closed_set[neighbour_node]
                    neighbour_data.g = neighbour_g_score
                    neighbour_data.parent = current_node
                    g_scores[neighbour_node] = neighbour_g_score
                    f_scores[neighbour_node] = neighbour_data.f
                    heapq.heapify(open_queue)

        # No path found
        return []

    def draw(self, robot_pos: tuple = None, balls: list = None, path: list = None) -> None:
        """
        Draws the grid.
        :param robot_pos: The position of the robot.
        :param balls: The list of balls.
        :param path: The path to draw.
        :return: None
        """
        if balls is None:
            balls = []
        if path is None:
            path = []
        for y, row in enumerate(self.nodes):
            nodes_string = ""
            conections_string = ""
            for x, node in enumerate(row):
                if robot_pos:
                    if robot_pos[0] == x and robot_pos[1] == y:
                        nodes_string = nodes_string + Fore.BLUE + " R" + Style.RESET_ALL
                    elif balls:
                        found = False
                        for ball in balls:
                            if ball.x == x and ball.y == y:
                                nodes_string = nodes_string + Fore.RED + " B" + Style.RESET_ALL
                                found = True
                                break
                        if not found:
                            nodes_string = nodes_string + " o"
                    else:
                        nodes_string = nodes_string + " o"
                elif balls:
                    found = False
                    for ball in balls:
                        if ball.x == x and ball.y == y:
                            nodes_string = nodes_string + Fore.RED + " B" + Style.RESET_ALL
                            found = True
                            break
                    if not found:
                        nodes_string = nodes_string + " o"
                else:
                    nodes_string = nodes_string + " o"

                neighbours = {"down": 0, "right": 0, "down_right": 0}
                if path:
                    node_path_parent = None
                    node_path_child = None
                    for node_path_check in path:
                        if node_path_check.parent:
                            if node_path_check.node == node:
                                node_path_parent = node_path_check.parent.node
                            if node_path_check.parent.node == node:
                                node_path_child = node_path_check.node
                            if node_path_parent and node_path_child:
                                break

                    if node_path_parent or node_path_child:
                        # print(f"got node relationships for node ({node.x}, {node.y})")
                        # if node_path_parent:
                        #     print(f"node parent debug ({node_path_parent.x}, {node_path_parent.y})")
                        # if node_path_child:
                        #     print(f"node child debug ({node_path_child.x}, {node_path_child.y})")

                        if node_path_parent and node_path_parent.y >= node.y and node_path_parent.x >= node.x:
                            # print(f"node parent ({node_path_parent.x}, {node_path_parent.y})")
                            if node.x == node_path_parent.x:
                                if node.y + 1 == node_path_parent.y:
                                    neighbours["down"] = 1
                            elif node.x + 1 == node_path_parent.x:
                                if node.y == node_path_parent.y:
                                    neighbours["right"] = 1
                                elif node.y + 1 == node_path_parent.y:
                                    neighbours["down_right"] += 1
                        if node_path_child and node_path_child.y >= node.y and node_path_child.x >= node.x:
                            # print(f"node child ({node_path_child.x}, {node_path_child.y})")
                            if node.x == node_path_child.x:
                                if node.y + 1 == node_path_child.y:
                                    neighbours["down"] = 1
                            elif node.x + 1 == node_path_child.x:
                                if node.y == node_path_child.y:
                                    neighbours["right"] = 1
                                elif node.y + 1 == node_path_child.y:
                                    neighbours["down_right"] += 1
                else:
                    for neighbour in node.get_neighbour_nodes():
                        if node.x == neighbour.x:
                            if node.y + 1 == neighbour.y:
                                neighbours["down"] = 1
                        elif node.x + 1 == neighbour.x:
                            if node.y == neighbour.y:
                                neighbours["right"] = 1
                            elif node.y + 1 == neighbour.y:
                                neighbours["down_right"] += 1
                    down_node = self.get_node((node.x, node.y + 1))
                    if down_node:
                        # check for cross the other way
                        for neighbour_down in down_node.get_neighbour_nodes():
                            if down_node.y == neighbour_down.y + 1 and down_node.x == neighbour_down.x - 1:
                                neighbours["down_right"] += 2
                                break

                # print(neighbours)

                if neighbours["right"]:
                    nodes_string = nodes_string + " -"
                else:
                    nodes_string = nodes_string + "  "

                if neighbours["down"]:
                    if neighbours["down_right"] == 1:
                        conections_string = conections_string + " | \\"
                    elif neighbours["down_right"] == 2:
                        conections_string = conections_string + " | /"
                    elif neighbours["down_right"] == 3:
                        conections_string = conections_string + " | X"
                    else:
                        conections_string = conections_string + " |  "
                elif neighbours["down_right"] == 1:
                    conections_string = conections_string + "   \\"
                elif neighbours["down_right"] == 2:
                    conections_string = conections_string + "   /"
                elif neighbours["down_right"] == 3:
                    conections_string = conections_string + "   X"
                else:
                    conections_string = conections_string + "    "

                # print(f"({x.x}, {x.y})", end=" ")
            if DEBUG:
                print(nodes_string)
                print(conections_string)


class Track:
    def __init__(self, bounds: Dict[str, int]):
        """
        Initialise the track with the bounds of the track
        :param bounds: The bounds of the track
        """
        self.bounds: Dict[str, int] = bounds
        self.balls: List[Ball] = []
        self.obstacles: List[Obstacle] = []
        self.small_goal: Optional[Goal] = None
        self.big_goal: Optional[Goal] = None
        self.robot_pos: Tuple[int, int] = (0, 0)
        self.robot_front_pos: Tuple[int, int] = (0, 0)
        self.robot_rear_pos: Tuple[int, int] = (0, 0)
        self.robot_direction: float = 0.0
        self.path: List[NodeData] = []

        self.last_target_path: List[NodeData] = []
        self.last_target_node: Optional[Node] = None
        self.integral: float = 0.0
        self.previous_error: float = 0.0

        self.graph: Graph = Graph(bounds["x"], bounds["y"])

    def add_ball(self, ball: Ball) -> None:
        """
        Add a ball to the track
        :param ball: The ball to add
        :return: None
        """
        self.balls.append(ball)

    def clear_balls(self) -> None:
        """
        Clear all balls from the track
        :return: None
        """
        self.balls = []

    def get_front_position(self) -> Tuple[int, int]:
        """
        Get the position to use for calculating paths (we want to use the front if possible)
        Returns:
            The position to use for calculating paths
        """
        if self.robot_front_pos:
            # logger.debug(f"Front pos: {self.robot_front_pos}")
            return self.robot_front_pos
        return self.robot_pos

    def get_middle_position(self) -> Tuple[int, int]:
        """
        Get the position to use for calculating angles
        Returns:
            The position to use for calculating angles
        """
        if self.robot_rear_pos:
            return self.robot_rear_pos
        return self.robot_pos

    def set_robot_direction(self, robot_direction: float) -> None:
        """
        Set the direction of the robot
        :param robot_direction: The direction of the robot
        :return: None
        """
        logger.debug(
            f"Old direction: {math.degrees(self.robot_direction)} deg, new direction: {math.degrees(robot_direction)} deg")

        self.robot_direction = robot_direction % (2 * math.pi)

    def set_robot_pos(self, middle: Tuple[int, int], front: Optional[Tuple[int, int]] = None, rear: Optional[Tuple[int, int]] = None) -> None:
        """
        Set the position of the robot
        :param middle: The middle position of the robot
        :param front: The front position of the robot
        :param rear: The rear position of the robot
        :return: None
        """
        logger.debug(f"Old position: {self.robot_pos}, new position: {middle}")

        # Update position
        self.robot_pos = middle
        self.robot_front_pos = front
        self.robot_rear_pos = rear

    async def calculate_path(self, objects_to_navigate_to: List[List[Tuple[int, int]]]) -> None:
        """
        Calculate the path to the next ball
        :param objects_to_navigate_to: The objects to navigate to
        :return: None
        """
        # if DEBUG:
        #     print("Calculating path")
        if not objects_to_navigate_to:
            logger.debug("No objects, returning empty")
            self.path = []
            return

        # For every ball calculate the path, then choose the best path
        logger.debug("Calculating path for every ball")
        paths: List[List[NodeData]] = []
        if objects_to_navigate_to:
            robot_node = self.graph.get_node(self.get_middle_position())
            if not robot_node:
                logger.debug("No robot node, returning empty")
                self.path = []
                return
            object_nodes = [self.graph.get_node(
                obj[0]) for obj in objects_to_navigate_to if obj]
            tasks = [asyncio.create_task(self.graph.get_path(start_node=robot_node, dst_node=ball_node)) for ball_node in object_nodes if
                     ball_node]
            if tasks:
                done, pending = await asyncio.wait(tasks, timeout=TIMEOUT_GET_PATH, return_when=asyncio.ALL_COMPLETED)
                logger.debug(
                    f"Done calculating paths: {len(done)} done, {len(pending)} timed out")
                [task.cancel() for task in pending]
                for task in done:
                    path_list: List[NodeData] = []
                    try:
                        # Get path from task
                        path_list = await task
                    except TimeoutError:
                        pass
                    # Add path to paths
                    if path_list:
                        # Add the last node to the path
                        last_in_path_list = path_list[-1]
                        target_obj = [obj[1] for obj in objects_to_navigate_to if
                                      obj[0] == last_in_path_list.node.get_position()]
                        path_list.append(
                            NodeData(node=self.graph.get_node(target_obj[0]), g=last_in_path_list.g,
                                     h=last_in_path_list.h,
                                     parent=last_in_path_list.node))
                        paths.append(path_list)

        if paths:
            # if DEBUG:
            #     print("Got paths, finding best path")
            min_path = {"path": None, "f": None}
            for path in paths:
                if path:
                    if not min_path["path"] or path[-1].f < min_path["f"]:
                        min_path["path"] = path
                        min_path["f"] = path[-1].f
            if min_path["path"] and len(min_path["path"]) > 1:
                logger.debug(f"Best path found with weight {min_path['f']}")
                # Log the path found
                # for i in range(len(min_path["path"])):
                #     node = min_path["path"][i].node
                #     end = " -> " if i < len(min_path['path']) - 1 else "\n"
                #     if DEBUG:
                #         print(f"({node.x}, {node.y})", end=end)
                self.path = min_path["path"]
            else:
                logger.debug("Found no path!")
                self.path = []
        else:
            logger.debug("Got no paths, returning empty")
            self.path = []

    def clear_obstacles(self) -> None:
        """
        Clear all obstacles from the track
        :return: None
        """
        for obstacle in self.obstacles:
            for node in obstacle.path:
                for x in range(-1, 1 + 1):
                    for y in range(-1, 1 + 1):
                        neighbour = self.graph.get_node(
                            (node.x + x, node.y + y))
                        self.graph.add_edge(node, neighbour)
        self.obstacles = []

    def add_obstacle(self, obstacle: Obstacle) -> None:
        """
        Add an obstacle to the track
        :param obstacle: The obstacle to add
        :return: None
        """
        for node in obstacle.path:
            # print(f"Removing node edges for node at {node.x}, {node.y}")
            for x in range(-1, 1 + 1):
                for y in range(-1, 1 + 1):
                    neighbour = self.graph.get_node((node.x + x, node.y + y))
                    self.graph.remove_edge(node, neighbour)
            if node.neighbours:
                logger.debug(
                    f"[add_obstacle] Failed to remove all neighbours for obstacle node at {node.x}, {node.y}")
        self.obstacles.append(obstacle)

    def add_goal(self, goal: Goal) -> None:
        """
        Add a goal to the track
        :param goal: The goal to add
        :return: None
        """
        if goal.small:
            self.small_goal = goal
        else:
            self.big_goal = goal

    def draw(self, with_path: bool = False) -> None:
        """
        Draw the track
        :param with_path: Whether to draw the path or not
        :return: None
        """
        if with_path:
            self.graph.draw(robot_pos=self.robot_pos,
                            balls=self.balls, path=self.path)
        else:
            self.graph.draw(robot_pos=self.robot_pos, balls=self.balls)

    async def plot(self) -> None:
        """
        Plot the path
        :return: None
        """
        await self.calculate_path([ball.drivePath for ball in self.balls if ball.get_drive_path()])
        points_in_path = [[], []]
        for nodeData in self.path:
            points_in_path[0].append(nodeData.node.x)
            points_in_path[1].append(nodeData.node.y)

        plt.figure()
        plt.scatter(points_in_path[0], points_in_path[1])
        plt.xticks(
            np.arange(min(points_in_path[0]), max(points_in_path[0]) + 1, 1))
        plt.yticks(
            np.arange(min(points_in_path[1]), max(points_in_path[1]) + 1, 1))
        plt.gca().invert_yaxis()
        plt.grid()
        # Add labels and title
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('Plotting Points')

        # Display the plot
        plt.show()


TRACK_GLOBAL: Optional[Track] = None


async def simplify_path(path: List[NodeData]) -> List[NodeData]:
    """
    Summarizes/simplifies a path of points/nodes into the least amount of points/nodes needed for a robot to traverse
    the path.

    Args:
        path (List[NodeData]): The path object containing the path to be summarized.

    Returns:
        list: The summarized path as a list of NodeData objects.
    """
    new_path: List[NodeData] = []

    # Check if the path is empty or contains only two points
    if len(path) <= 1:
        return []
    elif len(path) == 2:
        return path

    # Add the first point as the initial target
    new_path.append(path[0])
    current_from_node = path[0]

    # Current heading diff
    last_direction_diff = 0.0
    for i in range(1, len(path) - 1):
        previous_node = path[i - 1]
        current_node = path[i]

        # Check forward a bit to see if it's the same line
        same_path = True
        for j in range(i + 1, i + 4):
            # If last node stop
            if j >= len(path):
                break

            # Get j'th next node
            next_node = path[j]

            # If previous node is current from node, skip
            if previous_node == current_from_node:
                same_path = True
                continue

            # If on same line, ie. x and y
            if math_helpers.is_on_same_line(current_from_node.node.get_position(), current_node.node.get_position()):
                same_path = True
                continue

            # if DEBUG:
            #     print(f"Checking {current_node.node.get_position()}")

            # If same direction
            direction_diff = math_helpers.calculate_direction_difference(
                current_from_node.node.get_position(
                ), previous_node.node.get_position(), next_node.node.get_position()
            )

            # Check if the direction difference is within the tolerance
            if direction_diff <= math.radians(15):  # 15 degrees tolerance
                same_path = True
                continue

            # Check if direction diff is same as last
            if abs(direction_diff - last_direction_diff) <= math.radians(2):
                last_direction_diff = direction_diff
                same_path = True
                continue
            last_direction_diff = direction_diff

            same_path = False

        if not same_path:
            # Add the current point to the new path
            new_path.append(current_node)
            current_from_node = current_node

    # Add the last point to the new path
    new_path.append(path[-1])
    return new_path


async def add_buffer_to_path(path: List[NodeData]) -> List[NodeData]:
    """
    Using the list of obstacles in the track, moves nodes in a path away from the obstacles by a certain amount.

    Args:
        path (List[NodeData]): The path to move away from obstacles.

    Returns:
        list: The new path as a list of NodeData objects.
    """
    new_path: List[NodeData] = []

    # If the path is empty, return it
    if not path:
        return path

    # Loop through the path
    for i in range(1, len(path)):
        # Get the current node
        current_node = path[i]

        # Check if the current node is in an obstacle
        for obstacle in TRACK_GLOBAL.obstacles:
            distance_from_obstacle, obstacle_node = await obstacle.get_distance(current_node.node.get_position())
            if distance_from_obstacle < PATH_OBSTACLE_DISTANCE:
                # If the node is in an obstacle, move it away from the obstacle
                node_x = current_node.node.x
                node_y = current_node.node.y
                # if, on the x-axis, the obstacle is on the left of the node, move the node to the right
                if obstacle_node.x < node_x < obstacle_node.x + PATH_OBSTACLE_DISTANCE:
                    node_x = obstacle_node.x + PATH_OBSTACLE_DISTANCE
                elif obstacle_node.x > node_x > obstacle_node.x - PATH_OBSTACLE_DISTANCE:
                    node_x = obstacle_node.x - PATH_OBSTACLE_DISTANCE
                # same for y-axis
                if obstacle_node.y < node_y < obstacle_node.y + PATH_OBSTACLE_DISTANCE:
                    node_y = obstacle_node.y + PATH_OBSTACLE_DISTANCE
                elif obstacle_node.y > node_y > obstacle_node.y - PATH_OBSTACLE_DISTANCE:
                    node_y = obstacle_node.y - PATH_OBSTACLE_DISTANCE
                # Update the current node
                current_node = NodeData(Node((math.ceil(node_x), math.ceil(node_y))), g=current_node.g,
                                        h=current_node.h, parent=current_node.parent)

                break

        # Add the current node to the new path
        new_path.append(current_node)

    return new_path


async def check_new_path(path_queue: multiprocessing.JoinableQueue, session: aiohttp.ClientSession) -> bool:
    """
    Checks if the path in the track is the same as last, and if it is, updates the current path. Also updates if it isn't.

    :param path_queue: The queue to send the current path to the AI.
    :param session: The aiohttp session to use for the request.
    :return: True if we should drive, else False.
    """
    track = TRACK_GLOBAL
    if not track:
        return False
    robot_position = track.get_middle_position()
    # if DEBUG:
    #     print(
    #         f"current last target path: {[nodedata.node.get_position() for nodedata in last_target_path] if last_target_path else []}\n"
    #         f"new path target: {track.path[-1].node.get_position() if track.path else []}")
    # If not set already, set and reset
    # Check if the new path target is different than before
    if track.last_target_path and isinstance(track.last_target_path, list) and \
            not is_target_different(track, track.last_target_path[-1].node, track.path[-1].node):
        # if DEBUG:
        #     print("No change in target, keeping current path.")

        # If robot is at target, pop
        # if DEBUG:
        #     print("Checking if robot has reached target")
        has_stopped = False
        while track.last_target_path and len(track.last_target_path) > 1 and not is_target_different(track, track.last_target_path[0].node,
                                                                                                     track.graph.get_node(robot_position)):
            logger.debug("Robot reached target, popping")
            track.last_target_path.pop(0)
            track.last_target_node = track.graph.get_node(robot_position)
            # TODO: Maybe remove this
            if not has_stopped:
                # Stop robot
                await robot_api.set_speeds(session, 0, 0)
                has_stopped = True
                # await asyncio.sleep(10)
            await asyncio.sleep(0)
        # If we passed the target, pop
        while track.last_target_path and len(track.last_target_path) > 1 and math_helpers.has_passed_target(track.last_target_path[0].node.get_position(),
                                                                                                            robot_position,
                                                                                                            track.last_target_node.get_position()):
            logger.debug("Robot passed target, popping")
            track.last_target_path.pop(0)
            track.last_target_node = track.graph.get_node(robot_position)
            # TODO: Maybe remove this
            if not has_stopped:
                # Stop robot
                await robot_api.set_speeds(session, 0, 0)
                has_stopped = True
                # await asyncio.sleep(10)
            await asyncio.sleep(0)

        logger.debug(
            f"Current optimized path: {[(nodedata.node.x, nodedata.node.y) for nodedata in track.last_target_path]}")
        if track.last_target_path:
            # Call adjust to adjust for error, and to clear point from list if we reach the target
            return True
        else:
            # if not path_queue.full():
            #     try:
            #         path_queue.put([])
            #     except:
            #         pass
            pass
    logger.debug("New target, making new path")

    if track.path[0].node.x == 0 and track.path[0].node.y == 0:
        # if DEBUG:
        #     print("Robot pos is 0,0, not making new path")
        return False

    track.integral = 0.0
    track.previous_error = 0.0

    # "Summarize" path into good points for targets
    track.last_target_path = await simplify_path(track.path)
    logger.debug(
        f"Current target optimized path: {[(nodedata.node.x, nodedata.node.y) for nodedata in track.last_target_path]}")

    # track.last_target_path = await add_buffer_to_path(track.last_target_path)
    # logger.debug(
    #     f"Current target optimized path, with buffer: {[(nodedata.node.x, nodedata.node.y) for nodedata in track.last_target_path]}")

    # Adjust and go to
    if track.last_target_path:
        full_path = [opencv_helpers.graph_position_to_opencv_position(
            nodedata.node.get_position()) for nodedata in track.last_target_path]

        if not path_queue.full():
            try:
                path_queue.put({"path": full_path if full_path else [], "obstacles": [[
                    opencv_helpers.graph_position_to_opencv_position(point) for point in obstacle.points] for obstacle in
                    track.obstacles] if track.obstacles else [],
                    "small_goal": [opencv_helpers.graph_position_to_opencv_position(point) for point in track.small_goal.points] if track.small_goal else [],
                    "big_goal": [opencv_helpers.graph_position_to_opencv_position(point) for point in track.big_goal.points] if track.big_goal else []})
            except:
                pass

        track.last_target_node = track.last_target_path[-1].node

        while track.last_target_path and not is_target_different(track, track.last_target_path[0].node, track.graph.get_node(robot_position)):
            logger.debug("Robot reached target, popping")
            track.last_target_path.pop(0)
            track.last_target_node = track.graph.get_node(robot_position)
            await asyncio.sleep(0)

        return True


def is_target_different(track: Track, target_node: Node, other_node: Node) -> bool:
    """
    Checks if the target is different from another node.
    :param track: The track object
    :param target_node: The target node
    :param other_node: The other node
    :return: True if the target is different, else False
    """
    # Define a threshold for difference based on your requirements
    # Assume a difference of 1.0 cm is significant, we use pixels though, so it depends on the distance and camera

    # if DEBUG:
    #     print(f"Checking if target is different between {target_node.get_position()} and {other_node.get_position()}")

    # Calculate the position difference
    position_diff = math_helpers.calculate_distance(
        target_node.get_position(), other_node.get_position())
    # Calculate the direction difference
    # direction_diff = abs(target_node.get_heading(track.get_middle_position()) - other_node.get_heading(track.get_middle_position()))

    # Check if target is significantly different from the last target
    # or direction_diff > math.pi / 4:
    if position_diff > TARGET_DIFFERENT_POSITION_DIFF_THRESHOLD:
        # if DEBUG:
        #     print(
        #         f"Different target: posdiff = {position_diff} > {position_threshold} | headdiff = {direction_diff} > {math.pi / 4}")
        return True

    return False


def setup_debug() -> None:
    """
    Sets up the debug environment.
    :return: None
    """
    bounds = {"x": 15, "y": 20}
    track = setup(bounds)

    track.set_robot_pos((9, 18))
    track.add_ball(Ball((8, 0)))

    obstacle_path = [(6, 10), (7, 11), (8, 10), (9, 9), (10, 10), (11, 11), (12, 10), (11, 9), (10, 8), (11, 7),
                     (12, 6), (11, 5), (10, 6), (9, 7), (8, 6), (7,
                                                                 5), (6, 6), (7, 7), (8, 8), (8, 9), (7, 6), (8, 7),
                     (9, 8), (8, 9), (7, 10), (11, 10), (10, 9), (10, 7), (11, 6)]
    obstacle = Obstacle(track.graph.get_nodes_in_path(obstacle_path))
    track.add_obstacle(obstacle)

    wall_path = []
    wall_path.extend([(x, y) for x in [0, bounds["x"] - 1]
                      for y in range(0, bounds["y"])])
    wall_path.extend([(x, y) for x in range(0, bounds["x"])
                      for y in [0, bounds["y"] - 1]])
    wall_obstacle = Obstacle(track.graph.get_nodes_in_path(wall_path))
    # track.add_obstacle(wall_obstacle)

    global TRACK_GLOBAL
    TRACK_GLOBAL = track


def setup(bounds: Dict[str, float]) -> Track:
    """
    Set up a track based on the provided bounds.

    Args:
        bounds (Dict[str, float]): The bounds of the track as a dictionary with keys 'x' and 'y',
                                   representing the maximum x-coordinate and y-coordinate respectively.

    Returns:
        Track: The created track object.

    """

    # Make sure bounds are integers
    bounds["x"] = math.ceil(bounds["x"])
    bounds["y"] = math.ceil(bounds["y"])

    track = Track(bounds)
    global TRACK_GLOBAL
    TRACK_GLOBAL = track
    return track

# if __name__ == "__main__":
#     setup_debug()
#
#     if TRACK_GLOBAL:
#         track = TRACK_GLOBAL
#
#         print("=========================\nDrawing with obstacles!\n=========================")
#         track.draw(False)
#
#         track.calculate_path()
#
#         print("=========================\nDrawing driving path!\n=========================")
#         track.draw(True)
#         # track.plot()
