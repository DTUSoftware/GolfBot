import asyncio
import heapq
import math
import os
from typing import Any, Optional, List, Tuple, Set, Dict, Union

import matplotlib.pyplot as plt
import numpy as np
from colorama import Fore
from colorama import Style
from colorama import init as colorama_init
from torch import multiprocessing

from client.Utils import math_helpers

DEBUG = "true" in os.environ.get('DEBUG', "True").lower()
TIMEOUT_GET_PATH = 5  # in seconds

colorama_init()
distance_across = math.sqrt(1 ** 2 + 1 ** 2)


class Node:
    def __init__(self, coordinates: tuple) -> None:
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.neighbours: List[Dict[str, Union['Node', float]]] = []

    def __lt__(self, other: Any) -> bool:
        return True

    def __le__(self, other: Any) -> bool:
        return True

    def get_position(self) -> Tuple[int, int]:
        return self.x, self.y

    def add_neighbour(self, node: 'Node', weight: float = None) -> None:
        if not weight:
            if self.x == node.x or self.y == node.y:
                weight = max(abs(self.x - node.x), abs(self.y - node.y))
            else:
                weight = math_helpers.calculate_distance(self.get_position(), node.get_position())
        self.neighbours.append({"node": node, "weight": weight})

    def remove_neighbour(self, node: 'Node') -> None:
        for i in range(len(self.neighbours)):
            if self.neighbours[i]["node"] == node:
                self.neighbours.pop(i)
                return

    def get_neighbour_nodes(self) -> List['Node']:
        return [neighbour["node"] for neighbour in self.neighbours]

    def get_heading(self, from_position: tuple) -> float:
        return math_helpers.calculate_direction(from_position, self.get_position())


class Ball:
    def __init__(self, pos: tuple, golden=False) -> None:
        self.x = pos[0]
        self.y = pos[1]
        self.golden = golden


class Obstacle:
    def __init__(self, path: list, points: list = None) -> None:
        self.path = path
        self.points = points


class Goal:
    def __init__(self, path: list, points: list = None, small=False) -> None:
        self.path = path
        self.small = small
        self.points = points


class NodeData:
    def __init__(self, node: Node, g: float, h: float, parent: Optional[Node]) -> None:
        self.node = node
        self.g = g
        self.h = h
        self.parent = parent

        self.f = self.g + self.h

    def __lt__(self, other: Any) -> bool:
        return True

    def __le__(self, other: Any) -> bool:
        return True


class Graph:
    def __init__(self, size_x: int = 500, size_y: int = 200) -> None:
        if size_x and size_y:
            range_x = range(size_x)
            range_y = range(size_y)
            self.nodes: np.ndarray[Node] = np.empty((size_y, size_x), dtype=Node)

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
        np.append(self.nodes, Node(coordinates))

    def get_node(self, pos: tuple) -> Optional[Node]:
        if 0 <= pos[0] < len(self.nodes[0]) and 0 <= pos[1] < len(self.nodes):
            return self.nodes[pos[1]][pos[0]]
        else:
            return None

    def get_nodes_in_path(self, path: list) -> list:
        node_plusminus = 1
        nodes_in_path = []
        for i in range(len(path)):
            pos = path[i]
            node = self.get_node(pos)
            if node:
                if DEBUG:
                    print(f"Adding {pos[0]}, {pos[1]}")
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
                            if DEBUG:
                                print(f"Adding {x}+-{node_plusminus}, {y}+-{node_plusminus}")
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
                            if DEBUG:
                                print(f"Adding {x}+-{node_plusminus}, {y}+-{node_plusminus}")
                            for j in range(-node_plusminus, node_plusminus + 1):
                                pos = (x + j, y + j)
                                if pos not in path:
                                    node = self.get_node(pos)
                                    if node:
                                        nodes_in_path.append(node)

        return list(set(nodes_in_path))

    def add_edge(self, node_1: Node, node_2: Node) -> None:
        if node_1 is not node_2 and node_1 and node_2:
            if node_2 not in node_1.get_neighbour_nodes():
                node_1.add_neighbour(node_2)
            if node_1 not in node_2.get_neighbour_nodes():
                node_2.add_neighbour(node_1)

    def remove_edge(self, node_1: Node, node_2: Node) -> None:
        if node_1 is not node_2 and node_1 and node_2:
            if node_2 in node_1.get_neighbour_nodes():
                node_1.remove_neighbour(node_2)
            if node_1 in node_2.get_neighbour_nodes():
                node_2.remove_neighbour(node_1)

    # Manhattan Distance heuristic for A*
    def h(self, start_node: Node, dst_node: Node) -> float:
        dx = abs(start_node.x - dst_node.x)
        dy = abs(start_node.y - dst_node.y)
        return dx + dy

    # Get path and cost using A*
    async def get_path(self, start_node: Node, dst_node: Node) -> List[NodeData]:
        if DEBUG:
            print(f"Getting path between {start_node.x}, {start_node.y} and {dst_node.x}, {dst_node.y}")

        # Initialize the start node data
        start_node_data = NodeData(start_node, 0, self.h(start_node, dst_node), None)

        # Create open and closed sets
        open_set: Set[Node] = {start_node}
        closed_set: Dict[Node, NodeData] = {}

        # A dictionary to store the best known cost from start to each node
        g_scores: Dict[Node, float] = {start_node: 0}

        # A dictionary to store the estimated total cost from start to each node
        f_scores: Dict[Node, float] = {start_node: start_node_data.f}

        # A priority queue to efficiently extract the node with the lowest f-score
        open_queue: List[Tuple[float, Node]] = [(f_scores[start_node], start_node)]

        # A counter to track the number of iterations
        iteration = 0

        # The current node
        current_node: Node
        while open_queue:
            iteration += 1
            if iteration % 500 == 0:
                if DEBUG:
                    print(f"Iteration: {iteration}")
                await asyncio.sleep(0)

            _, current_node = heapq.heappop(open_queue)

            closed_set[current_node] = NodeData(
                node=current_node,
                g=g_scores[current_node],
                h=self.h(current_node, dst_node),
                parent=closed_set.get(current_node, None).parent if closed_set.get(current_node, None) else None
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
                    heapq.heappush(open_queue, (f_scores[neighbour_node], neighbour_node))
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
    def __init__(self, bounds: dict):
        self.bounds = bounds
        self.balls: List[Ball] = []
        self.obstacles: List[Obstacle] = []
        self.small_goal: Optional[Goal] = None
        self.big_goal: Optional[Goal] = None
        self.robot_pos = (0, 0)
        self.robot_direction = 0.0
        self.path: list = []

        self.last_target_path: List[NodeData] = []
        self.last_target_node: Optional[Node] = None
        self.integral = 0.0
        self.previous_error = 0.0

        self.graph = Graph(bounds["x"], bounds["y"])

    def add_ball(self, ball: Ball) -> None:
        self.balls.append(ball)

    def clear_balls(self) -> None:
        self.balls = []

    def set_robot_pos(self, robot_pos: tuple) -> None:
        # Recalibrate the direction / angle
        self.robot_direction = math_helpers.calculate_direction(robot_pos, self.robot_pos)
        # Update position
        self.robot_pos = robot_pos

    async def calculate_path(self) -> None:
        # if DEBUG:
        #     print("Calculating path")
        if not self.balls:
            if DEBUG:
                print("No balls, returning empty path")
            self.path = []
            return

        # Get every ball that's not golden
        balls_to_catch = [ball for ball in self.balls if not ball.golden]
        # If no balls that aren't golden
        if not balls_to_catch:
            if DEBUG:
                print("Only the golden ball is left, fetching it")
            # Include the golden ball
            balls_to_catch = self.balls

        # For every ball calculate the path, then choose the best path
        if DEBUG:
            print("Calculating path for every ball")
        paths = []
        if balls_to_catch:
            robot_node = self.graph.get_node(self.robot_pos)
            ball_nodes = [self.graph.get_node((ball.x, ball.y)) for ball in balls_to_catch]
            tasks = [self.graph.get_path(start_node=robot_node, dst_node=ball_node) for ball_node in ball_nodes]
            done, pending = await asyncio.wait(tasks, timeout=TIMEOUT_GET_PATH, return_when=asyncio.ALL_COMPLETED)
            if DEBUG:
                print(f"Done calculating paths: {len(done)} done, {len(pending)} timed out")
            [task.cancel() for task in pending]
            for task in done:
                try:
                    paths.append(await task)
                except TimeoutError:
                    pass

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
                if DEBUG:
                    print(f"Best path found with weight {min_path['f']}")
                    # Log the path found
                    # for i in range(len(min_path["path"])):
                    #     node = min_path["path"][i].node
                    #     end = " -> " if i < len(min_path['path']) - 1 else "\n"
                    #     if DEBUG:
                    #         print(f"({node.x}, {node.y})", end=end)
                self.path = min_path["path"]
            else:
                if DEBUG:
                    print("Found no path!")
                self.path = []
        else:
            print("Got no paths, returning empty")
            self.path = []

    def clear_obstacles(self) -> None:
        for obstacle in self.obstacles:
            for node in obstacle.path:
                for x in range(-1, 1 + 1):
                    for y in range(-1, 1 + 1):
                        neighbour = self.graph.get_node((node.x + x, node.y + y))
                        self.graph.add_edge(node, neighbour)
        self.obstacles = []

    def add_obstacle(self, obstacle: Obstacle) -> None:
        for node in obstacle.path:
            # print(f"Removing node edges for node at {node.x}, {node.y}")
            for x in range(-1, 1 + 1):
                for y in range(-1, 1 + 1):
                    neighbour = self.graph.get_node((node.x + x, node.y + y))
                    self.graph.remove_edge(node, neighbour)
            if node.neighbours and DEBUG:
                print(
                    f"Failed to remove all neighbours for obstacle node at {node.x}, {node.y}")
        self.obstacles.append(obstacle)

    def add_goal(self, goal: Goal) -> None:
        if goal.small:
            self.small_goal = goal
        else:
            self.big_goal = goal

    def draw(self, with_path: bool = False) -> None:
        if with_path:
            self.graph.draw(robot_pos=self.robot_pos,
                            balls=self.balls, path=self.path)
        else:
            self.graph.draw(robot_pos=self.robot_pos, balls=self.balls)

    async def plot(self):
        await self.calculate_path()
        points_in_path = [[], []]
        for nodeData in self.path:
            points_in_path[0].append(nodeData.node.x)
            points_in_path[1].append(nodeData.node.y)

        plt.figure()
        plt.scatter(points_in_path[0], points_in_path[1])
        plt.xticks(np.arange(min(points_in_path[0]), max(points_in_path[0]) + 1, 1))
        plt.yticks(np.arange(min(points_in_path[1]), max(points_in_path[1]) + 1, 1))
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
                current_from_node.node.get_position(), previous_node.node.get_position(), next_node.node.get_position()
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


async def check_new_path(path_queue: multiprocessing.JoinableQueue) -> bool:
    """
    Checks if the path in the track is the same as last, and if it is, updates the current path. Also updates if it isn't.

    :param path_queue: The queue to send the current path to the AI.
    :return: True if we should drive, else False.
    """
    track = TRACK_GLOBAL
    if not track:
        return False
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
        if not is_target_different(track, track.last_target_path[0].node, track.graph.get_node(track.robot_pos)):
            if DEBUG:
                print("Robot reached target, popping")
            track.last_target_path.pop(0)
            track.last_target_node = track.graph.get_node(track.robot_pos)
        # If we passed the target, pop
        has_passed_result = math_helpers.has_passed_target(track.last_target_path[0].node.get_position(),
                                                           track.graph.get_node(track.robot_pos).get_position(),
                                                           track.last_target_node.get_position())
        if has_passed_result:
            if DEBUG:
                print("Robot passed target, popping")
            track.last_target_path.pop(0)
            track.last_target_node = track.graph.get_node(track.robot_pos)
        elif has_passed_result is None:
            if DEBUG:
                print("Robot went completely wrong way?!?? Removing path!")
            track.last_target_node = None
            track.last_target_path = None
            # if not path_queue.full():
            #     try:
            #         path_queue.put([])
            #     except:
            #         pass

        if DEBUG:
            print(
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
    if DEBUG:
        print("New target, making new path")

    if track.path[0].node.x == 0 and track.path[0].node.y == 0:
        # if DEBUG:
        #     print("Robot pos is 0,0, not making new path")
        return False

    track.integral = 0.0
    track.previous_error = 0.0

    # "Summarize" path into good points for targets
    track.last_target_path = await simplify_path(track.path)
    if DEBUG:
        print(
            f"Current target optimized path: {[(nodedata.node.x, nodedata.node.y) for nodedata in track.last_target_path]}")

    # Adjust and go to
    if track.last_target_path:
        full_path = [nodedata.node.get_position() for nodedata in track.last_target_path]

        if not path_queue.full():
            try:
                path_queue.put({"path": full_path if full_path else [], "obstacles": [obstacle.points for obstacle in
                                                                                      track.obstacles] if track.obstacles else [],
                                "small_goal": track.small_goal.points if track.small_goal else [],
                                "big_goal": track.big_goal.points if track.big_goal else []})
            except:
                pass
        track.last_target_node = track.last_target_path[-1].node
        return True


def is_target_different(track: Track, target_node: Node, other_node: Node) -> bool:
    # Define a threshold for difference based on your requirements
    # Assume a difference of 1.0 cm is significant, we use pixels though, so it depends on the distance and camera
    position_threshold = 30.0

    # if DEBUG:
    #     print(f"Checking if target is different between {target_node.get_position()} and {other_node.get_position()}")

    # Calculate the position difference
    position_diff = math_helpers.calculate_distance(target_node.get_position(), other_node.get_position())
    # Calculate the direction difference
    direction_diff = abs(target_node.get_heading(track.robot_pos) - other_node.get_heading(track.robot_pos))

    # Check if target is significantly different from the last target
    if position_diff > position_threshold or direction_diff > math.pi / 4:
        # if DEBUG:
        #     print(
        #         f"Different target: posdiff = {position_diff} > {position_threshold} | headdiff = {direction_diff} > {math.pi / 4}")
        return True

    return False


def setup_debug() -> None:
    bounds = {"x": 15, "y": 20}
    track = setup(bounds)

    track.set_robot_pos((9, 18))
    track.add_ball(Ball((8, 0)))

    obstacle_path = [(6, 10), (7, 11), (8, 10), (9, 9), (10, 10), (11, 11), (12, 10), (11, 9), (10, 8), (11, 7),
                     (12, 6), (11, 5), (10, 6), (9, 7), (8, 6), (7, 5), (6, 6), (7, 7), (8, 8), (8, 9), (7, 6), (8, 7),
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


def setup(bounds: dict) -> Track:
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
