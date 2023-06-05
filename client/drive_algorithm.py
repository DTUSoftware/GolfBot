import os
import math
import heapq
from typing import Any, Optional, List, Tuple

import numpy as np
from colorama import init as colorama_init
from colorama import Fore
from colorama import Style
import matplotlib.pyplot as plt

DEBUG = "true" in os.environ.get('DEBUG', "True").lower()

colorama_init()
distance_across = math.sqrt(1 ** 2 + 1 ** 2)


class Node:
    def __init__(self, coordinates: tuple) -> None:
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.neighbours = []

    def add_neighbour(self, node: 'Node', weight: float = None) -> None:
        if not weight:
            if self.x == node.x or self.y == node.y:
                weight = max(abs(self.x - node.x), abs(self.y - node.y))
            else:
                weight = math.sqrt(abs(self.x - node.x) ** 2 + abs(self.y - node.y) ** 2)
        self.neighbours.append({"node": node, "weight": weight})

    def remove_neighbour(self, node: 'Node') -> None:
        for i in range(len(self.neighbours)):
            if self.neighbours[i]["node"] == node:
                self.neighbours.pop(i)
                return

    def get_neighbour_nodes(self) -> List['Node']:
        return [neighbour["node"] for neighbour in self.neighbours]

    def get_heading(self, from_position: tuple) -> float:
        return math.atan2(self.y - from_position[1], self.x - from_position[0])


class Ball:
    def __init__(self, pos: tuple, golden=False) -> None:
        self.x = pos[0]
        self.y = pos[1]
        self.golden = golden


class Obstacle:
    def __init__(self, path: list) -> None:
        self.path = path


class Goal:
    def __init__(self, path: list, small=False) -> None:
        self.path = path
        self.small = small


class NodeData:
    def __init__(self, node: Node, g: float, h: float, parent: Any) -> None:
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
        self.nodes: List[Node] = []

        if size_x and size_y:
            self.nodes = [[Node((x, y)) for x in range(size_x)]
                          for y in range(size_y)]

            for y, row in enumerate(self.nodes):
                for x, node in enumerate(row):
                    if x - 1 >= 0:
                        node.add_neighbour(row[x - 1], 1)
                        if y - 1 >= 0:
                            node.add_neighbour(
                                self.nodes[y - 1][x - 1], distance_across)
                        if y + 1 < len(self.nodes):
                            node.add_neighbour(
                                self.nodes[y + 1][x - 1], distance_across)
                    if x + 1 < len(row):
                        node.add_neighbour(row[x + 1], 1)
                        if y - 1 >= 0:
                            node.add_neighbour(
                                self.nodes[y - 1][x + 1], distance_across)
                        if y + 1 < len(self.nodes):
                            node.add_neighbour(
                                self.nodes[y + 1][x + 1], distance_across)
                    if y - 1 >= 0:
                        node.add_neighbour(self.nodes[y - 1][x], 1)
                    if y + 1 < len(self.nodes):
                        node.add_neighbour(self.nodes[y + 1][x], 1)

    def add_node(self, coordinates: tuple) -> None:
        self.nodes.append(Node(coordinates))

    def get_node(self, pos: tuple) -> Any:
        if 0 <= pos[0] < len(self.nodes[0]) and 0 <= pos[1] < len(self.nodes):
            return self.nodes[pos[1]][pos[0]]
        else:
            return None

    def get_nodes_in_path(self, path: list) -> list:
        nodes_in_path = []
        for i in range(len(path)):
            pos = path[i]
            node = self.get_node(pos)
            if node:
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
                            pos = (x, y)
                            if pos not in path:
                                node = self.get_node(pos)
                                if node:
                                    nodes_in_path.append(node)
                    elif abs(y1 - y2) > 1:
                        slope = (x1 - x2) / (y1 - y2)
                        x_intercept = (y1 * x2 - y2 * x1) / (y1 - y2)
                        y_min = min(y1, y2)
                        y_max = max(y1, y2)
                        for y in range(int(y_min + 1), int(y_max)):
                            x = int(slope * float(y) + x_intercept)
                            pos = (x, y)
                            if pos not in path:
                                node = self.get_node(pos)
                                if node:
                                    nodes_in_path.append(node)

        return nodes_in_path

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
        return abs(start_node.x - dst_node.x) + abs(start_node.y - dst_node.y)

    # Get path and cost using A*
    def get_path(self, start_node: Node, dst_node: Node) -> list:
        # if DEBUG:
        #     print(f"Getting path between {start_node.x}, {start_node.y} and {dst_node.x}, {dst_node.y}")
        start_node_data = NodeData(
            start_node, 0, self.h(start_node, dst_node), None)
        open_list: List[Tuple[float, NodeData]] = [
            (start_node_data.f, start_node_data)]
        closed_list: List[Tuple[float, NodeData]] = []

        heapq.heapify(open_list)

        # TODO: fix this hack with i, shouldn't be needed
        i = 0
        current_node = None
        while open_list:
            i += 1
            # if DEBUG:
            #     print(i)
            if i > 2000:
                if DEBUG:
                    print("While look fucky wucky")
                break
            current_node = heapq.heappop(open_list)
            if current_node[1].node is dst_node:
                closed_list.append(current_node)
                break

            for neighbour in current_node[1].node.neighbours:
                neighbour_data = NodeData(
                    node=neighbour["node"],
                    g=current_node[1].g + neighbour["weight"],
                    h=self.h(neighbour["node"], dst_node),
                    parent=current_node[1]
                )

                closed_list_check = False
                for i in range(len(closed_list)):
                    if closed_list[i][1].node == neighbour_data.node:
                        closed_list_check = True
                        if neighbour_data.g < closed_list[i][1].g:
                            closed_list[i] = (neighbour_data.f, neighbour_data)
                            found = False
                            for node_elem in list(open_list):
                                if node_elem[1].node == neighbour_data.node:
                                    found = True
                                    break
                            if not found:
                                heapq.heappush(
                                    open_list, (neighbour_data.f, neighbour_data))
                        break

                if not closed_list_check:
                    open_list_check = False
                    open_list_list = list(open_list)
                    for i in range(len(open_list_list)):
                        if open_list_list[i][1].node == neighbour_data.node:
                            open_list_check = True
                            if neighbour_data.g < open_list_list[i][1].g:
                                open_list_list[i] = (
                                    neighbour_data.f, neighbour_data)
                                open_list = open_list_list
                                heapq.heapify(open_list)
                            break

                    if not open_list_check:
                        heapq.heappush(
                            open_list, (neighbour_data.f, neighbour_data))

            closed_list.append(current_node)

        # print(closed_list)
        # for elem in closed_list:
        #     print(f"({elem[1].node.x}, {elem[1].node.y}) - f: {elem[1].f} g: {elem[1].g} h: {elem[1].h}")

        final_list: list = []
        node = closed_list[-1][1]
        # print(f"End walkthrough, f: {node.f} g: {node.g} h: {node.h}")
        while node:
            # if DEBUG:
            #     print("NODE WHILE LOOP")
            # print(f"({node.node.x}, {node.node.y})", end=" ")
            final_list.insert(0, node)
            node = node.parent
        # print("")

        return final_list

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

        self.graph = Graph(bounds["x"], bounds["y"])

    def add_ball(self, ball: Ball) -> None:
        self.balls.append(ball)

    def clear_balls(self) -> None:
        self.balls = []

    def set_robot_pos(self, robot_pos: tuple) -> None:
        # Recalibrate the direction / angle
        self.robot_direction = math.atan2(self.robot_pos[1] - robot_pos[1], self.robot_pos[0] - robot_pos[0])
        # Update position
        self.robot_pos = robot_pos

    def calculate_path(self) -> None:
        # if DEBUG:
        #     print("Calculating path")
        if not self.balls:
            # if DEBUG:
            #     print("No balls, returning empty path")
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
        # if DEBUG:
        #     print("Calculating path for every ball")
        paths: List[list] = []
        for ball in balls_to_catch:
            # if DEBUG:
            #     print(f"Trying for ball: ({ball.x}, {ball.y})")
            robot_node = self.graph.get_node(self.robot_pos)
            ball_node = self.graph.get_node((ball.x, ball.y))
            paths.append(self.graph.get_path(
                start_node=robot_node, dst_node=ball_node))
            # if DEBUG:
            #     print("Done finding path")

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
                for i in range(len(min_path["path"])):
                    node = min_path["path"][i].node
                    end = " -> " if i < len(min_path['path']) - 1 else "\n"
                    if DEBUG:
                        print(f"({node.x}, {node.y})", end=end)
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

    def plot(self):
        self.calculate_path()
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


def setup_debug() -> None:
    bounds = {"x": 15, "y": 18}
    track = setup(bounds)

    track.set_robot_pos((8, 7))
    track.add_ball(Ball((9, 11)))

    obstacle_path = [(4, 3), (4, 4), (4, 5), (3, 4), (5, 4)]
    obstacle = Obstacle(track.graph.get_nodes_in_path(obstacle_path))
    # track.add_obstacle(obstacle)
    # obstacle_path = [(1, 5), (3, 5), (4, 5), (5, 5), (6, 5), (7, 5), (8, 5), (9, 5), (10, 5), (11, 5), (12, 5), (13, 5), (14, 5)]
    # obstacle = Obstacle(track.graph.get_nodes_in_path(obstacle_path))
    # track.add_obstacle(obstacle)

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


if __name__ == "__main__":
    setup_debug()

    if TRACK_GLOBAL:
        track = TRACK_GLOBAL

        print("=========================\nDrawing with obstacles!\n=========================")
        track.draw(False)

        track.calculate_path()

        print("=========================\nDrawing driving path!\n=========================")
        track.draw(True)
        track.plot()
