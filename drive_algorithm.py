import math
import heapq
from colorama import init as colorama_init
from colorama import Fore
from colorama import Style

colorama_init()

distance_across = math.sqrt(1**2+1**2)

class Node:
    def __init__(self, coordinates: tuple):
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.neighbours = []
    
    def add_neighbour(self, node, weight=None):
        if not weight:
            if self.x == node.x or self.y == node.y:
                weight = 1
            else:
                weight = distance_across
        self.neighbours.append({"node": node, "weight": weight})
    
    def remove_neighbour(self, node):
        for i in range(len(self.neighbours)):
            if self.neighbours[i]["node"] == node:
                self.neighbours.pop(i)
                return
    
    def get_neighbour_nodes(self):
        return [neighbour["node"] for neighbour in self.neighbours]

class NodeData:
    def __init__(self, node, g, h, parent):
        self.node = node
        self.g = g
        self.h = h
        self.parent = parent
        
        self.f = self.g + self.h
    
    def __lt__(self, other):
        return True
    
    def __le__(self, other):
        return True

class AStar:
    def __init__(self, graph):
        self.graph = graph
    
    def set_from(self, from_node):
        self.from_node = from_node

    def set_target(self, target_node):
        self.target_node = target_node

    def calculate(self):
        if not self.graph or not self.from_node or not self.target_node:
            return []
        

class Graph:
    def __init__(self, size_x=500, size_y=200):
        self.nodes = []

        if size_x and size_y:
            self.nodes = [[Node((x, y)) for x in range(size_x)] for y in range(size_y)]

            for y, row in enumerate(self.nodes):
                for x, node in enumerate(row):
                    if x-1 >= 0:
                        node.add_neighbour(row[x-1], 1)
                        if y-1 >= 0:
                            node.add_neighbour(self.nodes[y-1][x-1], distance_across)
                        if y+1 < len(self.nodes):
                            node.add_neighbour(self.nodes[y+1][x-1], distance_across)
                    if x+1 < len(row):
                        node.add_neighbour(row[x+1], 1)
                        if y-1 >= 0:
                            node.add_neighbour(self.nodes[y-1][x+1], distance_across)
                        if y+1 < len(self.nodes):
                            node.add_neighbour(self.nodes[y+1][x+1], distance_across)
                    if y-1 >= 0:
                        node.add_neighbour(self.nodes[y-1][x], 1)
                    if y+1 < len(self.nodes):
                        node.add_neighbour(self.nodes[y+1][x], 1)


    def add_node(self, coordinates: tuple):
        self.nodes.append(Node(coordinates))
    
    def get_node(self, pos: tuple):
        if pos[0] >= 0 and pos[1] >= 0 and pos[1] < len(self.nodes) and pos[0] < len(self.nodes[0]):
            return self.nodes[pos[1]][pos[0]]
        else:
            return None
    
    def get_nodes_in_path(self, path: list):
        nodes_in_path = []
        for pos in path:
            node = self.get_node(pos)
            if node:
                nodes_in_path.append(node)
        return nodes_in_path
    
    def add_edge(self, node_1, node_2):
        if node_1 is not node_2 and node_1 and node_2:
            if node_2 not in node_1.get_neighbour_nodes():
                node_1.add_neighbour(node_2)
            if node_1 not in node_2.get_neighbour_nodes():
                node_2.add_neighbour(node_1)
    
    def remove_edge(self, node_1, node_2):
        if node_1 is not node_2 and node_1 and node_2:
            if node_2 in node_1.get_neighbour_nodes():
                node_1.remove_neighbour(node_2)
            if node_1 in node_2.get_neighbour_nodes():
                node_2.remove_neighbour(node_1)
    
    # Manhattan Distance heuristic for A*
    def h(self, start_node, dst_node):
        return abs(start_node.x - dst_node.x) + abs(start_node.y - dst_node.y)
    
    # Get path and cost using A*
    def get_path(self, start_node, dst_node):
        start_node_data = NodeData(start_node, 0, self.h(start_node, dst_node), None)
        open_list = [(start_node_data.f, start_node_data)]
        closed_list = []

        heapq.heapify(open_list)

        current_node = None
        while open_list:
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
                                heapq.heappush(open_list, (neighbour_data.f, neighbour_data))
                        break
                
                if not closed_list_check:
                    open_list_check = False
                    open_list_list = list(open_list)
                    for i in range(len(open_list_list)):
                        if open_list_list[i][1].node == neighbour_data.node:
                            open_list_check = True
                            if neighbour_data.g < open_list_list[i][1].g:
                                open_list_list[i] = (neighbour_data.f, neighbour_data)
                                open_list = open_list_list
                                heapq.heapify(open_list)
                            break
                
                    if not open_list_check:
                        heapq.heappush(open_list, (neighbour_data.f, neighbour_data))

            
            closed_list.append(current_node)
        
        # print(closed_list)
        # for elem in closed_list:
        #     print(f"({elem[1].node.x}, {elem[1].node.y}) - f: {elem[1].f} g: {elem[1].g} h: {elem[1].h}")
        
        # DEBUG
        node = closed_list[-1][1]
        print(f"End walkthrough, f: {node.f} g: {node.g} h: {node.h}")
        while node:
            print(f"({node.node.x}, {node.node.y})", end=" ")
            node = node.parent
        print("")

        return closed_list
    
    def draw(self, robot_pos: tuple = None, balls=[]):
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
                for neighbour in node.get_neighbour_nodes():
                    if node.x == neighbour.x:
                        if node.y+1 == neighbour.y:
                            neighbours["down"] = 1
                    elif node.x+1 == neighbour.x:
                        if node.y == neighbour.y:
                            neighbours["right"] = 1
                        elif node.y+1 == neighbour.y:
                            neighbours["down_right"] += 1
                down_node = self.get_node((node.x, node.y+1))
                if down_node:
                    # check for cross the other way
                    for neighbour_down in down_node.get_neighbour_nodes():
                        if down_node.y == neighbour_down.y+1 and down_node.x == neighbour_down.x-1:
                            neighbours["down_right"] += 2
                            break
                
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
            print(nodes_string)
            print(conections_string)


class Ball:
    def __init__(self, pos: tuple):
        self.x = pos[0]
        self.y = pos[1]

class Obstacle:
    def __init__(self, path):
        self.path = path

class Track:
    def __init__(self, bounds):
        self.bounds = bounds
        self.balls = []
        self.obstacles = []
        self.robot_pos = (0, 0)
        self.path = []

        self.graph = Graph(bounds["x"], bounds["y"])
    
    def add_ball(self, ball):
        self.balls.append(ball)
    
    def clear_balls(self):
        self.balls = []
    
    def set_robot_pos(self, robot_pos: tuple):
        self.robot_pos = robot_pos
    
    def calculate_path(self):
        # For every ball calculate the path, then choose the best path
        paths = []
        for ball in self.balls:
            robot_node = self.graph.get_node(self.robot_pos)
            ball_node = self.graph.get_node((ball.x, ball.y))
            paths.append(self.graph.get_path(start_node = robot_node, dst_node = ball_node))
        
        if paths:
            min_path = {"path": None, "f": None}
            for path in paths:
                if path:
                    if not min_path["path"] or path[-1][0] < min_path["f"]:
                        min_path["path"] = path
                        min_path["f"] = path[-1][0]
            if min_path["path"]:
                print(f"Best path found with weight {min_path['f']}")
                self.path = min_path["path"]
            else:
                self.path = []
        else:
            self.path = []
    
    def clear_obstacles(self):
        for obstacle in self.obstacles:
            for node in obstacle.path:
                for x in range(-1, 1+1):
                    for y in range(-1, 1+1):
                        neighbour = self.graph.get_node((node.x+x, node.y+y))
                        self.graph.add_edge(node, neighbour)
        self.obstacles = []
    
    def add_obstacle(self, obstacle):
        for node in obstacle.path:
            #print(f"Removing node edges for node at {node.x}, {node.y}")
            for x in range(-1, 1+1):
                for y in range(-1, 1+1):
                    neighbour = self.graph.get_node((node.x+x, node.y+y))
                    self.graph.remove_edge(node, neighbour)
            if node.neighbours:
                print(f"Failed to remove all neighbours for obstacle node at {node.x}, {node.y}")
        self.obstacles.append(obstacle)


    def draw(self):
        self.graph.draw(robot_pos = self.robot_pos, balls = self.balls)
    

if __name__ == "__main__":
    bounds = {"x": 20, "y": 8}
    track = Track(bounds)

    track.set_robot_pos((15, 5))
    track.add_ball(Ball((1, 2)))
    track.add_ball(Ball((8, 4)))

    obstacle_path = [(4, 3), (4, 4), (4, 5), (3, 4), (5, 4)]
    obstacle = Obstacle(track.graph.get_nodes_in_path(obstacle_path))
    track.add_obstacle(obstacle)

    wall_path = []
    wall_path.extend([(x, y) for x in [0, bounds["x"]-1] for y in range(0, bounds["y"])])
    wall_path.extend([(x, y) for x in range(0, bounds["x"]) for y in [0, bounds["y"]-1]])
    wall_obstacle = Obstacle(track.graph.get_nodes_in_path(wall_path))
    track.add_obstacle(wall_obstacle)

    track.calculate_path()

    track.draw()
