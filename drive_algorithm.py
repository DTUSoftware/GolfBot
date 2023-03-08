from colorama import init as colorama_init
from colorama import Fore
from colorama import Style

colorama_init()

class Node:
    def __init__(self, coordinates: tuple):
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.neighbours = []
    
    def add_neighbour(self, node):
        self.neighbours.append(node)

class Graph:
    def __init__(self, size_x=500, size_y=200):
        self.nodes = []

        if size_x and size_y:
            self.nodes = [[Node((x, y)) for x in range(size_x)] for y in range(size_y)]

            for y, row in enumerate(self.nodes):
                for x, node in enumerate(row):
                    if x-1 >= 0:
                        node.add_neighbour(row[x-1])
                        if y-1 >= 0:
                            node.add_neighbour(self.nodes[y-1][x-1])
                        if y+1 < len(self.nodes):
                            node.add_neighbour(self.nodes[y+1][x-1])
                    if x+1 < len(row):
                        node.add_neighbour(row[x+1])
                        if y-1 >= 0:
                            node.add_neighbour(self.nodes[y-1][x+1])
                        if y+1 < len(self.nodes):
                            node.add_neighbour(self.nodes[y+1][x+1])
                    if y-1 >= 0:
                        node.add_neighbour(self.nodes[y-1][x])
                    if y+1 < len(self.nodes):
                        node.add_neighbour(self.nodes[y+1][x])


    def add_node(self, coordinates: tuple):
        self.nodes.append(Node(coordinates))
    
    def get_node(self, pos: tuple):
        if pos[0] >= 0 and pos[1] >= 0 and pos[1] < len(self.nodes) and pos[0] < len(self.nodes[0]):
            return self.nodes[pos[1]][pos[0]]
        else:
            return None
    
    def add_edge(self, node_1, node_2):
        if node_1 is not node_2:
            if node_2 not in node_1.neighbours:
                node_1.neighbours.append(node_2)
            if node_1 not in node_2.neighbours:
                node_2.neighbours.apend(node_1)
    
    def remove_edge(self, node_1, node_2):
        if node_1 is not node_2:
            if node_2 in node_1.neighbours:
                node_1.neighbours.remove(node_2)
            if node_1 in node_2.neighbours:
                node_2.neighbours.remove(node_1)
    
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
                for neighbour in node.neighbours:
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
                    for neighbour_down in down_node.neighbours:
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

        self.graph = Graph(bounds["x"], bounds["y"])
    
    def add_ball(self, ball):
        self.balls.append(ball)
    
    def clear_balls(self):
        self.balls = []
    
    def set_robot_pos(self, robot_pos: tuple):
        self.robot_pos = robot_pos
    
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
            for x in range(-1, 1+1):
                for y in range(-1, 1+1):
                    neighbour = self.graph.get_node((node.x+x, node.y+y))
                    self.graph.remove_edge(node, neighbour)
        self.obstacles.append(obstacle)


    def draw(self):
        self.graph.draw(robot_pos = self.robot_pos, balls = self.balls)
    

if __name__ == "__main__":
    bounds = {"x": 20, "y": 8}
    track = Track(bounds)

    track.set_robot_pos((5, 5))
    track.add_ball(Ball((1, 2)))
    track.add_ball(Ball((5, 4)))

    obstacle_path = [track.graph.get_node((4, 1)), track.graph.get_node((4, 2)), track.graph.get_node((4, 3)), track.graph.get_node((3, 2)), track.graph.get_node((5, 2))]
    obstacle = Obstacle(obstacle_path)
    track.add_obstacle(obstacle)

    track.draw()
