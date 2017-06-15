import math
import random
import matplotlib.pyplot as plt
import numpy as np
from queue import PriorityQueue

from search import Search

def dist(pos1, pos2):
    return math.sqrt(((pos2.x - pos1.x) ** 2) + ((pos2.y - pos1.y) ** 2))

class AStar(Search):

    def __init__(self, field_dim, tag_radius, robot_radius):
        super(AStar, self).__init__(field_dim, tag_radius, robot_radius)
        self.graph = {}
        self.error_tolerance = tag_radius

    def reconstruct_path(self, came_from, current_node):
        path = [current_node.get_pos()]
        while current_node in came_from.keys():
            self.plot_line(current_node.x, came_from[current_node].x, current_node.y, came_from[current_node].y)
            current_node = came_from[current_node]
            path.append(current_node.get_pos())
        path.reverse()
        return path

    def plot_obstacle(self, x0, x1, y0, y1):
        for y in range(y0, y1):
            ys = np.ones((x1 - x0))
            ys = ys * y * -1
            self.plt.scatter(range(x0, x1), ys, color='r')

    def plot_line(self, x0, x1, y0, y1, color='b'):
        self.plt.plot([x0, x1], [-1 * y0, -1 * y1], color=color)

    def collision(self, node_pos, obstacle_pos_list, exclude_dist):
        for obstacle_pos in obstacle_pos_list:
            if (obstacle_pos.x - exclude_dist <= node_pos.x and node_pos.x <= obstacle_pos.x + exclude_dist) and \
               (obstacle_pos.y - exclude_dist <= node_pos.y and node_pos.y <= obstacle_pos.y + exclude_dist):
                return True
#            if dist(node_pos, obstacle_pos) <= exclude_dist:
#                return True
        return False

    def init_obstacles(self, obstacle_pos_list):
        pos_obj_list = []
        for pos in obstacle_pos_list:
            pos_obj_list.append(Position(pos))
        return pos_obj_list

    def init_graph(self, obstacle_pos_list, unit_length):
        exclude_dist = self.tag_radius + self.robot_radius
        x = 0
        y = 0
        while y <= self.field_dim[1]:
            x = 0
            while x <= self.field_dim[0]:
                pos = Position([x, y])
                if not self.collision(pos, obstacle_pos_list, exclude_dist):
                    self.graph[pos] = (Node(pos), set())
                    left_pos = Position([x - unit_length, y])
                    up_pos = Position([x, y - unit_length])
                    up_left_pos = Position([x - unit_length, y - unit_length])
                    up_right_pos = Position([x + unit_length, y - unit_length])
                    if left_pos in self.graph.keys():
                        self.graph[pos][1].add(self.graph[left_pos][0])
                        self.graph[left_pos][1].add(self.graph[pos][0])
                    if up_pos in self.graph.keys():
                        self.graph[pos][1].add(self.graph[up_pos][0])
                        self.graph[up_pos][1].add(self.graph[pos][0])
                    if up_left_pos in self.graph.keys():
                        self.graph[pos][1].add(self.graph[up_left_pos][0])
                        self.graph[up_left_pos][1].add(self.graph[pos][0])
                    if up_right_pos in self.graph.keys():
                        self.graph[pos][1].add(self.graph[up_right_pos][0])
                        self.graph[up_right_pos][1].add(self.graph[pos][0])
                x = x + unit_length
            y = y + unit_length
        return

    def init_plot(self, obstacle_pos, tag_radius, robot_radius):
        for obstacle in obstacle_pos:
            x0 = obstacle[0] - tag_radius - robot_radius
            x1 = obstacle[0] + tag_radius + robot_radius + 1
            y0 = obstacle[1] - tag_radius - robot_radius
            y1 = obstacle[1] + tag_radius + robot_radius + 1
            self.plot_obstacle(x0, x1, y0, y1)

# start_pos = [x, y], end_pos = [x, y], obstacle_pos = [[x0, y0], [x1, y1], ...], unit_length = d
    def get_path(self, start_pos, end_pos, obstacle_pos, unit_length):
        self.plt = plt
        start_pos = [(round(start_pos[0] / unit_length) * unit_length), (round(start_pos[1] / unit_length) * unit_length)]
        node_count = 1
        obstacle_pos_list = self.init_obstacles(obstacle_pos)
        self.init_graph(obstacle_pos_list, unit_length)
        self.init_plot(obstacle_pos, self.tag_radius, self.robot_radius)
        came_from = {}
        open_queue = PriorityQueue()
        open_set = set()
        closed_set = set()
        start_pos_obj = Position(start_pos)
        start_node = self.graph[start_pos_obj][0]
        start_node.cost_from_start = 0
        start_node.total_cost = start_node.get_heuristic(end_pos)
        open_queue.put((start_node.total_cost, start_node))
        open_set.add(start_node)
        while not open_queue.empty():
            current_node = open_queue.get()[1]
            if current_node.get_heuristic(end_pos) < self.error_tolerance:
                path = self.reconstruct_path(came_from, current_node)
                self.print_grid(start_pos, end_pos, obstacle_pos, self.field_dim, unit_length, self.tag_radius * 2, path)
                self.plt.show()
                return path
            open_set.remove(current_node)
            closed_set.add(current_node)
            neighbor_nodes = self.graph[current_node.pos][1]
            for neighbor_node in neighbor_nodes:
                if neighbor_node in closed_set:
                    continue
                tentative_cost_from_start = current_node.cost_from_start + dist(current_node.pos, neighbor_node.pos)
                if neighbor_node not in open_set:
                    node_count += 1
                    open_queue.put((tentative_cost_from_start + neighbor_node.get_heuristic(end_pos) + random.random(), neighbor_node))
                    open_set.add(neighbor_node)
                elif tentative_cost_from_start >= neighbor_node.cost_from_start:
                    continue
                came_from[neighbor_node] = current_node
                neighbor_node.cost_from_start = tentative_cost_from_start
                neighbor_node.total_cost = neighbor_node.cost_from_start + neighbor_node.get_heuristic(end_pos)
        return None

    def print_grid(self, start_pos, end_pos, obstacle_pos, field_dim, unit_length, tag_radius, path):
        map_grid = [['-' for x in range(0, int(field_dim[0] / unit_length))] for y in range(0, int(field_dim[1] / unit_length))]

        start_y = start_pos[1] - tag_radius
        while start_y <= start_pos[1] + tag_radius:
            start_x = start_pos[0] - tag_radius
            while start_x <= start_pos[0] + tag_radius:
                map_grid[int(start_y / unit_length)][int(start_x / unit_length)] = 'S'
                start_x += 1
            start_y += 1

        end_y = end_pos[1] - tag_radius
        while end_y <= end_pos[1] + tag_radius:
            end_x = end_pos[0] - tag_radius
            while end_x <= end_pos[0] + tag_radius:
                map_grid[int(end_y / unit_length)][int(end_x / unit_length)] = 'E'
                end_x += 1
            end_y += 1

        for obstacle in obstacle_pos:
            obstacle_y = obstacle[1] - tag_radius
            while obstacle_y <= obstacle[1] + tag_radius:
                obstacle_x = obstacle[0] - tag_radius
                while obstacle_x <= obstacle[0] + tag_radius:
                    map_grid[int(obstacle_y / unit_length)][int(obstacle_x / unit_length)] = 'O'
                    obstacle_x += 1
                obstacle_y += 1

        for y in range(0, int(field_dim[1] / unit_length)):
            for x in range(0, int(field_dim[0] / unit_length)):
                print(map_grid[y][x], end=' ')
            print('\n', end='')

        for pos in path:
            print(pos)
            map_grid[int(pos[1] / unit_length)][int(pos[0] / unit_length)] = 'X'

        for y in range(0, int(field_dim[1] / unit_length)):
            for x in range(0, int(field_dim[0] / unit_length)):
                print(map_grid[y][x], end=' ')
            print('\n', end='')

class Position(object):

    def __init__(self, pos):
        self.x = pos[0]
        self.y = pos[1]

    def __eq__(self, other):
        return (self.x == other.x and self.y == other.y)

    def __hash__(self):
        return (self.x * 53) + (self.y * 7)

    def get_pos(self):
        return [self.x, self.y]



class Node(object):

    def __init__(self, pos):
        self.pos = pos
        self.x = self.pos.x
        self.y = self.pos.y
        self.cost_from_start = float('inf')
        self.total_cost = float('inf')

    def __eq__(self, other):
        return (self.pos.x == other.pos.x and self.pos.y == other.pos.y)

    def __hash__(self):
        return (self.pos.x * 53) + (self.pos.y * 7)

    def get_heuristic(self, end_pos):
        return math.sqrt(((end_pos[0] - self.pos.x) ** 2) + ((end_pos[1] - self.pos.y) ** 2))

    def get_pos(self):
        return self.pos.get_pos()





# field_dim = [200, 200]
# tag_radius = 5
# robot_radius = 5
# unit_length = 5
#
# start_pos = [25, 25]
#
# obstacle_pos = []
# for x in range(0, 20):
#     new_obstacle = [int(random.randint(2 * tag_radius, field_dim[0] - 2 * tag_radius)), int(random.randint(2 * tag_radius, field_dim[1] - 2 * tag_radius))]
#     obstacle_pos.append(new_obstacle)
#
# end_pos = [int(random.randint(2 * tag_radius, field_dim[0] - 2 * tag_radius)), int(random.randint(2 * tag_radius, field_dim[1] - 2 * tag_radius))]
#
# map_grid = [['-' for x in range(0, int(field_dim[0] / unit_length))] for y in range(0, int(field_dim[1] / unit_length))]
#
# start_y = start_pos[1] - tag_radius
# while start_y <= start_pos[1] + tag_radius:
#     start_x = start_pos[0] - tag_radius
#     while start_x <= start_pos[0] + tag_radius:
#         map_grid[int(start_y / unit_length)][int(start_x / unit_length)] = 'S'
#         start_x += 1
#     start_y += 1
#
# end_y = end_pos[1] - tag_radius
# while end_y <= end_pos[1] + tag_radius:
#     end_x = end_pos[0] - tag_radius
#     while end_x <= end_pos[0] + tag_radius:
#         map_grid[int(end_y / unit_length)][int(end_x / unit_length)] = 'E'
#         end_x += 1
#     end_y += 1
#
# for obstacle in obstacle_pos:
#     obstacle_y = obstacle[1] - tag_radius
#     while obstacle_y <= obstacle[1] + tag_radius:
#         obstacle_x = obstacle[0] - tag_radius
#         while obstacle_x <= obstacle[0] + tag_radius:
#             map_grid[int(obstacle_y / unit_length)][int(obstacle_x / unit_length)] = 'O'
#             obstacle_x += 1
#         obstacle_y += 1
#
# for y in range(0, int(field_dim[1] / unit_length)):
#     for x in range(0, int(field_dim[0] / unit_length)):
#         print(map_grid[y][x], end=' ')
#     print('\n', end='')
#
# a = AStar(field_dim, tag_radius, robot_radius)
# path = a.get_path(start_pos, end_pos, obstacle_pos, unit_length)
# for pos in path:
#     print(pos)
#     map_grid[int(pos[1] / unit_length)][int(pos[0] / unit_length)] = 'X'
#
# for y in range(0, int(field_dim[1] / unit_length)):
#     for x in range(0, int(field_dim[0] / unit_length)):
#         print(map_grid[y][x], end=' ')
#     print('\n', end='')
