from random import randint
import numpy as np
import matplotlib.pyplot as plt

from rrt_node import RRTNode
from search import Search

class RRTSearch(Search):

    def __init__(self, field_dim, tag_radius, robot_radius):
        super(RRTSearch, self).__init__(field_dim, tag_radius, robot_radius)
        self.EMPTY = 0
        self.OCCUPIED = 1
        self.START = 2
        self.END = 3
        self.NODE = 4
        self.PATH = 5

    def print_visualization(self):
        output = ''
        for y in range(self.visualization.shape[0]):
            for x in range(self.visualization.shape[1]):
                if (self.visualization[y,x] == self.EMPTY):
                    output += 'O '
                elif (self.visualization[y,x] == self.OCCUPIED):
                    output += 'X '
                elif (self.visualization[y,x] == self.START):
                    output += 'S '
                elif (self.visualization[y,x] == self.END):
                    output += 'E '
                elif (self.visualization[y,x] == self.NODE):
                    output += 'N '
                elif (self.visualization[y,x] == self.PATH):
                    output += '  '
            output += '\n'
        print(output)

    def plot_obstacle(self, x0, x1, y0, y1):
        for y in range(y0, y1):
            ys = np.ones((x1 - x0))
            ys = ys * y * -1
            self.plt.scatter(range(x0, x1), ys, color='r')

    def plot_line(self, x0, x1, y0, y1, color='b'):
        self.plt.plot([x0, x1], [-1 * y0, -1 * y1], color=color)

    def plot_path_line(self, x0, x1, y0, y1):
        self.plot_line(x0, x1, y0, y1, color='g')

    def get_path(self, start_pos, end_pos, obstacle_pos, unit_length):
        self.plt = plt
        self.create_occupancy_grid(self.field_dim, obstacle_pos, self.tag_radius, self.robot_radius)

        self.visualization = np.copy(self.occ_grid)
        self.visualization[int(start_pos[1]), int(start_pos[0])] = self.START
        self.visualization[int(end_pos[1]), int(end_pos[0])] = self.END

        self.nodes = []
        q_new = RRTNode([round(start_pos[0]), round(start_pos[1])])
        q_goal = RRTNode([round(end_pos[0]), round(end_pos[1])])
        self.nodes.append(q_new)

        while(q_new.get_dist(q_goal) > unit_length):
            q_backup = q_new
            q_rand = self.get_random_node()
            q_near = self.get_nearest_node(q_rand)
            if (q_near.get_dist(q_rand) <= unit_length):
                q_new = q_rand
            else:
                unit_vector = self.get_unit_vector(unit_length, q_near, q_rand)
                pos = [round(q_near.pos[0] + unit_vector[0]), round(q_near.pos[1] + unit_vector[1])]
                q_new = RRTNode(pos)
            if (self.visualization[int(q_new.pos[1]), int(q_new.pos[0])] != self.EMPTY or self.occupied(q_near, q_new)):
                q_new = q_backup
                continue
            self.visualization[int(q_new.pos[1]), int(q_new.pos[0])] = self.NODE
            self.plot_line(q_near.pos[0], q_new.pos[0], q_near.pos[1], q_new.pos[1])
            q_near.add_child(q_new)
            q_new.set_parent(q_near)
            self.nodes.append(q_new)
        q_new.add_child(q_goal)
        q_goal.set_parent(q_new)
        #self.print_visualization()
        path = self.create_path(q_goal)
        #self.print_visualization()
        self.plt.show()
        return path

    def create_occupancy_grid(self, field_dim, obstacle_pos, tag_radius, robot_radius):
        self.occ_grid = np.zeros((field_dim[1], field_dim[0]))

        # x0 = 0
        # x1 = robot_radius
        # y0 = 0
        # y1 = field_dim[1]
        # self.occ_grid[y0:y1, x0:x1] = 1
        # self.plot_obstacle(x0, x1, y0, y1)
        #
        # x0 = field_dim[0] - robot_radius
        # x1 = field_dim[0]
        # self.occ_grid[y0:y1, x0:x1] = 1
        # self.plot_obstacle(x0, x1, y0, y1)
        #
        # x0 = 0
        # x1 = field_dim[0]
        # y0 = 0
        # y1 = robot_radius
        # self.occ_grid[y0:y1, x0:x1] = 1
        # self.plot_obstacle(x0, x1, y0, y1)
        #
        # y0 = field_dim[1] - robot_radius
        # y1 = field_dim[1]
        # self.occ_grid[y0:y1, x0:x1] = 1
        # self.plot_obstacle(x0, x1, y0, y1)

        for pos in obstacle_pos:
            x0 = pos[0] - tag_radius - robot_radius
            x1 = pos[0] + tag_radius + robot_radius + 1
            y0 = pos[1] - tag_radius - robot_radius
            y1 = pos[1] + tag_radius + robot_radius + 1
            self.occ_grid[y0:y1, x0:x1] = 1
            self.plot_obstacle(x0, x1, y0, y1)

    def occupied(self, q_near, q_new):
        x0 = q_near.pos[0]
        y0 = q_near.pos[1]
        x1 = q_new.pos[0]
        y1 = q_new.pos[1]
        deltax = x1 - x0
        deltay = y1 - y0

        if (abs(deltay) < abs(deltax)):
            a_first = False
            a0 = x0
            a1 = x1
            b0 = y0
            b1 = y1
            delta_a = deltax
            delta_b = deltay
            deltaerr = abs(deltay / deltax)
        else:
            a_first = True
            a0 = y0
            a1 = y1
            b0 = x0
            b1 = x1
            delta_a = deltay
            delta_b = deltax
            deltaerr = abs(deltax / deltay)

        b_increment = 1
        if (delta_b < 0):
            b_increment = -1

        a_increment = 1
        if (delta_a < 0):
            a_increment = -1

        arrays = [[],[]]
        error = deltaerr - .5
        b = b0
        for a in range(int(a0), int(a1), int(a_increment)):
            if a_first:
                if (self.occ_grid[int(a),int(b)] == 1):
                    return True
            else:
                if (self.occ_grid[int(b),int(a)] == 1):
                    return True
            error += deltaerr
            if error >= .5:
                b += b_increment
                error -= 1
        return False

    def get_random_node(self):
        pos = [randint(0, self.field_dim[0] - 1), randint(0, self.field_dim[1] - 1)]
        q_rand = RRTNode(pos)
        return q_rand

    def get_nearest_node(self, q_rand):
        min_dist = 1000000
        for node in self.nodes:
            dist = node.get_dist(q_rand)
            if (dist < min_dist):
                min_dist = dist
                q_near = node
        return q_near

    def get_unit_vector(self, unit_length, a, b):
        vector = [b.pos[0] - a.pos[0], b.pos[1] - a.pos[1]]
        normalized_vector = vector / np.linalg.norm(vector)
        return normalized_vector * unit_length

    def create_path(self, q_goal):
        path = []
        q = q_goal
        while(q.parent != None):
            path.append(q.pos)
            self.visualization[int(q.pos[1]), int(q.pos[0])] = self.PATH
            self.plot_path_line(q.pos[0], q.parent.pos[0], q.pos[1], q.parent.pos[1])
            q = q.parent
        self.visualization[int(q.pos[1]), int(q.pos[0])] = self.PATH
        path.append(q.pos)
        path.reverse()
        return path
