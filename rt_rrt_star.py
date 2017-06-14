import random
import time

import numpy as np

from collections import deque

from rrt_container import RRTContainer
from rrt_node import RRTNode
from search   import Search

class RTRRTStar(object):

    def __init__(self, field_dim, time_limit, tag_radius, robot_radius):
       self.field_dim = field_dim
       self.time_limit = time_limit
       self.tag_radius = tag_radius
       self.robot_radius = robot_radius
       self.near_radius = 50
       self.Q_r = deque()
       self.Q_s = deque()
       self.k_max = 15
       self.T = RRTContainer(field_dim, self.near_radius)
       self.node_occupancy_grid = np.zeros((field_dim[1], field_dim[0]))


    def create_occupancy_grid(self, field_dim, obstacle_pos, tag_radius, robot_radius):
        self.occ_grid = np.zeros((field_dim[1], field_dim[0]))

        for pos in obstacle_pos:
            x0 = pos[0] - tag_radius - robot_radius
            x1 = pos[0] + tag_radius + robot_radius + 1
            y0 = pos[1] - tag_radius - robot_radius
            y1 = pos[1] + tag_radius + robot_radius + 1
            self.occ_grid[y0:y1, x0:x1] = 1
            # self.plot_obstacle(x0, x1, y0, y1)

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

    def reconstruct_path(self):
        pass


    def get_path(self, robot_pos, obstacle_pos, goal_pos):
        self.create_occupancy_grid(self.field_dim, obstacle_pos, self.tag_radius, self.robot_radius)
        for i in range(0, 5):
            self.expand_rewire(self.T, self.Q_r, self.Q_s, self.k_max, self.near_radius)
        return self.reconstruct_path()
    

    def expand_rewire(self, T, Q_r, Q_s, k_max, r_s):
        while True:
            x_rand = Node(random.randint(0, field_dim[0] - 1), random.randint(0, field_dim[1] - 1))
            if (self.node_occupancy_grid[x_rand.pos[1], x_rand.pos[0]] == 0) and (self.occupancy_grid[x_rand.pos[1], x_rand.pos[0]] == 0):
                break
        x_closest = T.get_nearest_node(x_rand)
        if not self.occupied(x_closest, x_rand):
            X_near = T.get_neighbors(x_rand)
            if (len(X_near) < k_max) or (x_rand.get_distance(x_closest) > r_s):
                self.add_node_to_tree(T, x_rand, x_closest, X_near)
                Q_r.appendleft(x_rand)
            else:
                Q_r.appendleft(x_closest)
            self.rewire_random_nodes(Q_r, T)
        self.rewire_from_root(Q_s, T)


    def add_node_to_tree(self, T, x_new, x_closest, X_near):
        x_min = x_closest
        c_min = x_closest.cost + x_closest.get_distance(x_new)
        for x_near in X_near:
            c_new = x_near.cost + x_near.get_distance(x_new)
            if (c_new < c_min) and (not self.occupied(x_near, x_new)):
                c_min = c_new
                x_min = x_near
        # add vertex to tree
        T.add_node(x_new)
        # add edge to tree
        x_new.set_parent(x_min)


    def rewire_random_nodes(self, Q_r, T):
        start_time = time()
        while (time() - start_time < self.time_limit) and (len(Q_r) > 0):
            x_r = Q_r.popleft()
            X_near = T.get_neighbors(x_r)
            for x_near in X_near:
                c_old = x_near.cost
                c_new = x_r.cost + x_r.get_distance(x_near)
                if (c_new < c_old) and (not self.occupied(x_r, x_near)):
                    x_near.set_parent(x_r)
                    Q_r.append(x_near)


    def rewire_from_root(self, Q_s, T):
        start_time = time()
        if (len(Q_s) == 0):
            # append root to Q_s
        rewired_nodes = set(Q_s)
        while (time() - start_time < self.time_limit) and (len(Q_r) > 0):
            x_s = Q_s.popleft()
            X_near = T.get_neighbors(x_s)
            for x_near in X_near:
                c_old = x_near.cost
                c_new = x_s.cost + x_s.get_distance(x_near)
                if (c_new < c_old) and (not occupied(x_r, x_near):
                    x_near.set_parent(x_s)
                if not x_near in rewired_nodes:
                    Q_s.append(x_near)
                    rewired_nodes.add(x_near)


