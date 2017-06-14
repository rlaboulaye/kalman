import math
import numpy as np

from rrt_node import RRTNode

class SpatialNode(object):

    def __init__(self, dim, center, near_radius, depth, max_depth=5, parent=None):
       
        assert(dim[0] > near_radius and dim[1] > near_radius)

        self.near_radius = near_radius
        self.dim = np.array(dim)
        self.center = np.array(center)

        # corners
        self.blc = np.array(center + np.array([-1 * dim[0] / 2, dim[1] / 2]))
        self.tlc = np.array(center + np.array([-1 * dim[0] / 2, -1 * dim[1] / 2]))
        self.brc = np.array(center + np.array([dim[0] / 2, dim[1] / 2]))
        self.trc = np.array(center + np.array([dim[0] / 2, -1 * dim[1] / 2]))

        self.depth = depth
        self.max_depth = max_depth

        self.nodes = []

        self.parent = parent
        self.children = None

        if (depth < max_depth):
            self.generate_children()

        self.neighbors = [None, None, None, None, None, None, None, None]

    def generate_children(self):
        # child order
        # bottom_left, top_left, bottom_right, top_right
        self.children = []
        new_dim = [self.dim[0] / 2, self.dim[1] / 2]
        new_center = [self.center[0] - new_dim[0] / 2, self.center[1] + new_dim[1] / 2]
        bottom_left_child = SpatialNode(new_dim, new_center, self.near_radius, self.depth + 1, self.max_depth, parent = self)
        self.children.append(bottom_left_child)
        new_center = [self.center[0] - new_dim[0] / 2, self.center[1] - new_dim[1] / 2]
        top_left_child = SpatialNode(new_dim, new_center, self.near_radius, self.depth + 1, self.max_depth, parent = self)
        self.children.append(top_left_child)
        new_center = [self.center[0] + new_dim[0] / 2, self.center[1] + new_dim[1] / 2]
        bottom_right_child = SpatialNode(new_dim, new_center, self.near_radius, self.depth + 1, self.max_depth, parent = self)
        self.children.append(bottom_right_child)
        new_center = [self.center[0] + new_dim[0] / 2, self.center[1] - new_dim[1] / 2]
        top_right_child = SpatialNode(new_dim, new_center, self.near_radius, self.depth + 1, self.max_depth, parent = self)
        self.children.append(top_right_child)

    def label_neighbors_of_children(self):
        
        # label bottom left
        bottom_left = self.children[0]
        bottom_left.add_neighbor(self.children[1], 3)
        bottom_left.add_neighbor(self.children[3], 4)
        bottom_left.add_neighbor(self.children[2], 5)
        if (self.neighbors[7] != None):
            bottom_left.add_neighbor(self.neighbors[7].children[3], 6)
            bottom_left.add_neighbor(self.neighbors[7].children[1], 7)
        if (self.neighbors[0] != None):
            bottom_left.add_neighbor(self.neighbors[0].children[3], 0)
        if (self.neighbors[1] != None):
            bottom_left.add_neighbor(self.neighbors[1].children[2], 1)
            bottom_left.add_neighbor(self.neighbors[1].children[3], 2)
       
        # label top left
        top_left = self.children[1]
        top_left.add_neighbor(self.children[3], 5)
        top_left.add_neighbor(self.children[2], 6)
        top_left.add_neighbor(self.children[0], 7)
        if (self.neighbors[1] != None):
            top_left.add_neighbor(self.neighbors[1].children[2], 0)
            top_left.add_neighbor(self.neighbors[1].children[3], 1)
        if (self.neighbors[2] != None):
            top_left.add_neighbor(self.neighbors[2].children[2], 2)
        if (self.neighbors[3] != None):
            top_left.add_neighbor(self.neighbors[3].children[0], 3)
            top_left.add_neighbor(self.neighbors[3].children[2], 4)
       
        # label bottom right
        bottom_right = self.children[2]
        bottom_right.add_neighbor(self.children[0], 1)
        bottom_right.add_neighbor(self.children[1], 2)
        bottom_right.add_neighbor(self.children[3], 3)
        if (self.neighbors[5] != None):
            bottom_right.add_neighbor(self.neighbors[5].children[1], 4)
            bottom_right.add_neighbor(self.neighbors[5].children[0], 5)
        if (self.neighbors[6] != None):
            bottom_right.add_neighbor(self.neighbors[6].children[1], 6)
        if (self.neighbors[7] != None):
            bottom_right.add_neighbor(self.neighbors[7].children[3], 7)
            bottom_right.add_neighbor(self.neighbors[7].children[1], 0)

        # label top right
        bottom_right = self.children[3]
        bottom_right.add_neighbor(self.children[2], 7)
        bottom_right.add_neighbor(self.children[0], 0)
        bottom_right.add_neighbor(self.children[1], 1)
        if (self.neighbors[3] != None):
            bottom_right.add_neighbor(self.neighbors[3].children[0], 2)
            bottom_right.add_neighbor(self.neighbors[3].children[2], 3)
        if (self.neighbors[4] != None):
            bottom_right.add_neighbor(self.neighbors[4].children[0], 4)
        if (self.neighbors[5] != None):
            bottom_right.add_neighbor(self.neighbors[5].children[1], 5)
            bottom_right.add_neighbor(self.neighbors[5].children[0], 6)

        if (self.depth < self.max_depth - 1):
            for child in self.children:
                child.label_neighbors_of_children()

    def add_neighbor(self, neighbor, index):
        # neighbor order
        # bl, l, tl, t, tr, r, br, b
        self.neighbors[index] = neighbor

    def get_child_index(self, node):
        if (node.pos[0] <= self.center[0]):
            if (node.pos[1] <= self.center[1]):
                child_index = 1
            else:
                child_index = 0
        else:
            if (node.pos[1] <= self.center[1]):
                child_index = 3
            else:
                child_index = 2
        return child_index

    def dist_to_line(self, line_pt1, line_pt2, pt):
        line = line_pt2 - line_pt1
        line_direction = line / np.linalg.norm(line)
        line_to_pt = pt - line_pt1
        pt_on_line = np.dot(line_to_pt, line_direction) * line_direction + line_pt1
        dist = self.get_dist(pt, pt_on_line)
        return dist

    def get_neighbor_indices(self, node, max_dist):
        indices = []
        # bottom left neighbor
        dist = self.get_dist(node.pos, self.blc)
        if (dist < max_dist):
            indices.append(0)
        # left neighbor
        dist = self.dist_to_line(self.blc, self.tlc, np.array(node.pos))
        if (dist < max_dist):
            indices.append(1)
        # top left neighbor
        dist = self.get_dist(node.pos, self.tlc)
        if (dist < max_dist):
            indices.append(2)
        # top neighbor
        dist = self.dist_to_line(self.tlc, self.trc, np.array(node.pos))
        if (dist < max_dist):
            indices.append(3)
        # top right neighbor
        dist = self.get_dist(node.pos, self.trc)
        if (dist < max_dist):
            indices.append(4)
        # right neighbor
        dist = self.dist_to_line(self.trc, self.brc, np.array(node.pos))
        if (dist < max_dist):
            indices.append(5)
        # bottom right neighbor
        dist = self.get_dist(node.pos, self.brc)
        if (dist < max_dist):
            indices.append(6)
        # bottom neighbor
        dist = self.dist_to_line(self.brc, self.blc, np.array(node.pos))
        if (dist < max_dist):
            indices.append(7)
        return indices

    def get_dist(self, pos1, pos2):
        dist = math.sqrt(((pos1[0] - pos2[0]) ** 2) + ((pos1[1] - pos2[1]) ** 2))
        return dist

    def add_rrt_node(self, node):
        if (self.depth < self.max_depth):
            child_index = self.get_child_index(node)
            self.children[child_index].add_rrt_node(node)
        else:
            self.nodes.append(node)

    def get_nearest_rrt_node(self, node, include_neighbor_spaces=True):
        if (self.depth < self.max_depth):
            child_index = self.get_child_index(node)
            return self.children[child_index].get_nearest_rrt_node(node)
        else:
            nearest = None
            min_dist = 1000000
            for near in self.nodes:
                if (node != near):
                    dist = node.get_dist(near)
                    if (dist < min_dist):
                        min_dist = dist
                        nearest = near
            if (include_neighbor_spaces):
                neighbor_indices = self.get_neighbor_indices(node, min_dist)
                for index in neighbor_indices:
                    if (self.neighbors[index] != None):
                        dist, near = self.neighbors[index].get_nearest_rrt_node(node, include_neighbor_spaces=False)
                        if (dist < min_dist):
                            min_dist = dist
                            nearest = near
            return (min_dist, nearest)

    def get_neighbors(self, node, neighborhood, include_neighbor_spaces=True):
        if (self.depth < self.max_depth):
            child_index = self.get_child_index(node)
            return self.children[child_index].get_neighbors(node, neighborhood)
        else:
            for near in self.nodes:
                if (node != near):
                    dist = node.get_dist(near)
                    if (dist <= self.near_radius):
                        neighborhood.append(near)
            if (include_neighbor_spaces):
                neighbor_indices = self.get_neighbor_indices(node, self.near_radius)
                for index in neighbor_indices:
                    if (self.neighbors[index] != None):
                        self.neighbors[index].get_neighbors(node, neighborhood, include_neighbor_spaces=False)
               

class RRTContainer(object):

    def __init__(self, dim, near_radius, max_depth=5):
        self.root = SpatialNode(dim, [dim[0] / 2, dim[1] / 2], near_radius, 1, max_depth)
        self.root.label_neighbors_of_children()
        self.rrt_nodes = []

    # this function gets called if the tree method does not work
    def get_nearest_node_in_linear_time(self, node):
        nearest = None
        min_dist = 1000000
        for near in self.rrt_nodes:
            dist = node.get_dist(near)
            if (dist < min_dist):
                min_dist = dist
                nearest = near
        return nearest

    def get_nearest_node(self, node):
        min_dist, near_node = self.root.get_nearest_rrt_node(node)
        if (near_node == None):
            near_node = self.get_nearest_node_in_linear_time(node)
        return near_node

    def get_neighbors(self, node):
        neighborhood = []
        self.root.get_neighbors(node, neighborhood)
        return neighborhood

    def add_node(self, node):
        self.rrt_nodes.append(node)
        self.root.add_rrt_node(node)

