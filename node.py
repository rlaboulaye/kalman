import math

class Node(object):

    def __init__(self, pos):
        self.pos = pos

    def get_dist(self, node):
        dist = math.sqrt(((self.pos[0] - node.pos[0]) ** 2) + ((self.pos[1] - node.pos[1]) ** 2))
        return dist

