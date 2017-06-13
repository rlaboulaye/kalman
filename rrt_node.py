from node import Node

class RRTNode(Node):

    current_id = 0

    def __init__(self, pos):
        super(RRTNode, self).__init__(pos)
        self.parent = None
        self.cost = 0
        self.id = RRTNode.current_id
        RRTNode.current_id += 1

    def set_parent(self, node):
        self.parent = node
        self.cost = node.cost + self.get_dist(node)

    def get_distance(self, node):
        return super(RRTNode, self).get_distance(node)

