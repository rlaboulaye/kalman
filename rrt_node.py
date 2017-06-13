
class RRTNode(object):

    current_id = 0

    def __init__(self):
        self.id = RRTNode.current_id
        RRTNode.current_id += 1
        self.parent = None

    def add_parent(self, parent):
        self.parent = parent
