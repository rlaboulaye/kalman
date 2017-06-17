from threading import Lock

class PositionContainer(object):


    def __init__(self, robot_pos, obstacle_pos, goal_pos):
        self.lock = Lock()
        self.robot_pos = robot_pos
        self.obstacle_pos = obstacle_pos
        self.goal_pos = goal_pos
        self.changed = False


    def get_positions(self):
        self.lock.acquire()
        positions = (self.robot_pos, self.obstacle_pos, self.goal_pos)
        self.changed = False
        self.lock.release()
        return positions


    def set_positions(self, robot_pos, obstacle_pos, goal_pos):
        self.lock.acquire()
        self.robot_pos = robot_pos
        self.obstacle_pos = obstacle_pos
        self.goal_pos = goal_pos
        self.changed = True
        self.lock.release()


    def set_robot_position(self, robot_pos):
        self.lock.acquire()
        self.robot_pos = robot_pos
        self.lock.release()


    def set_obstacle_positions(self, obstacle_pos):
        self.lock.acquire()
        self.obstacle_pos = obstacle_pos
        self.lock.release()


    def set_goal_position(self, goal_pos):
        self.lock.acquire()
        self.goal_pos = goal_pos
        self.lock.release()


    def changed():
        return self.changed
