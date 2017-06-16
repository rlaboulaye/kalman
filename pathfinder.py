import math

from threading import Thread

class Pathfinder(Thread):


    def __init__(self, searcher, path_container, pos_container, tag_radius)
        self.searcher = searcher
        self.path_container = path_container
        self.pos_container = pos_container
        self.tag_radius = tag_radius


    def run(self):
        not_at_goal = True
        while not_at_goal:
            (robot_position, obstacle_pos, goal_position) = self.pos_container.get_positions()
            old_path = path_container.get_path(reset_changed=False)
            path = searcher.get_path(robot_position, obstacle_pos, goal_position, 10)
            if path != old_path:
                self.path_container.set_path(path)
            distance_to_goal = math.sqrt((goal_position[0] - robot_position[0]) ** 2 + (goal_position[1] - robot_position[1]) ** 2)
            not_at_goal = distance_to_goal > tag_radius * 2


