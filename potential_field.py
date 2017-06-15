import math

class PotentialField(object):
    
    def get_vector(self, robot_position, object_position):
        pass
        
    def get_distance(self, robot_position, object_position):
        return math.sqrt((object_position[0] - robot_position[0]) ** 2 + (object_position[1] - robot_position[1]) ** 2)
        
    def get_angle(self, robot_position, object_position):
        return math.atan2((object_position[1] - robot_position[1]), (object_position[0] - robot_position[0]))