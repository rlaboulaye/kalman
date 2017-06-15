import math
import numpy as np

from potential_field import PotentialField

class TangentialField(PotentialField):

    def __init__(self, object_radius, movement_distance, max_force, exists, counterclockwise = True):
        self.object_radius = object_radius
        self.movement_distance = movement_distance
        self.max_force = max_force
        self.exists = exists
        if counterclockwise:
            self.direction = 1
        else:
            self.direction = -1

    def get_vector(self, robot_position, object_position):
        if not self.exists:
            return [0., 0.]
        distance = self.get_distance(robot_position, object_position)
        angle = self.get_angle(robot_position, object_position)
        if distance < self.object_radius:
            print('robot within obstacle radius')
            dx = 0
            dy = 0
            # dx = -(np.sign(np.cos(angle))) * (1.5 * self.max_force)
            # dy = -(np.sign(np.sin(angle))) * (1.5 * self.max_force)
        elif (self.object_radius <= distance) and (distance <= (self.object_radius + self.movement_distance)):
            constant_proportion = .25
            if distance >= self.object_radius + (3 * self.movement_distance / 4):
                magnitude = (self.max_force * constant_proportion)
                dx = self.direction * magnitude * np.cos(angle + math.pi / 2)
                dy = self.direction * magnitude * np.sin(angle + math.pi / 2)
            else:
                magnitude = (((1 - constant_proportion) * self.max_force) / (3 * self.movement_distance / 4)) * ((3 * self.movement_distance / 4) - distance + self.object_radius) + (constant_proportion * self.max_force)
                dx = self.direction * magnitude * np.cos(angle + math.pi / 2)
                dy = self.direction * magnitude * np.sin(angle + math.pi / 2)
        else:
            dx = 0.
            dy = 0.
        return [dx, dy]

    def exists(self):
        return self.exists
