import numpy as np

from potential_field import PotentialField

class RepulsiveField(PotentialField):

    def __init__(self, object_radius, movement_distance, max_force, exists):
        self.object_radius = object_radius
        self.movement_distance = movement_distance
        self.max_force = max_force
        self.exists = exists

    def get_vector(self, robot_position, object_position):
        if not self.exists:
            return [0., 0.]
        distance = self.get_distance(robot_position, object_position)
        angle = self.get_angle(robot_position, object_position)
        if distance < self.object_radius:
            print('robot within obstacle radius')
            dx = -(np.sign(np.cos(angle))) * (1.5 * self.max_force)
            dy = -(np.sign(np.sin(angle))) * (1.5 * self.max_force)
        elif (self.object_radius <= distance) and (distance <= (self.object_radius + self.movement_distance)):
            magnitude = (self.max_force / self.movement_distance) * (self.movement_distance - distance + self.object_radius)
            dx = -magnitude * np.cos(angle)
            dy = -magnitude * np.sin(angle)
            # if distance >= self.object_radius + (self.movement_distance / 2):
            #     dx = -(self.max_force * 0.75) * np.cos(angle)
            #     dy = -(self.max_force * 0.75) * np.sin(angle)
            # else:
            #     magnitude = ((0.25 * self.max_force) / (self.movement_distance / 2)) * ((self.movement_distance / 2) - distance + self.object_radius) + (0.75 * self.max_force)
            #     dx = -magnitude * np.cos(angle)
            #     dy = -magnitude * np.sin(angle)
        else:
            dx = 0.
            dy = 0.
        return [dx, dy]

    def exists(self):
        return self.exists
