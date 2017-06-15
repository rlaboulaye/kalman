import numpy as np

from potential_field import PotentialField

class AttractiveField(PotentialField):

    def __init__(self, object_radius, slowdown_distance, max_force, exists):
        self.object_radius = object_radius
        self.slowdown_distance = slowdown_distance
        self.max_force = max_force
        self.exists = exists

    def get_vector(self, robot_position, object_position):
        if not self.exists:
            return [0., 0.]
        distance = self.get_distance(robot_position, object_position)
        angle = self.get_angle(robot_position, object_position)
        if distance < self.object_radius + self.slowdown_distance:
            dx = (self.max_force - ((self.max_force / (self.slowdown_distance + self.object_radius)) * (self.object_radius + self.slowdown_distance - distance) * .75)) * np.cos(angle)
            dy = (self.max_force - ((self.max_force / (self.slowdown_distance + self.object_radius)) * (self.object_radius + self.slowdown_distance - distance) * .75)) * np.sin(angle)
        # if distance < self.object_radius:
        #     dx = 0.
        #     dy = 0.
        # elif (self.object_radius <= distance) and (distance <= (self.object_radius + self.slowdown_distance)):
        #     dx = (self.max_force - ((self.max_force / self.slowdown_distance) * (self.object_radius + self.slowdown_distance - distance))) * np.cos(angle)
        #     dy = (self.max_force - ((self.max_force / self.slowdown_distance) * (self.object_radius + self.slowdown_distance - distance))) * np.sin(angle)
        #     # dx = self.scale_factor * (distance - self.object_radius) * np.cos(angle)
        #     # dy = self.scale_factor * (distance - self.object_radius) * np.sin(angle)
        else:
            dx = self.max_force * np.cos(angle)
            dy = self.max_force * np.sin(angle)
            # dx = self.scale_factor * self.slowdown_distance * np.cos(angle)
            # dy = self.scale_factor * self.slowdown_distance * np.sin(angle)
        return [dx, dy]

    def exists(self):
        return self.exists
