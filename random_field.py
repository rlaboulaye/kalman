import math
import random
import numpy as np

from potential_field import PotentialField

class RandomField(PotentialField):

    def __init__(self, max_force):
        self.max = float(max_force) / 10.

    def get_vector(self, robot_position, object_position):
        theta = random.random() * 2 * math.pi
        magnitude = random.random() * self.max
        dx = magnitude * np.cos(theta)
        dy = magnitude * np.sin(theta)
        return [dx, dy]
