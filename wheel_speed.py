import math
import numpy as np

class WheelSpeed(object):

    def __init__(self, max_force):
        self.MAX_ROTATION_DELTA = max_force

    def get_wheel_speed(self, force):
        force_magnitude = np.linalg.norm(force)
        delta_v = force_magnitude
        speed = [int(round(delta_v)), int(round(delta_v))]
        return speed

    def adjust_speed_for_rotation(self, speed, object_direction, force):
        force = [force[0], -1 * force[1]]
        object_direction = [object_direction[0], -1 * object_direction[1]]
        dot_product = np.dot(force, object_direction)
        magnitude_product = np.linalg.norm(force) * np.linalg.norm(object_direction)
        theta = np.arccos(dot_product / magnitude_product)
        if np.isnan(theta):
            theta = 0
        if (theta > math.pi / 2):
            speed[0] = -1 * speed[0]
            speed[1] = -1 * speed[1]
            object_direction[0] = -1 * object_direction[0]
            object_direction[1] = -1 * object_direction[1]
            dot_product = np.dot(force, object_direction)
            magnitude_product = np.linalg.norm(force) * np.linalg.norm(object_direction)
            theta = np.arccos(dot_product / magnitude_product)
            if np.isnan(theta):
                theta = 0
        direction = np.dot(np.cross(np.append(object_direction, [0]), np.append(force, [0])), [0,0,1])
        delta_v = int(self.MAX_ROTATION_DELTA * theta / (math.pi / 2))
        if (direction < 0):
            speed[0] += delta_v
            speed[1] -= delta_v
        else:
            speed[0] -= delta_v
            speed[1] += delta_v
