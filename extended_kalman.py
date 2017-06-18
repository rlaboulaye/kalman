from math import sin, cos, pi
import numpy as np
from queue import Queue

class ExtendedKalman(object):

    def __init__(self, delta_t, lag_multiplier, initial_pos, initial_orientation):
        
        self.delta_t = delta_t
        self.lag_multiplier = lag_multiplier

        self.velocities = Queue(maxsize=self.lag_multiplier)

        self.E_x = np.array([[1, 0, 0, 0, 0, 0, 0],[1, 0, 0, 0, 0, 0, 0],[0, 0, 4, 0, 0, 0, 0],[0, 0, 0, 4, 0, 0, 0],[0, 0, 0, 0, 10, 0, 0],[0, 0, 0, 0, 0, 10, 0],[0, 0, 0, 0, 0, 0, .4]])
        self.H = np.array([[1, 0, 0, 0, 0, 0, 0],[0, 1, 0, 0, 0, 0, 0],[0, 0, 1, 0, 0, 0, 0],[0, 0, 0, 1, 0, 0, 0],[0, 0, 0, 0, 0, 0, 1]])
        self.E_z = np.array([[4, 0, 0, 0, 0],[0, 4, 0, 0, 0],[0, 0, 5, 0, 0],[0, 0, 0, 5, 0],[0, 0, 0, 0, .05]])

        self.H_vel = np.array([[0, 0, 1, 0, 0, 0, 0],[0, 0, 0, 1, 0, 0, 0]])
        self.E_z_vel = np.array([[5, 0],[0, 5]])

        self.reset(initial_pos, initial_orientation)

    def reset(self, initial_pos, initial_orientation):
        self.mu_t = np.array([[initial_pos[0]],[initial_pos[1]],[0],[0],[0],[0],[initial_orientation]])
        self.E_t = np.zeros((7, 7))
        self.velocities = Queue(maxsize=self.lag_multiplier)

    def apply_f(self, mu_t, delta_t):
        # b is the distance between the wheels
        b = 1

        vec = np.zeros((7,1))
        px = mu_t[0,0]
        py = mu_t[1,0]
        vr = mu_t[2,0]
        vl = mu_t[3,0]
        ar = mu_t[4,0]
        al = mu_t[5,0]
        theta = mu_t[6,0]

        if (vr == vl):
            vl -= .0001

        # r is the radius of the circle
        r = (b / 2) * ((vl + vr) / (vr - vl))

        # w is angular velocity
        w = (1 / b) * (vr - vl)

        vec[0,0] = px + r * (-sin(theta) + (cos(theta) * sin(w * delta_t)) + (sin(theta) * cos(w * delta_t)))
        vec[1,0] = py + r * (cos(theta) + (sin(theta) * sin(w * delta_t)) - (cos(theta) * cos(w * delta_t)))
        vec[2,0] = vr + delta_t * ar
        vec[3,0] = vl + delta_t * al
        vec[4,0] = ar
        vec[5,0] = al
        vec[6,0] = (theta + (w * delta_t)) % (2 * pi)
        return vec

    def get_jacobian(self, mu_t, delta_t):
        # b is the distance between the wheels
        b = 1

        jac = np.zeros((7,7))
        px = mu_t[0,0]
        py = mu_t[1,0]
        vr = mu_t[2,0]
        vl = mu_t[3,0]
        ar = mu_t[4,0]
        al = mu_t[5,0]
        theta = mu_t[6,0]

        if (vr == vl):
            vl -= .0001

        jac[0,0] = 1
        jac[0,1] = 0
        jac[0,2] = ((delta_t * cos((delta_t * (vr - vl) / b) + theta) * (vr + vl) * (vr - vl)) - (2 * vl * b * ((cos(delta_t * (vr - vl) / b) * sin(theta)) + (sin(delta_t * (vr - vl) / b) * cos(theta)) - sin(theta)))) / (2 * (vr - vl) ** 2)
        jac[0,3] = ((2 * b * vr * ((cos(delta_t * (vr - vl) / b) * sin(theta)) + (sin(delta_t * (vr - vl) / b) * cos(theta)) - sin(theta))) - (delta_t * cos((delta_t * (vr - vl) / b) + theta) * (vr + vl) * (vr - vl))) / (2 * (vr - vl) ** 2)
        jac[0,4] = 0
        jac[0,5] = 0
        jac[0,6] = b * (vr + vl) * (cos(delta_t * (vr - vl) / b) * cos(theta) - cos(theta) - sin(delta_t * (vr - vl) / b) * sin(theta)) / (2 * (vr - vl))
    
        jac[1,0] = 0
        jac[1,1] = 1
        jac[1,2] = ((delta_t * sin((delta_t * (vr - vl) / b) + theta) * (vr + vl) * (vr - vl)) - (2 * vl * b * (cos(theta) + (sin(delta_t * (vr - vl) / b) * sin(theta)) - (cos(delta_t * (vr - vl) / b) * cos(theta))))) / (2 * (vr - vl) ** 2)
        jac[1,3] = ((2 * vr * b * (cos(theta) + (sin(delta_t * (vr - vl) / b) * sin(theta)) - (cos(delta_t * (vr - vl) / b) * cos(theta)))) - (delta_t * sin((delta_t * (vr - vl) / b) + theta) * (vr + vl) * (vr - vl))) / (2 * (vr - vl) ** 2)
        jac[1,4] = 0
        jac[1,5] = 0
        jac[1,6] = b * (vr + vl) * ((sin(delta_t * (vr - vl) / b) * cos(theta)) + (cos(delta_t * (vr - vl) / b) * sin(theta)) - sin(theta)) / (2 * (vr - vl))

        jac[2,0] = 0
        jac[2,1] = 0
        jac[2,2] = 1
        jac[2,3] = 0
        jac[2,4] = delta_t
        jac[2,5] = 0
        jac[2,6] = 0

        jac[3,0] = 0
        jac[3,1] = 0
        jac[3,2] = 0
        jac[3,3] = 1
        jac[3,4] = 0
        jac[3,5] = delta_t
        jac[3,6] = 0

        jac[4,0] = 0
        jac[4,1] = 0
        jac[4,2] = 0
        jac[4,3] = 0
        jac[4,4] = 1
        jac[4,5] = 0
        jac[4,6] = 0

        jac[5,0] = 0
        jac[5,1] = 0
        jac[5,2] = 0
        jac[5,3] = 0
        jac[5,4] = 0
        jac[5,5] = 1
        jac[5,6] = 0

        jac[6,0] = 0
        jac[6,1] = 0
        jac[6,2] = delta_t / b
        jac[6,3] = - delta_t / b
        jac[6,4] = 0
        jac[6,5] = 0
        jac[6,6] = 1

        return jac

    def step(self, mu_t, E_t, E_x, H, E_z, z_tp1, delta_t):
        mu_prediction = self.apply_f(mu_t, delta_t)
        F = self.get_jacobian(mu_t, delta_t)
        identity_dim = F.shape[0]
        K_tp1 = np.dot(np.dot(np.add(np.dot(np.dot(F, E_t), F.T), E_x), H.T), np.linalg.inv(np.add(np.dot(np.dot(H, np.add(np.dot(np.dot(F, E_t), F.T), E_x)), H.T), E_z)))
        mu_tp1 = np.add(mu_prediction, np.dot(K_tp1, np.subtract(z_tp1, np.dot(H, mu_prediction))))
        E_tp1 = np.dot(np.subtract(np.identity(identity_dim), np.dot(K_tp1, H)), np.add(np.dot(F, np.dot(E_t, F.T)), E_x))
        return (mu_tp1, E_tp1)

    def get_position(self, pos, vel, orientation, prediction_factor=0):
        if (self.velocities.full()):
            orientation_old = orientation
            pos_old = pos
            vel_old = self.velocities.get()
            z_tp1 = np.array([[pos_old[0]],[pos_old[1]],[vel_old[0]],[vel_old[1]],[orientation_old]])
            mu_tp1, E_tp1 = self.step(self.mu_t, self.E_t, self.E_x, self.H, self.E_z, z_tp1, delta_t)
            self.mu_t = mu_tp1
            self.E_t = E_tp1

        self.velocities.put(vel)
        backup_velocities = Queue(maxsize=self.lag_multiplier)
        mu_t = self.mu_t
        E_t = self.E_t
        while(not self.velocities.empty()):
            new_vel = self.velocities.get()
            backup_velocities.put(new_vel)
            z_tp1 = np.array([[new_vel[0]],[new_vel[1]]])
            mu_tp1, E_tp1 = self.step(mu_t, E_t, F, self.E_x, self.H_vel, self.E_z_vel, z_tp1)
            mu_t = mu_tp1
            E_t = E_tp1
        self.velocities = backup_velocities

        assert(type(prediction_factor) == int)
        if (prediction_factor > 0):
            delta_t = prediction_factor * self.delta_t
            mu_tp1 = self.apply_f(mu_tp1, delta_t)

        new_pos = [mu_tp1[0], mu_tp1[3]]
        return new_pos

