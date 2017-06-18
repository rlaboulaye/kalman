import numpy as np
from queue import Queue

class Kalman(object):

    def __init__(self, delta_t, lag_multiplier, initial_pos, pos_only=False):
        
        self.delta_t = delta_t
        self.lag_multiplier = lag_multiplier

        if (pos_only):
            self.get_position = self.get_position_from_position
        else:
            self.get_position = self.get_position_from_position_and_velocity
            self.velocities = Queue(maxsize=self.lag_multiplier)

        self.E_x = np.array([[1, 0, 0, 0, 0, 0],[0, 4, 0, 0, 0, 0],[0, 0, 10, 0, 0, 0],[0, 0, 0, 1, 0, 0],[0, 0, 0, 0, 4, 0],[0, 0, 0, 0, 0, 10]])

        self.H_pos = np.array([[1, 0, 0, 0, 0, 0],[0, 0, 0, 1, 0, 0]])
        self.E_z_pos = np.array([[4, 0],[0, 4]])

        self.H_vel = np.array([[0, 1, 0, 0, 0, 0],[0, 0, 0, 0, 1, 0]])
        self.E_z_vel = np.array([[5, 0],[0, 5]])

        self.H = np.array([[1, 0, 0, 0, 0, 0],[0, 1, 0, 0, 0, 0],[0, 0, 0, 1, 0, 0],[0, 0, 0, 0, 1, 0]])
        self.E_z = np.array([[5, 0, 0, 0],[0, 5, 0, 0],[0, 0, 5, 0],[0, 0, 0, 5]])

        self.reset(initial_pos)

    def reset(self, initial_pos):
        self.mu_t = np.array([[initial_pos[0]],[0],[0],[initial_pos[1]],[0],[0]])
        self.E_t = np.zeros((6, 6))
        self.velocities = Queue(maxsize=self.lag_multiplier)

    def get_transition_matrix(self, t):
        F = np.array([[1, t, .5*t**2, 0, 0, 0],[0, 1, t, 0, 0, 0],[0, 0, 1, 0, 0, 0],[0, 0, 0, 1, t, .5*t**2],[0, 0, 0, 0, 1, t],[0, 0, 0, 0, 0, 1]])
        return F

    def step(self, mu_t, E_t, F, E_x, H, E_z, z_tp1):
        identity_dim = F.shape[0]
        K_tp1 = np.dot(np.dot(np.add(np.dot(np.dot(F, E_t), F.T), E_x), H.T), np.linalg.inv(np.add(np.dot(np.dot(H, np.add(np.dot(np.dot(F, E_t), F.T), E_x)), H.T), E_z)))
        mu_tp1 = np.add(np.dot(F, mu_t), np.dot(K_tp1, np.subtract(z_tp1, np.dot(H, np.dot(F, mu_t)))))
        E_tp1 = np.dot(np.subtract(np.identity(identity_dim), np.dot(K_tp1, H)), np.add(np.dot(F, np.dot(E_t, F.T)), E_x))
        return (mu_tp1, E_tp1)

    def get_position_from_position(self, pos, prediction_factor=0):
        F = self.get_transition_matrix(self.delta_t)
        z_tp1 = np.array([[pos[0]],[pos[1]]])
        mu_tp1, E_tp1 = self.step(self.mu_t, self.E_t, F, self.E_x, self.H_pos, self.E_z_pos, z_tp1)
        self.mu_t = mu_tp1
        self.E_t = E_tp1

        delta_t = (self.lag_multiplier + prediction_factor) * self.delta_t
        F = self.get_transition_matrix(delta_t)
        mu_tp1 = np.dot(F, mu_tp1)

        new_pos = [mu_tp1[0], mu_tp1[3]]
        return new_pos

    def get_position_from_position_and_velocity(self, pos, vel, prediction_factor=0):
        F = self.get_transition_matrix(self.delta_t)
        if (self.velocities.full()):
            pos_old = pos
            vel_old = self.velocities.get()
            z_tp1 = np.array([[pos_old[0]],[vel_old[0]],[pos_old[1]],[vel_old[1]]])
            mu_tp1, E_tp1 = self.step(self.mu_t, self.E_t, F, self.E_x, self.H, self.E_z, z_tp1)
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
            F = self.get_transition_matrix(delta_t)
            mu_tp1 = np.dot(F, mu_tp1)

        new_pos = [mu_tp1[0], mu_tp1[3]]
        return new_pos

