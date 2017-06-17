from math import sin, cos, pi
import numpy as np

def step(self, mu_t, E_t, F, E_x, H, E_z, z_tp1):
    identity_dim = F.shape[0]
    K_tp1 = np.dot(np.dot(np.add(np.dot(np.dot(F, E_t), F.T), E_x), H.T), np.linalg.inv(np.add(np.dot(np.dot(H, np.add(np.dot(np.dot(F, E_t), F.T), E_x)), H.T), E_z)))
    mu_tp1 = np.add(np.dot(F, mu_t), np.dot(K_tp1, np.subtract(z_tp1, np.dot(H, np.dot(F, mu_t)))))
    E_tp1 = np.dot(np.subtract(np.identity(identity_dim), np.dot(K_tp1, H)), np.add(np.dot(F, np.dot(E_t, F.T)), E_x))
    return (mu_tp1, E_tp1)

def apply_f(mu_t, delta_t):
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
    vec[1,0] = px + r * (cos(theta) + (sin(theta) * sin(w * delta_t)) - (cos(theta) * cos(w * delta_t)))
    vec[2,0] = vr + delta_t * ar
    vec[3,0] = vl + delta_t * al
    vec[4,0] = ar
    vec[5,0] = al
    vec[6,0] = (theta + (w * delta_t)) % (2 * pi)
    return vec

def get_jacobian(mu_t, delta_t):
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

    # r is the radius of the circle
    r = abs((b / 2) * ((vl + vr) / (vr - vl)))

    jac[0,0]


# px, py, vr, vl, ar, al
E_x = np.array([[1, 0, 0, 0, 0, 0, 0],[1, 0, 0, 0, 0, 0, 0],[0, 0, 5, 0, 0, 0, 0],[0, 0, 0, 5, 0, 0, 0],[0, 0, 0, 0, 10, 0, 0],[0, 0, 0, 0, 0, 10, 0],[0, 0, 0, 0, 0, 0, .3]])
H = np.array([[1, 0, 0, 0, 0, 0, 0],[0, 1, 0, 0, 0, 0, 0],[0, 0, 1, 0, 0, 0, 0],[0, 0, 0, 1, 0, 0, 0],[0, 0, 0, 0, 0, 0, 1]])
E_z = np.array([[5, 0, 0, 0, 0],[0, 5, 0, 0, 0],[0, 0, 5, 0, 0],[0, 0, 0, 5, 0],[0, 0, 0, 0, .1]])

mu_t = np.zeros((7,1))
E_t = np.zeros((7,7))

mu_t = np.array([[0],[0],[2],[1],[0],[0],[pi/2]])

for i in range(1):
    mu_t = apply_f(mu_t, 1)
    print()
    print(mu_t)

