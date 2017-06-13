import numpy as np

def step(mu_t, E_t, F, E_x, H, E_z, z_tp1):
    identity_dim = F.shape[0]
    K_tp1 = np.dot(np.dot(np.add(np.dot(np.dot(F, E_t), F.T), E_x), H.T), np.linalg.inv(np.add(np.dot(np.dot(H, np.add(np.dot(np.dot(F, E_t), F.T), E_x)), H.T), E_z)))
    mu_tp1 = np.add(np.dot(F, mu_t), np.dot(K_tp1, np.subtract(z_tp1, np.dot(H, np.dot(F, mu_t)))))
    E_tp1 = np.dot(np.subtract(np.identity(identity_dim), np.dot(K_tp1, H)), np.add(np.dot(F, np.dot(E_t, F.T)), E_x))
    return (mu_tp1, E_tp1)

t = 1
mu_0 = np.array([[0],[0],[0],[0],[0],[0]])
E_0 = np.zeros((6, 6))
E_x = np.array([[1, 0, 0, 0, 0, 0],[0, 10, 0, 0, 0, 0],[0, 0, 100, 0, 0, 0],[0, 0, 0, 1, 0, 0],[0, 0, 0, 0, 10, 0],[0, 0, 0, 0, 0, 100]])
E_z = np.array([[5, 0],[0, 5]])
F = np.array([[1, t, .5*t**2, 0, 0, 0],[0, 1, t, 0, 0, 0],[0, 0, 1, 0, 0, 0],[0, 0, 0, 1, t, .5*t**2],[0, 0, 0, 0, 1, t],[0, 0, 0, 0, 0, 1]])
H = np.array([[1, 0, 0, 0, 0, 0],[0, 0, 0, 1, 0, 0]])

mu_t = mu_0
E_t = E_0
z_t = np.zeros((2, 1))

for t in range(1, 11):
    z_tp1 = np.array([[z_t[0,0] + t],[0]])
    mu_tp1, E_tp1 = step(mu_t, E_t, F, E_x, H, E_z, z_tp1)
    print('t: ', t)
    print('z_tp1:')
    print(z_tp1)
    print('mu_tp1:')
    print(mu_tp1)
    print()
    z_t = z_tp1
    mu_t = mu_tp1
    E_t = E_tp1

mu_t = mu_0
E_t = E_0
z_t = np.zeros((4, 1))
H = np.array([[1, 0, 0, 0, 0, 0],[0, 1, 0, 0, 0, 0],[0, 0, 0, 1, 0, 0],[0, 0, 0, 0, 1, 0]])
E_z = np.array([[5, 0, 0, 0],[0, 5, 0, 0],[0, 0, 5, 0],[0, 0, 0, 5]])

for t in range(1, 11):
    z_tp1 = np.array([[z_t[0,0] + t],[t],[0],[0]])
    mu_tp1, E_tp1 = step(mu_t, E_t, F, E_x, H, E_z, z_tp1)
    print('t: ', t)
    print('z_tp1:')
    print(z_tp1)
    print('mu_tp1:')
    print(mu_tp1)
    print()
    z_t = z_tp1
    mu_t = mu_tp1
    E_t = E_tp1
