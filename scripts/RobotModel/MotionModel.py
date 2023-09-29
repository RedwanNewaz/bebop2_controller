import numpy as np

R_sim = np.diag([1.0, np.deg2rad(30.0)]) ** 2

def motion_model(x, u, dt):
    '''
    :param x: state variable (x, y, z, yaw, x_dot, y_dot, z_dot, yaw_dot
    :param u: control (vx, vy, vz, w_yaw)
    :param dt: sample time
    :return: next state
    '''
    F = np.diag([1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0])

    B = np.array([
                 [dt, 0.0, 0.0, 0.0],
                 [0.0, dt, 0.0, 0.0],
                 [0.0, 0.0, dt, 0.0],
                 [0.0, 0.0, 0.0, dt],
                 [1.0, 0.0, 0.0, 0.0],
                 [0.0, 1.0, 0.0, 0.0],
                 [0.0, 0.0, 1.0, 0.0],
                 [0.0, 0.0, 0.0, 1.0]
                 ])

    x = F.dot(x) + B.dot(u)

    return x

def generate_noisy_control(u):
    # add noise to input
    ud1 = u[0, 0] + np.random.randn() * R_sim[0, 0] ** 0.5
    ud2 = u[1, 0] + np.random.randn() * R_sim[0, 0] ** 0.5
    ud3 = u[2, 0] + np.random.randn() * R_sim[0, 0] ** 0.5
    ud4 = u[3, 0] + np.random.randn() * R_sim[1, 1] ** 0.5
    ud = np.array([[ud1, ud2, ud3, ud4]]).T
    return ud