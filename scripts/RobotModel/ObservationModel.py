import numpy as np
import math



def pi_2_pi(angle):

    return (angle + math.pi) % (2 * math.pi) - math.pi


def convert_spherical_to_cartesian(Z):
    r, phi, theta = Z[:3]
    x = r * np.sin(theta) * np.cos(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(theta)
    return np.array([x, y, z, 0])

def estimate_control(Z_t, Z_t_1, dt):
    x_t = convert_spherical_to_cartesian(Z_t)
    x_t_1 = convert_spherical_to_cartesian(Z_t_1)
    v = (x_t - x_t_1) / dt
    return v.reshape((4, 1))


def estimate_zx(Z, landmarks):
    markers = np.squeeze(landmarks)
    X = np.zeros((0, 4))


    for obs in Z:
        landmarkID = int(obs[-2])
        x0 = markers[landmarkID, 0]
        y0 = markers[landmarkID, 1]
        z0 = markers[landmarkID, 2]

        transformed_point = convert_spherical_to_cartesian(obs)

        initial_point = np.array([x0, y0, z0, 0])
        xi = initial_point - transformed_point
        xi[-1] = obs[-1]
        X = np.vstack((X, xi))


    return X