import math
import numpy as np
from . import generate_noisy_control, motion_model
from .ObservationModel import pi_2_pi
from concurrent.futures import ThreadPoolExecutor
Q = np.diag([0.02, np.deg2rad(5.0)]) ** 2  # range error
np.random.seed(1234)


def get_noisy_reading(x, zd):
    r, phi, theta = zd
    phi = pi_2_pi(phi - x[3, 0])
    # theta = pi_2_pi(theta - x[3, 0])
    # xx = x[0, 0] + r * math.cos(theta)
    # yy = x[1, 0] + r * math.sin(theta)
    # Calculate x-coordinate
    xx = x[0, 0] + r * np.sin(theta) * np.cos(phi)

    # Calculate y-coordinate
    y = x[1, 0] + r * np.sin(theta) * np.sin(phi)

    # Calculate z-coordinate
    z = x[2, 0] + r * np.cos(theta)

    return np.array([xx, y, z])


def gauss_likelihood(x, sigma):
    p = 1.0 / math.sqrt(2.0 * math.pi * sigma ** 2) * \
        math.exp(-x ** 2 / (2 * sigma ** 2))

    return p


def calc_covariance(x_est, px, pw):
    """
    calculate covariance matrix
    see ipynb doc
    """
    cov = np.zeros((8, 8))
    n_particle = px.shape[1]
    for i in range(n_particle):
        dx = (px[:, i:i + 1] - x_est)[0:8]
        cov += pw[0, i] * dx @ dx.T
    cov *= 1.0 / (1.0 - pw @ pw.T)

    return cov

def re_sampling(px, pw, NP):
    """
    low variance re-sampling
    """

    w_cum = np.cumsum(pw)
    base = np.arange(0.0, 1.0, 1 / NP)
    re_sample_id = base + np.random.uniform(0, 1 / NP)
    indexes = []
    ind = 0
    for ip in range(NP):
        while re_sample_id[ip] > w_cum[ind]:
            ind += 1
        indexes.append(ind)

    px = px[:, indexes]
    pw = np.zeros((1, NP)) + 1.0 / NP  # init weight

    return px, pw


# def pf_localization(px, pw, z, u, dt, NP):
#     """
#     Localization with Particle filter
#     """
#     NTh = NP / 2.0  # Number of particle for re-sampling
#     for ip in range(NP):
#         x = np.array([px[:, ip]]).T
#         w = pw[0, ip]
#
#         #  Predict with random input sampling
#         ud = generate_noisy_control(u)
#         x = motion_model(x, ud, dt)
#
#         #  Calc Importance Weight
#         for i, y in enumerate(z):
#             x_noise = get_noisy_reading(x, y[:2])
#             dx = x[0, 0] - x_noise[0]
#             dy = x[1, 0] - x_noise[1]
#             pre_z = math.hypot(dx, dy)
#             dz = pre_z - y[0]
#             w = w * gauss_likelihood(dz, math.sqrt(Q[0, 0]))
#
#         px[:, ip] = x[:, 0]
#         pw[0, ip] = w
#
#     # print(pw.sum())
#     pw = pw / pw.sum()  # normalize
#
#
#     x_est = px.dot(pw.T)
#     p_est = calc_covariance(x_est, px, pw)
#
#     N_eff = 1.0 / (pw.dot(pw.T))[0, 0]  # Effective particle number
#     if N_eff < NTh:
#         px, pw = re_sampling(px, pw)
#     return x_est, p_est, px, pw


def particle_filter_worker(ip, px, pw, z, u, dt):
    x = np.array([px[:, ip]]).T
    w = pw[0, ip]
    if u is None:
        u = generate_noisy_control(np.zeros((4, 1))) 

    # Predict with random input sampling
    ud = generate_noisy_control(u)
    x = motion_model(x, ud, dt)

    # Calculate Importance Weight
    for i, y in enumerate(z):
        x_noise = get_noisy_reading(x, y[:3])
        dx = x[0, 0] - x_noise[0]
        dy = x[1, 0] - x_noise[1]
        dz = x[2, 0] - x_noise[2]
        pre_z = math.hypot(dx, dy, dz)

        delta_z = pre_z - y[0]
        p_dz = gauss_likelihood(delta_z, math.sqrt(Q[0, 0]))

        # phi = pi_2_pi(np.arctan2(dy, dx) - x[3, 0])
        # phi_z = pi_2_pi(phi - y[1]) + np.pi / 2.0
        # p_dphi = gauss_likelihood(phi_z, math.sqrt(Q[1, 1]))
        # w = w * (p_dz + p_dphi)
        w = w * (p_dz)

    px[:, ip] = x[:, 0]
    pw[0, ip] = w


def pf_localization(px, pw, z, u, dt, NP, max_threads):
    NTh = NP / 2.0 # Number of particles for re-sampling
    num_particles = len(px[0])

    # Create a thread pool with a maximum number of threads
    with ThreadPoolExecutor(max_threads) as executor:
        # Submit particle_filter_worker for each particle
        futures = [executor.submit(particle_filter_worker, ip, px, pw, z, u, dt) for ip in range(num_particles)]

        # Wait for all tasks to complete
        for future in futures:
            future.result()

    pw = pw / pw.sum()  # Normalize

    x_est = px.dot(pw.T)
    p_est = calc_covariance(x_est, px, pw)

    N_eff = 1.0 / (pw.dot(pw.T))[0, 0]  # Effective particle number
    # print(N_eff, NTh)
    if N_eff <= NTh:
        # print('resampling')
        px, pw = re_sampling(px, pw, NP)

    return x_est, p_est, px, pw