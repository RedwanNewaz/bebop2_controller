import numpy as np
import math
from .ObservationModel import estimate_zx
from . import pf_localization
class ParticleFilter:
    def __init__(self, landmarks, Z0, STATE_DIM, NP, DT):
        self.px = np.zeros((STATE_DIM, NP))  # Particle store
        self.pw = np.zeros((1, NP)) + 1.0 / NP  # Particle weight

        self.landmarks = landmarks
        self.DT = DT
        self.NP = NP

        self.NUM_MAX_THREADS = 12

        # initialize particle distribution from noisy landmark observations
        pdf = estimate_zx(Z0, landmarks)
        if len(pdf) > 0:
            K = NP // len(pdf)  # number of particle groups
        else:
            # Handle the case where pdf is empty, set K to a default value
            K = 1
        #K = NP // len(pdf) # number of particle groups
        for i, pd in enumerate(pdf):
            self.px[:4, i * K: (i + 1) * K] = np.array([pd[:4] for _ in range(K)]).T

        if len(pdf) > 0:
            self.z_t_1 = pdf.mean(axis=0).reshape((4, 1))
        else:
            # Handle the case where pdf is empty
            self.z_t_1 = np.zeros((4, 1))
        
        #self.z_t_1 = pdf.mean(axis=0).reshape((4, 1))
        self.initialized = False
        self.x_est = None
        self.p_est = None

    def estimate_control(self, Z):
        u = None
        if len(Z):
            zx = estimate_zx(Z, self.landmarks).mean(axis=0).reshape((4, 1))
            dx = zx - self.z_t_1
            v = dx / self.DT
            self.z_t_1 = zx.copy()
            if (self.initialized):
                u = v.copy()
            self.initialized = True
        else:
            self.initialized = False

        return u

    def __call__(self, Z, u):
        self.x_est, self.p_est, self.px, self.pw = pf_localization(self.px, self.pw, Z, u, self.DT, self.NP, self.NUM_MAX_THREADS)


    def getState(self):
        return self.x_est

    def getCovariance(self):
        return self.p_est

