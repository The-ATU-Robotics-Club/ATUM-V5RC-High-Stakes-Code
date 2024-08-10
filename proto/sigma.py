from collections import namedtuple
import numpy as np


def normalize(angle):
    angle = angle % (2 * np.pi)
    if angle > np.pi:
        angle -= 2 * np.pi
    return angle


SigmaPoint = namedtuple("SigmaPoint", ("state", "meanW", "covW"))


class SigmaPointGenerator:
    def __init__(self, alpha, beta, kappa, n):
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.n = n

    def createPoints(self, state, cov):
        lda = self.alpha**2 * (self.n + self.kappa) - self.n
        sigmas = np.zeros((2 * self.n + 1, self.n))
        meanW = np.full(2 * self.n + 1, 0.5 / (self.n + lda))
        covW = np.full(2 * self.n + 1, 0.5 / (self.n + lda))
        sigmas[0] = state
        meanW[0] = lda / (self.n + lda)
        covW[0] = lda / (self.n + lda) + (1 - self.alpha**2 + self.beta)
        # Needs upper triangle (lower is default so must transpose)
        term = np.linalg.cholesky((self.n + lda) * cov).T
        for k in range(self.n):
            sigmas[k + 1] = state + term[k]
            sigmas[self.n + k + 1] = state - term[k]
        return sigmas, meanW, covW
