import numpy as np
import pygame
import robot
import sigma
import sim

x = np.array([[0, 0, 0, 0, 0]])

pointGen = sigma.SigmaPointGenerator(0.05, 2, -2, 5)

# <STATES, STATES>
Q = np.array(
    [
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 70.75, 0],
        [0, 0, 0, 0, 70.75],
    ]
)

# <STATES, STATES>
P = np.array(
    [
        [2, 0, 0, 0, 0],
        [0, 2, 0, 0, 0],
        [0, 0, 4, 0, 0],
        [0, 0, 0, 0.001, 0],
        [0, 0, 0, 0, 0.001],
    ]
)

# <SENSORS, SENSORS>
R = np.array(
    [
        [0.05, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0.05, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, (np.pi / 180) * 2, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0.075, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0.075, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, (np.pi / 180) * 4, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0.5, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0.5, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, (np.pi / 180) * 2],
    ]
)


# Not done here, but make sure to properly average angles
def avg(X):
    mean = np.zeros((1, X.shape[1]))
    for point in X:
        mean += point
    mean /= len(X)
    return mean


def UT(X, meanW, covW, cov):
    mean = avg(X)
    statePrior = np.dot(meanW, X)
    kmax, n = X.shape
    PPrior = np.zeros((n, n))
    for k in range(kmax):
        y = X[k] - mean
        PPrior += covW[k] * np.outer(y, y)
    PPrior += cov
    return statePrior, PPrior


def H(points):
    # <MEASUREMENTS, STATES>
    HMat = np.vstack((np.eye(3, 5), np.eye(3, 5), np.eye(3, 5)))
    # Dot needed here, because numpy won't transpose row matrices
    return np.dot(HMat, points)


def sigmasToPriors(model, X, input):
    Y = np.zeros(X.shape)
    for point in range(len(X)):
        Y[point] = model.F(X[point], input)
    return Y


def sigmasToSensors(Y):
    # Should be size <SIGMAS, SENSORS>
    Z = np.zeros((len(Y), 9))
    for point in range(len(Y)):
        Z[point] = H(Y[point])
    return Z


def ukf():
    global x, P, Q, R

    input = sim.tank()

    # Predict
    sigmas, meanW, covW = pointGen.createPoints(x, P)
    priorSigmas = sigmasToPriors(sim.robotModel, sigmas, input)
    priorX, priorP = UT(priorSigmas, meanW, covW, Q)

    sim.robotModel.step(input)
    measurements = np.hstack(
        (
            sim.robotModel.getState(0.05, 0.05, (np.pi / 180) * 2)[0:3],
            sim.robotModel.getState(0.075, 0.075, (np.pi / 180) * 4)[0:3],
            sim.robotModel.getState(0.5, 0.5, (np.pi / 180) * 2)[0:3],
        )
    )

    # Update
    sensorSigmas = sigmasToSensors(priorSigmas)
    z, zP = UT(sensorSigmas, meanW, covW, R)
    xzP = np.zeros((len(Q), len(R)))
    for i in range(len(sensorSigmas)):
        prioriDiff = priorSigmas[i] - priorX
        postDiff = sensorSigmas[i] - z
        xzP += covW[i] * np.outer(prioriDiff, postDiff)
    K = np.dot(xzP, np.linalg.inv(zP))
    x = priorX + np.dot(K, measurements - z)
    P = priorP - np.dot(K, zP).dot(K.T)
    P = (P + P.T) / 2
    print(P)

    sim.robotView.draw()
    avgError = (P[0][0] + P[1][1]) / 2
    pygame.draw.circle(
        sim.screen, sim.red, sim.center(x[0], x[1]), 5 * sim.mToPix(avgError)
    )


sim.runSim(ukf)
