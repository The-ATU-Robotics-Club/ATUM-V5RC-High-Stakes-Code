import matplotlib.pyplot as plt
import numpy as np
import timeit


def distance(a, b):
    return (((a[0] - b[0]) ** 2) + ((a[1] - b[1]) ** 2)) ** 0.5


class CubicHermite:
    def __init__(
        self,
        startPosition,
        startDirection,
        endPosition,
        endDirection,
        track,
        maxVelocity,
        acceleration,
        spacing=0.01,
        maxError=0.001,
        scaling=0.75,
    ):
        self.startPosition = startPosition
        self.startVelocity = startDirection
        self.endPosition = endPosition
        self.endVelocity = endDirection
        self.track = track
        self.halfTrack = track / 2
        self.maxVelocity = maxVelocity
        self.acceleration = acceleration
        self.spacing = spacing
        self.maxError = maxError
        self.scaling = scaling
        self.path = []
        self.times = []
        self.headings = []
        self.curvatures = []
        self.velocity = []
        self.angularVelocity = []

    def generatePaths(self):
        self.path = [self.startPosition]
        self.curvatures = [self._getCurvature(0)]
        t0 = 0
        while distance(self.path[-1], self.endPosition) > self.spacing + self.maxError:
            # Inlining helps, but not done here for readability sake.
            t0 = self._addNextPoint(t0)
            self.curvatures.append(self._getCurvature(t0))
        self.path.append(self.endPosition)
        self.curvatures.append(self._getCurvature(1))

    def generateTrajectory(self):
        self.generatePaths()
        self.velocity = [
            min(
                self.maxVelocity,
                self.maxVelocity / (abs(self.curvatures[i]) * self.track),
            )
            for i in range(len(self.path))
        ]
        self.velocity[0] = self.velocity[-1] = 0
        for i in range(1, len(self.path) - 2):
            twoA = 2 * self.acceleration
            accelerated = self.velocity[i - 1] * self.velocity[
                i - 1
            ] + twoA * distance(self.path[i - 1], self.path[i])
            self.velocity[i] = min(np.sqrt(accelerated), self.velocity[i])
            j = len(self.path) - 1 - i
            
            accelerated = self.velocity[j + 1] * self.velocity[
                j + 1
            ] + twoA * distance(self.path[j + 1], self.path[j])
            self.velocity[j] = min(np.sqrt(accelerated), self.velocity[j])
        self.times = [0]
        self.angularVelocity = [0]
        for i in range(1, len(self.path)):
            avgV = (self.velocity[i - 1] + self.velocity[i]) / 2
            self.times.append(self.times[-1] + self.spacing / avgV)
            self.angularVelocity.append(self.velocity[i] * self.curvatures[i])
        return [self.path, self.times, self.velocity, self.angularVelocity]

    def _addNextPoint(self, t0):
        t2 = 1
        t1 = t0 * self.scaling + t2 * (1 - self.scaling)
        p0 = self.path[-1]
        p1 = self._getPoint(t1)
        distanceToNext = distance(p0, p1)
        while abs(distanceToNext - self.spacing) > self.maxError:
            if distanceToNext > self.spacing:
                t2 = t1
            else:
                t0 = t1
            t1 = t0 * self.scaling + t2 * (1 - self.scaling)
            p1 = self._getPoint(t1)
            distanceToNext = distance(p0, p1)
        self.path.append(p1)
        return t1

    def _getPoint(self, t):
        t2 = t * t
        t3 = t2 * t
        threeT2 = 3 * t2
        twoT3 = 2 * t3
        return (
            (twoT3 - threeT2 + 1) * self.startPosition
            + (t3 - 2 * t2 + t) * self.startVelocity
            + (-twoT3 + threeT2) * self.endPosition
            + (t3 - t2) * self.endVelocity
        )

    def _getCurvature(self, t):
        derivative = self._getDerivative(t)
        derivative2nd = self._get2ndDerivative(t)
        cross = derivative[0] * derivative2nd[1] - derivative[1] * derivative2nd[0]
        denom = pow(derivative[0] * derivative[0] + derivative[1] * derivative[1], 1.5)
        if cross == 0:
            cross = 0.00000001  # This is an easy fix for now, don't do this in C++
        return -cross / denom

    def _getDerivative(self, t):
        t2 = t * t
        t3 = t2 * t
        threeT2 = 3 * t2
        return (
            (6 * t2 - 6 * t) * (self.startPosition - self.endPosition)
            + (threeT2 - 4 * t + 1) * self.startVelocity
            + (threeT2 - 2 * t) * self.endVelocity
        )

    def _get2ndDerivative(self, t):
        sixT = 6 * t
        return (
            (12 * t - 6) * (self.startPosition - self.endPosition)
            + (sixT - 4) * self.startVelocity
            + (sixT - 2) * self.endVelocity
        )


# 76.5 in / s => 1.94 m/s
chSpline = CubicHermite(
    np.array([-1, 0]),
    np.array([0, 15]),
    np.array([1, 0]),
    np.array([0, 15]),
    0.35,
    1.94,
    1.94,
)
t0 = timeit.default_timer()
path, times, velocity, angularVelocity = chSpline.generateTrajectory()
print(timeit.default_timer() - t0)
xsPath, ysPath = zip(*path)
fig, ax1 = plt.subplots()
ax1.scatter(xsPath, ysPath)
ax1.set_xlim([-1.5, 1.5])
ax1.set_ylim([-1.5, 5])

_, ax2 = plt.subplots()
ax2.scatter(times, velocity)
ax2.set_xlim([0, 6])
ax2.set_ylim([-2, 2])

_, ax3 = plt.subplots()
ax3.scatter(times, angularVelocity)
ax3.set_xlim([0, 6])
ax3.set_ylim([-15, 15])
plt.show()
