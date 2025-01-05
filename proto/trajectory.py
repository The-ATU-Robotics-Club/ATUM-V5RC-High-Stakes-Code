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
        curve,
        track,
        maxVelocity,
        acceleration,
        spacing=0.01,
        maxError=0.001,
        scaling=0.75,
    ):
        self.startPosition = startPosition
        startDirection = np.pi / 2 - startDirection
        self.startDirection = curve * \
            np.array([np.cos(startDirection), np.sin(startDirection)])
        self.endPosition = endPosition
        endDirection = -np.pi / 2 + endDirection
        self.endDirection = curve * \
            np.array([np.cos(endDirection), np.sin(endDirection)])
        self.track = track
        self.halfTrack = track / 2
        self.maxVelocity = maxVelocity
        self.acceleration = acceleration
        self.spacing = spacing
        self.maxError = maxError
        self.scaling = scaling
        self.path = []
        self.times = []
        self.derivatives = []
        self.headings = []
        self.curvatures = []
        self.velocity = []
        self.angularVelocity = []

    def generateTrajectory(self):
        self.generatePaths()
        self.velocity = [
            min(
                self.maxVelocity,
                # Maybe multiply by 2?
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
            dt = self.spacing / avgV
            self.times.append(self.times[-1] + dt)
            self.angularVelocity.append(
                np.pi / 180 * (self.headings[i] - self.headings[i - 1]) / dt)
        return [self.path, self.headings, self.times, self.velocity, self.angularVelocity]

    def generatePaths(self):
        self.path = [self.startPosition]
        self.derivatives = [self._getDerivative(0)]
        self.headings = [self._getHeading(0)]
        self.curvatures = [self._getCurvature(0)]
        t0 = 0
        while distance(self.path[-1], self.endPosition) > self.spacing + self.maxError:
            # Inlining helps, but not done here for readability sake.
            t0 = self._addNextPoint(t0)
            self.derivatives = [self._getDerivative(t0)]
            self.headings.append(self._getHeading(t0))
            self.curvatures.append(self._getCurvature(t0))
        self.path.append(self.endPosition)
        self.derivatives.append(self._getDerivative(1))
        self.headings.append(self._getHeading(1))
        self.curvatures.append(self._getCurvature(1))

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
            + (t3 - 2 * t2 + t) * self.startDirection
            + (-twoT3 + threeT2) * self.endPosition
            + (t3 - t2) * self.endDirection
        )

    def _getHeading(self, t):
        derivative = self.derivatives[-1]
        return (np.pi / 2 - np.arctan2(derivative[1], derivative[0])) * 180 / np.pi

    def _getCurvature(self, t):
        derivative = self.derivatives[-1]
        derivative2nd = self._get2ndDerivative(t)
        cross = derivative[0] * derivative2nd[1] - \
            derivative[1] * derivative2nd[0]
        if cross == 0:
            return 0.00000001  # This is an easy fix for now, don't do this in C++
        denom = pow(derivative[0] * derivative[0] +
                    derivative[1] * derivative[1], 1.5)
        return -cross / denom

    def _getDerivative(self, t):
        t2 = t * t
        threeT2 = 3 * t2
        return (
            (6 * t2 - 6 * t) * (self.startPosition - self.endPosition)
            + (threeT2 - 4 * t + 1) * self.startDirection
            + (threeT2 - 2 * t) * self.endDirection
        )

    def _get2ndDerivative(self, t):
        sixT = 6 * t
        return (
            (12 * t - 6) * (self.startPosition - self.endPosition)
            + (sixT - 4) * self.startDirection
            + (sixT - 2) * self.endDirection
        )


# 76.5 in / s => 1.94 m/s
chSpline = CubicHermite(
    np.array([-2.5, 2.5]),
    np.pi / 2,
    np.array([-0.5, 0.5]),
    np.pi / 2,
    5,
    0.35,
    1.94,
    1.94,
)
t0 = timeit.default_timer()
for i in range(100):
    path, headings, times, velocity, angularVelocity = chSpline.generateTrajectory()
print(timeit.default_timer() - t0)
xsPath, ysPath = zip(*path)
_, ax1 = plt.subplots()
ax1.scatter(xsPath, ysPath)
ax1.set_xlim([-3, 3])
ax1.set_ylim([-3, 3])

_, ax2 = plt.subplots()
ax2.scatter(times, headings)
ax2.set_xlim([0, 6])
ax2.set_ylim([-180, 180])

_, ax3 = plt.subplots()
ax3.scatter(times, velocity)
ax3.set_xlim([0, 6])
ax3.set_ylim([-2, 2])

_, ax4 = plt.subplots()
ax4.scatter(times, angularVelocity)
ax4.set_xlim([0, 6])
ax4.set_ylim([-15, 15])
plt.show()
