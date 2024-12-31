import matplotlib.pyplot as plt
import numpy as np
import timeit


def distance(a, b):
    return (((a[0] - b[0]) ** 2) + ((a[1] - b[1]) ** 2)) ** 0.5


class CubicHermite:
    def __init__(
        self,
        startPosition,
        startVelocity,
        endPosition,
        endVelocity,
        spacing=0.1,
        maxError=0.01,
        scaling=0.75,
    ):
        self.startPosition = startPosition
        self.startVelocity = startVelocity
        self.endPosition = endPosition
        self.endVelocity = endVelocity
        self.spacing = spacing
        self.maxError = maxError
        self.path = []
        self.scaling = scaling

    def generatePath(self):
        self.path = [self.startPosition]
        t0 = 0
        while distance(self.path[-1], self.endPosition) > self.spacing:
            # Inlining helps, but not done here for readability sake. 
            t0 = self._addNextPoint(t0)
        self.path.append(self.endPosition)
        return self.path

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
        twoT2 = 2 * t2
        threeT2 = 3 * t2
        twoT3 = 2 * t3
        return (
            (1 - threeT2 + twoT3) * self.startPosition
            + (t - twoT2 + t3) * self.startVelocity
            + (-t2 + t3) * self.endVelocity
            + (threeT2 - twoT3) * self.endPosition
        )


chSpline = CubicHermite(
    np.array([-1, 0]), np.array([10, 10]), np.array([1, 0]), np.array([10, 10])
)
t0 = timeit.default_timer()
for i in range(100):
    points = chSpline.generatePath()
print(timeit.default_timer() - t0)
xs, ys = zip(*points)
xsSpaced, ysSpaced = zip(*points)
fig, ax = plt.subplots()
ax.scatter(xs, ys)
plt.xlim(-1, 1)
plt.ylim(-1, 1)
plt.show()
