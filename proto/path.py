import numpy as np
import sim
import pygame


def clamp(x, constraint):
    if x <= constraint[0]:
        return constraint[0]
    elif x >= constraint[1]:
        return constraint[1]
    return x


def distance(a, b):
    return np.sqrt((b[0][0] - a[0][0]) ** 2 + (b[0][1] - a[0][1]) ** 2)


class TrajectoryGenerator:
    def __init__(self, vMax, aMax, rB, bend, n):
        self.vMax = vMax
        self.aMax = aMax
        self.rB = rB
        self.bend = bend
        self.n = n

    def makeTrajectory(self, waypoints):
        temp = self._constrainedTrajectory(waypoints)
        timedTrajectory = [(0, temp[0])]
        for i in range(1, len(temp)):
            timeDiff = (
                2 * distance(temp[i - 1], temp[i]) /
                (temp[i - 1][0][3] + temp[i][0][3])
            )
            timedTrajectory.append(
                (timedTrajectory[i - 1][0] + timeDiff, temp[i]))
        return timedTrajectory

    def _constrainedTrajectory(self, waypoints):
        curvatures, points = self._makePath(waypoints)
        # Skip beginning and end to keep them zero for velocity.
        for i in range(1, len(points) - 1):
            v1 = np.sqrt(
                points[i - 1][0][3] ** 2
                + 2 * self.aMax * distance(points[i - 1], points[i])
            )
            points[i][0][3] = min(
                v1, self.vMax / np.abs(2 * self.rB * curvatures[i]))
        for i in range(len(points) - 2, 0, -1):
            v1 = np.sqrt(
                points[i + 1][0][3] ** 2
                + 2 * self.aMax * distance(points[i + 1], points[i])
            )
            points[i][0][3] = min(points[i][0][3], v1)
            points[i][0][4] = points[i][0][3] * curvatures[i]
        return points

    def _makePath(self, waypoints):
        curvatures = []
        points = []
        for i in range(len(waypoints) - 1):
            p0 = waypoints[i]
            p3 = waypoints[i + 1]
            h0 = -p0[0][2] + np.pi / 2
            h3 = -p3[0][2] - np.pi / 2
            p1 = p0 + self.bend * np.array([np.cos(h0), np.sin(h0), 0, 0, 0])
            p2 = p3 + self.bend * np.array([np.cos(h3), np.sin(h3), 0, 0, 0])
            for t in np.linspace(0, 1, self.n):
                curvatures.append(self._curvature(t, p0, p1, p2, p3))
                points.append(self._B(t, p0, p1, p2, p3))
        curvatures.append(0)
        points.append(waypoints[-1])
        return curvatures, points

    def _B(self, t, p0, p1, p2, p3):
        point = (
            (1 - t) ** 3 * p0
            + 3 * (1 - t) ** 2 * t * p1
            + 3 * (1 - t) * t**2 * p2
            + t**3 * p3
        )
        pointDer = self._BDer(t, p0, p1, p2, p3)
        point[0][2] = np.pi / 2 - np.arctan2(pointDer[0][1], pointDer[0][0])
        return point

    def _BDer(self, t, p0, p1, p2, p3):
        return (
            3 * (1 - t) ** 2 * (p1 - p0)
            + 6 * (1 - t) * t * (p2 - p1)
            + 3 * t**2 * (p3 - p2)
        )

    def _BDerDer(self, t, p0, p1, p2, p3):
        return 6 * (1 - t) * (p2 - 2 * p1 + p0) + 6 * t * (p3 - 2 * p2 + p1)

    def _curvature(self, t, p0, p1, p2, p3):
        der = np.array([self._BDer(t, p0, p1, p2, p3)[0][:2]])
        derDer = np.array([self._BDerDer(t, p0, p1, p2, p3)[0][:2]])
        numerator = der[0][1] * derDer[0][0] - der[0][0] * derDer[0][1]
        denom = np.sqrt(der[0][0]**2 + der[0][1]**2)
        return numerator / denom ** 3


class PIDF:
    def __init__(self, kP, kI, kD, ff, constraints):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.ff = ff
        self.constraints = constraints
        self.prevState = 0
        self.I = 0

    def getOutput(self, state, reference):
        # Probably want to later check for sample time
        error = reference - state
        P = self.kP * error
        self.I += self.kI * error
        self.I = clamp(self.I, self.constraints)
        D = self.kD * (self.prevState - state)
        self.prevState = state
        F = self.ff * reference
        output = P + self.I + D + F
        return clamp(output, self.constraints)


class RAMSETE:
    def __init__(self, model, b, l, lCtrl, rCtrl, rB, rW):
        self.model = model
        self.b = b
        self.l = l
        self.lCtrl = lCtrl
        self.rCtrl = rCtrl
        self.rB = rB
        self.rW = rW

    # Want it to follow full path later
    def follow(self, point):
        v, w = self._getOutput(self.model.state, point)
        vLin = v
        vAng = w * self.rB
        print((v, w))
        left = self.lCtrl.getOutput(self.model.state[3], vLin + vAng)
        right = self.rCtrl.getOutput(self.model.state[4], vLin - vAng)
        self.model.step((left, right))

    def _getOutput(self, state, desired):
        error = desired - state
        print(state)
        tempA = error[0][0] * np.sin(state[2]) + error[0][1] * np.cos(state[2])
        tempB = error[0][0] * np.cos(state[2]) - error[0][1] * np.sin(state[2])
        error[0][0] = tempA
        error[0][1] = tempB
        error[0][2] = ((error[0][2] + np.pi) % (2 * np.pi)) - np.pi
        k = 2 * self.l * np.sqrt(desired[0][4]
                                 ** 2 + self.b * desired[0][3] ** 2)
        v = desired[0][3] * np.cos(error[0][2]) + k * error[0][0]
        sincHErr = 1 if error[0][2] == 0 else np.sin(error[0][2]) / error[0][2]
        w = (
            desired[0][4]
            + k * error[0][2]
            + self.b * desired[0][3] * sincHErr * error[0][1]
        )
        return v, w


trajectoryGenerator = TrajectoryGenerator(2.4, 1.2, 0.19, 0.5, 50)
# Points are (t, <x, y, h, v, w>); v and w are all initially 0.
points = trajectoryGenerator.makeTrajectory(
    [
        np.array([[0.25, 0, 0, 0, 0]]),
        np.array([[0.5, 1.5, np.pi / 2, 0, 0]]),
        np.array([[1, -1.5, np.pi, 0, 0]]),
        np.array([[-1, -1.5, -np.pi / 2, 0, 0]]),
        np.array([[-1.5, 1.5, np.pi / 2, 0, 0]]),
        np.array([[0, -1, np.pi / 2, 0, 0]]),
        np.array([[1.65, 1.65, 0, 0, 0]]),
        np.array([[-1.65, 1.25, -5 * np.pi / 4, 0, 0]]),
        np.array([[0, 0, 0, 0, 0]])
    ]
)

_, rawPoints = zip(*points)

leftControl = PIDF(15, 0, 0, 4.5, (-12, 12))
rightControl = PIDF(15, 0, 0, 4.5, (-12, 12))

driveControl = RAMSETE(sim.robotModel, 2, 0.7,
                       leftControl, rightControl, 0.19, 0.0508)


time = 0


def path():
    global time
    time += 0.01
    if len(points) > 1 and time >= points[1][0]:
        points.pop(0)
    desired = points[0][1]
    driveControl.follow(desired)
    for i in range(len(rawPoints)):
        x, y = sim.center(rawPoints[i][0][0], rawPoints[i][0][1])
        h = np.pi / 2 - rawPoints[i][0][2]
        xEnd = x + 20 * np.cos(h)
        yEnd = y - 20 * np.sin(h)
        pygame.draw.line(sim.screen, sim.red, (x, y), (xEnd, yEnd), 3)
    sim.robotView.draw()


time = 0
sim.runSim(path)
