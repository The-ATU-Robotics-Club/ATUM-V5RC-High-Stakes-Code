from collections import namedtuple
import numpy as np
import random

R = 3.84
kT = 0.198
kV = 6.68

State = namedtuple("State", ("x", "y", "h", "vl", "vr"))
Input = namedtuple("Input", ("voltL", "voltR"))


class RobotModel:
    def __init__(self, initial: State, n, rb, rw, J, G, m, dt):
        # Everything is in meters, internally.
        C1 = -(G * G * kT * n) / (kV * R * rw * rw)
        C2 = (G * kT * n) / (R * rw)
        self.D1 = (1 / m + rb * rb / J) * C1
        self.D2 = (1 / m - rb * rb / J) * C1
        self.D3 = (1 / m + rb * rb / J) * C2
        self.D4 = (1 / m - rb * rb / J) * C2
        self.rb = rb
        self.dt = dt
        self.state = initial

    def step(self, input: Input):
        self.state = self.F(self.state, input)
        return self.state

    def F(self, state: State, input: Input):
        x, y, h, vl, vr = state
        voltL, voltR = input
        dh = self.dt * (0.5 * (vl - vr) / self.rb)
        hAvg = h + 0.5 * dh
        x += self.dt * 0.5 * (np.sin(hAvg) * vl + np.sin(hAvg) * vr)
        y += self.dt * 0.5 * (np.cos(hAvg) * vl + np.cos(hAvg) * vr)
        h += dh
        vl += self.dt * (
            self.D1 * vl + self.D2 * vr + self.D3 * voltL + self.D4 * voltR
        )
        vr += self.dt * (
            self.D2 * vl + self.D1 * vr + self.D4 * voltL + self.D3 * voltR
        )
        return State(x, y, h, vl, vr)

    def getState(self, xVar=0, yVar=0, hVar=0):
        x = random.gauss(self.state.x, xVar**2)
        y = random.gauss(self.state.y, yVar**2)
        h = random.gauss(self.state.h, hVar**2)
        return State(x, y, h, self.state.vl, self.state.vr)