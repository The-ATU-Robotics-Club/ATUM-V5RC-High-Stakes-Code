import matplotlib.pyplot as plt
from collections import namedtuple

MotionStep = namedtuple("MotionStep", "x v a t")


class TrapezoidalProfile:
    def __init__(self, maxV=0, maxA=0, dt=0.01):
        self.maxV = maxV
        self.maxA = maxA
        self.dt = dt

    def profile(self, desired, maxV=None, maxA=None, dt=None):
        maxV, maxA, dt = self._setConstraints(maxV, maxA, dt)
        profile = [MotionStep(0, 0, maxA, 0)]
        while profile[-1].x < desired / 2:
            profile.append(self._nextStep(profile[-1], maxA, maxV, dt))
        firstHalf = len(profile)
        for i in range(firstHalf):
            step = profile[firstHalf - 1 - i]
            x = profile[-1].x + step.v * dt + 0.5 * -step.a * dt * dt
            profile.append(MotionStep(x, step.v, -step.a, profile[-1].t + dt))
        return profile

    def _setConstraints(self, maxV, maxA, dt):
        if (not maxV):
            maxV = self.maxV
        if (not maxA):
            maxA = self.maxA
        if (not dt):
            dt = self.dt
        return (maxV, maxA, dt)

    def _nextStep(self, previous, a, maxV, dt):
        x = previous.x + previous.v * dt + 0.5 * a * dt * dt
        v = min(maxV, previous.v + a * dt)
        if v == maxV:
            a = 0
        t = previous.t + dt
        #print(MotionStep(x, v, a, t))
        return MotionStep(x, v, a, t)


trapezoidalProfiler = TrapezoidalProfile(610, 1830, 0.01)
profile = trapezoidalProfiler.profile(90)
x, v, a, t = zip(*profile)
print(x)
fig, (ax1, ax2, ax3) = plt.subplots(3)
ax1.set_title('Position')
ax1.plot(t, x)
ax2.set_title('Velocity')
ax2.plot(t, v)
ax3.set_title('Acceleration')
ax3.plot(t, a)
plt.show()