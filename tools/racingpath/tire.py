import numpy as np
import math


def fiala_tire_model(Fz, miu, C):
    alpha = []
    force = []

    thres = math.atan(3*miu*Fz/C)
    alpha = np.linspace(np.radians(-10), np.radians(10), 2000)
    for a in alpha:
        if abs(a) < thres:
            tan_a = math.tan(a)
            force.append(-C*tan_a + C**2*abs(tan_a)*tan_a /
                         (3*miu*Fz) - C**3 * (tan_a**3) / (27*miu**2*Fz**2))
        else:
            force.append(-miu*Fz*np.sign(a))

    return alpha, force


def fiala_lookup(alpha, force, F):
    """Returns alpha_hat, C_hat linearized model"""
    idx = np.searchsorted(force, F, sorter=np.arange(len(force)-1, -1, -1))
    idx = len(force) - 1 - idx
    if idx >= len(force) - 2:
        idx = len(force)-2
    elif idx < 0:
        idx = 0
    return alpha[idx], -(force[idx+1] - force[idx]) / (alpha[idx+1] - alpha[idx])
