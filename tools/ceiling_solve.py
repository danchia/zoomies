import numpy as np
import scipy.optimize

POINTS1 = [
    (282, 375),
    (282, 351),
    (341, 358),
    (288, 170),
]

POINTS2 = [
    (209, 391),
    (204, 364),
    (283, 382),
    (215, 163),
]

ORIGIN = np.array([1.195, 0.0, 0.0], dtype=np.float32)

lut = np.load('../data/calib/camera_lut.npy').reshape((480, 640, 3))


def project(uv):
    p = np.array([lut[x[1]][x[0]] for x in uv], dtype=np.float32)
    p = p / p[:, 2][:, None]
    return p


p1 = project(POINTS1)
p2 = project(POINTS2)


def F(z):
    return np.linalg.norm(p1*z - (ORIGIN+p2*z)) / len(POINTS1)


x = scipy.optimize.broyden1(F, 2.0, f_tol=0.08)
print("Estimated height ", x)

TO_PROJECT = [
    (282, 375),  # UL - 0
    (282, 351),  # UR
    (341, 358),  # LL
    (288, 170),  # UL - 3
    (291, 141),
    (350, 187),
    (306, 55),  # UL - 6
    (116, 380),
    (110, 146),
    (111, 146),
]

ps = project(TO_PROJECT)
print(ps * x)

print(np.linalg.norm(ps[0, :] - ps[3, :]))
print(np.linalg.norm(ps[3, :] - ps[6, :]))
print(np.linalg.norm(ps[7, :] - ps[8, :]))
print("")
print(np.linalg.norm(ps[0, :] - ps[7, :]))
print(np.linalg.norm(ps[9, :] - ps[3, :]))
