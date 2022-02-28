import numpy as np
import math
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt


class Track:
    def __init__(self):
        pass

    def racing_line_from_points(self, pts, smooth_factor):
        # racing line is interpolated from a cubic spline fit against pts.
        tck, u = splprep(pts, per=True, s=smooth_factor)
        self.u = u
        self.tck = tck
        self.pts = splev(u, tck)
        # S[idx] == distance between pt at idx, and previous point.
        self.S = [0.0]
        for idx in range(1, len(self.pts[0])):
            x, y = self.pts[0][idx], self.pts[1][idx]
            px, py = self.pts[0][idx-1], self.pts[1][idx-1]
            s = math.sqrt((x-px)**2 + (y-py)**2)
            self.S.append(s)
        self.S[0] = self.S[-1]
        self.total_s = sum(self.S)

        pts_d = splev(u, tck, der=1)
        pts_dd = splev(u, tck, der=2)
        self.K = (pts_d[0]*pts_dd[1] - pts_d[1]*pts_dd[0]) / \
            np.power(np.square(pts_d[0]) + np.square(pts_d[1]), 1.5)

    def set_left_boundary(self, pts):
        self.left_boundary = pts

    def set_right_boundary(self, pts):
        self.right_boundary = pts

    def boundary_dist(self, margin):
        """Returns left, right distances to road boundary, removing margin distance on each side"""
        left = _calc_min_dist(self.pts, self.left_boundary, margin)
        right = _calc_min_dist(self.pts, self.right_boundary, margin)
        return left, right

    def display(self):
        plt.figure()
        plt.plot(self.pts[0], self.pts[1], 'k')
        plt.scatter(
            self.left_boundary[0], self.left_boundary[1], s=1, marker='.', c='g')
        plt.scatter(
            self.right_boundary[0], self.right_boundary[1], s=1, marker='.', c='b')
        plt.show()


def _calc_min_dist(racing_pts, boundary_pts, margin):
    res = []
    for idx in range(len(racing_pts[0])):
        x, y = racing_pts[0][idx], racing_pts[1][idx]
        xdiff_sq = np.square(boundary_pts[0] - x)
        ydiff_sq = np.square(boundary_pts[1] - y)
        dists_sq = xdiff_sq + ydiff_sq
        min_dist = math.sqrt(np.min(dists_sq))
        res.append(max(min_dist - margin, 0))
    return res
