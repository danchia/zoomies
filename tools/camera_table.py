import cv2
import numpy as np
import glob

K = np.load('../data/calib/camera_K.npy')
D = np.load('../data/calib/camera_D.npy')

# Calibrated at full res, but only doing 640 x 480
K[:2] /= 4.05

uv = np.mgrid[:480, :640][[1, 0]].transpose(1, 2, 0).astype(np.float32)
pts = cv2.undistortPoints(uv.reshape(640*480,1,2), K, D)
pts = pts.reshape(480*640,2)
pts = np.append(pts, np.ones((480*640, 1)), axis=1)

R, _ = cv2.Rodrigues(np.array([0, np.radians(29), 0], dtype=np.float32))
pts = np.dot(pts, R.transpose)