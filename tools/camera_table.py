import cv2
import numpy as np

CAMERA_OFFSET = 0.116  # camera is in front of CG along x.

K = np.load('../data/calib/camera_K.npy')
D = np.load('../data/calib/camera_D.npy')

# Calibrated at full res, but only doing 640 x 480
K[:2] /= 4.05

uv = np.mgrid[:480, :640][[1, 0]].transpose(1, 2, 0).astype(np.float32)
pts = cv2.fisheye.undistortPoints(uv.reshape(640*480, 1, 2), K, D)
pts = pts.reshape(480*640, 2)
pts_cam = pts
pts = np.append(pts, np.ones((480*640, 1)), axis=1)

R, _ = cv2.Rodrigues(np.array([0, np.radians(30), 0], dtype=np.float32))
pts = np.dot(pts, R.transpose())

# pts = pts / np.linalg.norm(pts, axis=1, keepdims=True)
# normalize z=1
pts = pts / np.absolute(pts[:, 2][:, None])
np.save('../data/calib/camera_lut.npy', pts, False)

lut = pts[:, 0:2]

lut_offset = np.copy(lut)
lut_offset[:, 0] += CAMERA_OFFSET
lut_1d = lut_offset.reshape(480*640*2).astype(np.float32)
f = open('../data/calib/camera_lut.bin', 'wb')
f.write(lut_1d.tobytes())
f.close()

ceil_mask = np.zeros(480*640, dtype=np.uint8)
# * We want ceiling, so mask out floor (neg z).
# * OpenCV fisheye can't handle past 180, and close to 180 deg it's quite poor. acos(1/10) ~ 84deg
# * Also, mask out really far ceiling pts. tan(70deg) ~ 2.8
ceil_mask[(pts[:, 2] > 0) & (np.sum(
    pts_cam**2, axis=1) < 100) & (np.sum(lut**2, axis=1) < 2.8**2)] = 255

f = open('../data/calib/ceil_mask.bin', 'wb')
f.write(ceil_mask.reshape(480*640).tobytes())
f.close()

floor_mask = np.zeros(480*640, dtype=np.uint8)
floor_mask[(pts[:, 2] < 0) & (np.sum(
    pts_cam**2, axis=1) < 100) & (np.sum(lut**2, axis=1) < 10**2)] = 255

f = open('../data/calib/floor_mask.bin', 'wb')
f.write(floor_mask.reshape(480*640).tobytes())
f.close()
