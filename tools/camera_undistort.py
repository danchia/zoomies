import cv2
import numpy as np
import glob

K = np.load('../data/calib/camera_K.npy')
D = np.load('../data/calib/camera_D.npy')

fnames = glob.glob('../data/calib/*.jpg')
for fname in fnames:
  img = cv2.imread(fname)

  img = img
  cv2.fisheye.undistortImage(img, K, D, img, K)

  img = cv2.resize(img, (int(img.shape[1] * 0.5), int(img.shape[0] * 0.5)))
  cv2.imshow(fname, img)
  cv2.waitKey(50)

cv2.waitKey()
cv2.destroyAllWindows()