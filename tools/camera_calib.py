import cv2
import glob
import numpy as np

objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

objp_t = []
grid_size = 0.02
for i in range(0, 11):
  for j in range(0, 4):
    objp_t.append((i * grid_size, (2*j + i%2)*grid_size, 0))
objp_t = np.array(objp_t).astype('float32')
objp = np.zeros((1, 11*4, 3), np.float32)
objp[0,:,:] = objp_t
print(objp)

blob_params = cv2.SimpleBlobDetector_Params()
blob_params.minThreshold = 8
blob_params.maxThreshold = 255
blob_params.thresholdStep = 10

# Filter by Area.
blob_params.filterByArea = True
blob_params.minArea = 64 
blob_params.maxArea = 2500

# Filter by Circularity
blob_params.filterByCircularity = False
blob_params.minCircularity = 0.05

# Filter by Convexity
blob_params.filterByConvexity = False
blob_params.minConvexity = 0.87

# Filter by Inertia
blob_params.filterByInertia = False
blob_params.minInertiaRatio = 0.01

# Create a detector with the parameters
blob_detector = cv2.SimpleBlobDetector_create(blob_params)


fnames = glob.glob('../data/calib/*.jpg')
for fname in fnames:
  img = cv2.imread(fname)
  img_shape = img.shape[:2]

  gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
  # Find the chess board corners
  ret, centers = cv2.findCirclesGrid(gray, (4, 11), flags = cv2.CALIB_CB_ASYMMETRIC_GRID, blobDetector = blob_detector)
  # If found, add object points, image points (after refining them)
  if ret == True:
    print(fname, "circles found")
    objpoints.append(objp)
    imgpoints.append(centers)
  else:
    print(fname, "circles not found")
  cv2.drawChessboardCorners(img, (4, 11), centers, ret)
  img = cv2.resize(img, (int(img.shape[1] * 0.5), int(img.shape[0] * 0.5)))
  cv2.imshow(fname, img)
  cv2.waitKey(50)

# calculate K & D
N_imm = len(objpoints)
K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = [np.zeros((1, 1, 3), dtype=np.float32) for i in range(N_imm)]
tvecs = [np.zeros((1, 1, 3), dtype=np.float32) for i in range(N_imm)]
# need this to get good RMS. Why?
calibration_flags = (
    cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
#    + cv2.fisheye.CALIB_FIX_SKEW
#    + cv2.fisheye.CALIB_CHECK_COND
#    + cv2.fisheye.CALIB_FIX_K2
#    + cv2.fisheye.CALIB_FIX_K3
#    + cv2.fisheye.CALIB_FIX_K4
)
retval, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
    objpoints,
    imgpoints,
    gray.shape[::-1],
    K,
    D,
    rvecs,
    tvecs,
    calibration_flags)
print("Camera calibrated. rms: ", retval)
np.save('../data/calib/camera_K', K, False)
np.save('../data/calib/camera_D', D, False)

print("Camera K: ", K)

cv2.waitKey(10000)
cv2.destroyAllWindows()