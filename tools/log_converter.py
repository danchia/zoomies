import cv2
import numpy as np
from rosbags.rosbag2 import Writer
from rosbags.serde import serialize_cdr
import rosbags.typesys.types as rtypes

npix_y = 640*480
npix_uv = 320*240

K = np.load('../data/calib/camera_K.npy')
# Calibrated at full res, but only doing 640 x 480
K[:2] /= 4.05
D = np.load('../data/calib/camera_D.npy')

def vid_reader(vidfile):
  while True:
    fbytes = vidfile.read(8)
    if not fbytes:
      break
    fno = int.from_bytes(fbytes, 'little', signed=True)
    tstamp = fno * 10000

    fbytes = vidfile.read(4)
    if not fbytes:
      break
    vid_len = int.from_bytes(fbytes, 'little', signed=True)

    if vid_len != npix_y + 2*npix_uv:
      vidfile.read(vid_len)
      continue

    vbytes = vidfile.read(npix_y+npix_uv*2)
    vbytes = np.frombuffer(vbytes, np.uint8)

    rgb = cv2.cvtColor(vbytes.reshape(-1, 640), cv2.COLOR_YUV2BGR_I420)
    recti = np.zeros(rgb.shape)
    cv2.fisheye.undistortImage(rgb, K, D, recti, K)
    # y = vbytes[0:npix_y].reshape((480,640)))
    # u = vbytes[npix_y:(npix_y+npix_uv)].reshape((240,320)))
    # v = vbytes[(npix_y+npix_uv):].reshape((240,320)))

    yield (tstamp, cv2.rotate(rgb, cv2.ROTATE_90_CLOCKWISE), cv2.rotate(recti, cv2.ROTATE_90_CLOCKWISE))

with Writer('rosbag_test') as writer:
    image_topic = '/image_color/compressed'
    msgtype = rtypes.sensor_msgs__msg__CompressedImage.__msgtype__
    image_connection = writer.add_connection(image_topic, msgtype, 'cdr', '')

    with open(r'c:\tmp\car\1401\vidlog.bin', 'rb') as vidfile:
      for (tstamp, img_distorted, img_recti) in vid_reader(vidfile):
        stamp = rtypes.builtin_interfaces__msg__Time(tstamp // 1000000, (tstamp % 1000000) * 1000)
        header = rtypes.std_msgs__msg__Header(stamp, 'camera')
        r, ibytes = cv2.imencode('.png', img_distorted)
        if not r: raise Exception('imencode failed')
        message = rtypes.sensor_msgs__msg__CompressedImage(header, 'png', ibytes)
        writer.write(image_connection, tstamp*1000, serialize_cdr(message, msgtype))