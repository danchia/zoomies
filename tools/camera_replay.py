import cv2
import numpy as np

npix_y = 640*480
npix_uv = 320*240
vidfile = open(r'c:\tmp\car\1401\vidlog.bin', 'rb')

while True:
  fbytes = vidfile.read(8)
  if not fbytes:
    break
  fno = int.from_bytes(fbytes, 'little', signed=True)

  fbytes = vidfile.read(4)
  if not fbytes:
    break
  vid_len = int.from_bytes(fbytes, 'little', signed=True)

  print("Frame ", fno, " len ", vid_len)
  if vid_len != npix_y + 2*npix_uv:
    vidfile.read(vid_len)
    continue

  vbytes = vidfile.read(npix_y+npix_uv*2)
  vbytes = np.frombuffer(vbytes, np.uint8)

  rgb = cv2.cvtColor(vbytes.reshape(-1, 640), cv2.COLOR_YUV2BGR_I420)

  cv2.imshow('Y', vbytes[0:npix_y].reshape((480,640)))
  cv2.imshow('U', vbytes[npix_y:(npix_y+npix_uv)].reshape((240,320)))
  cv2.imshow('V', vbytes[(npix_y+npix_uv):].reshape((240,320)))
  cv2.imshow('BGR', rgb)
  if cv2.waitKey() & 0xFF == ord('q'):
    break

cv2.destroyAllWindows()
