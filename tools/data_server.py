import asyncio
import os
import sys
import time
from base64 import standard_b64encode
from foxglove_websocket import run_cancellable
from foxglove_websocket.examples.proto import ExampleMsg_pb2
from foxglove_websocket.server import FoxgloveServer, FoxgloveServerListener
from foxglove_websocket.types import ChannelId
import cv2
import numpy as np
from ros_msgs.ros.sensor_msgs import CompressedImage_pb2
from ros_msgs.ros.std_msgs import Header_pb2
from ros_msgs.ros import builtins_pb2


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


async def main():
    class Listener(FoxgloveServerListener):
        def on_subscribe(self, server: FoxgloveServer, channel_id: ChannelId):
            print("First client subscribed to", channel_id)

        def on_unsubscribe(self, server: FoxgloveServer, channel_id: ChannelId):
            print("Last client unsubscribed from", channel_id)

    # Load the FileDescriptorSet, which was generated via `protoc --descriptor_set_out`.
    with open(
        os.path.join('ros_msgs', 'descriptor.bin'), "rb"
    ) as schema_bin:
        schema_base64 = standard_b64encode(schema_bin.read()).decode("ascii")

    async with FoxgloveServer("0.0.0.0", 8765, "example server") as server:
        server.set_listener(Listener())
        chan_id = await server.add_channel(
            {
                "topic": "/image_color/compressed",
                "encoding": "protobuf",
                "schemaName": "ros.sensor_msgs.CompressedImage",
                "schema": schema_base64,
            }
        )

        with open(r'c:\tmp\car\1401\vidlog.bin', 'rb') as vidfile:
            for (tstamp, img_distorted, img_recti) in vid_reader(vidfile):
                await asyncio.sleep(0.2)

                r, ibytes = cv2.imencode('.png', img_distorted)
                if not r:
                    raise Exception('imencode failed')

                stamp = builtins_pb2.Time(
                    sec=tstamp // 1000000, nsec=(tstamp % 1000000) * 1000)
                header = Header_pb2.Header(stamp=stamp, frame_id='camera')
                msg = CompressedImage_pb2.CompressedImage(
                    header=header, format='png', data=ibytes.tobytes())

                await server.send_message(
                    chan_id,
                    tstamp * 1000, msg.SerializeToString())


if __name__ == "__main__":
    run_cancellable(main())
