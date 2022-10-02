import struct


def write(fname, segment_length, accel, pts):
    with open(fname, "wb") as f:
        f.write(struct.pack('<iff', len(pts), segment_length, accel))

        for pt in pts:
            f.write(struct.pack('<ffffff', pt.s,
                    pt.heading, pt.curvature, pt.velocity, pt.x, pt.y))
