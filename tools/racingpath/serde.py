import struct


def write(fname, segment_length, pts):
    with open(fname, "wb") as f:
        f.write(struct.pack('<if', len(pts), segment_length))

        for pt in pts:
            f.write(struct.pack('<fffff', pt.s,
                    pt.heading, pt.velocity, pt.x, pt.y))
