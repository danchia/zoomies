import serde
from collections import namedtuple
import math

PathPoint = namedtuple('PathPoint', ['s', 'heading', 'velocity', 'x', 'y'])

pts = []

# sweep a circle of radius R
R = 0.65
SEGMENT_LENGTH = 0.05

t_inc = SEGMENT_LENGTH / R

px, py = 0.0, 0.0
t = 0
while t < 2*math.pi:
    ta = -math.pi/2 + t
    x = R*math.cos(ta)
    y = R+R*math.sin(ta)
    s = t / t_inc * SEGMENT_LENGTH
    heading = math.atan2(y-py, x-px)
    px, py = x, y
    v = 1.0 - math.fabs(0.5 * (t - math.pi)/math.pi)
    pt = PathPoint(s=s, heading=heading, velocity=v, x=x, y=y)
    print(pt)
    pts.append(pt)
    t += t_inc

serde.write(r'C:\tmp\car\path.bin', 0.05, 9.81, pts)
