import sqlite3
import csv

npix_y = 640*480
npix_uv = 320*240

tstamps = dict()


def vid_reader(vidfile):
    while True:
        fbytes = vidfile.read(8)
        if not fbytes:
            break
        fno = int.from_bytes(fbytes, 'little', signed=True)
        # HACK HACK
        if fno == 1301:
            break
        tstamp = tstamps[fno]

        fbytes = vidfile.read(4)
        if not fbytes:
            break
        vid_len = int.from_bytes(fbytes, 'little', signed=True)

        if vid_len != npix_y + 2*npix_uv:
            vidfile.read(vid_len)
            continue

        vbytes = vidfile.read(npix_y+npix_uv*2)

        yield (tstamp, vbytes)


SCHEMA = '''
pragma journal_mode = WAL;
pragma synchronous = normal;

CREATE TABLE videos (
  t_us INTEGER NOT NULL,
  data BLOB NOT NULL
);

CREATE TABLE data (
  t_us INTEGER NOT NULL,
  odo_dist_delta REAL,
  odo_heading_delta REAL,
  imu_linear_x REAL,
  imu_linear_y REAL,
  imu_linear_z REAL,
  imu_rot_x REAL,
  imu_rot_y REAL,
  imu_rot_z REAL
);
'''

con = sqlite3.connect(r'c:\tmp\car\1401\data.db3')
con.executescript(SCHEMA)
con.commit()

with open(r'c:\tmp\car\1401\datalog.csv') as csvfile:
    reader = csv.DictReader(csvfile)
    last_dist = 0.0
    fno = 1
    for row in reader:
        t_us = int(row['t_micros'])
        tstamps[fno] = t_us
        fno += 1
        dist = float(row['total_distance'])
        dist_delta = dist - last_dist
        last_dist = dist
        heading_delta = float(row['heading'])
        rot_z = heading_delta * 100
        con.execute('INSERT INTO data(t_us, odo_dist_delta, odo_heading_delta, '
                    'imu_linear_x, imu_linear_y, imu_linear_z, imu_rot_x, imu_rot_y, imu_rot_z) '
                    'VALUES ( ?, ?, ?, ?, ?, ?, ?, ?, ?)',
                    (t_us, dist_delta, heading_delta, 0, 0, 0, 0, 0, rot_z))

con.commit()

with open(r'c:\tmp\car\1401\vidlog.bin', 'rb') as vidfile:
    con.executemany("INSERT INTO videos(t_us, data) VALUES (?, ?)",
                    vid_reader(vidfile))
con.commit()

con.close()
