#!/usr/bin/env python3
"""校验 RTK bag 中 /livox/lidar 的 CDR 布局是否与 livox_ros_driver2/msg/CustomMsg 完全一致。

只读若干条消息，按假定布局逐字段解析，要求解析后游标恰好停在报文末尾
（无剩余、不越界）。若全部吻合，则两包名的消息 CDR 字节流等价，
离线转换只需改写类型字符串，无需重新序列化。
"""
import sqlite3
import struct
import sys
import glob

BAG = glob.glob('/home/ywj/slam_data/rtk_slam/construction_seq2/*.db3')[0]


class CDR:
    def __init__(self, buf):
        assert buf[0:2] == b'\x00\x01', 'expect CDR LE encapsulation, got %r' % buf[0:4]
        self.b = buf
        self.o = 4

    def _align(self, n):
        r = (self.o - 4) % n
        if r:
            self.o += n - r

    def i32(self):
        self._align(4)
        v = struct.unpack_from('<i', self.b, self.o)[0]
        self.o += 4
        return v

    def u32(self):
        self._align(4)
        v = struct.unpack_from('<I', self.b, self.o)[0]
        self.o += 4
        return v

    def u64(self):
        self._align(8)
        v = struct.unpack_from('<Q', self.b, self.o)[0]
        self.o += 8
        return v

    def f32(self):
        self._align(4)
        v = struct.unpack_from('<f', self.b, self.o)[0]
        self.o += 4
        return v

    def u8(self):
        v = self.b[self.o]
        self.o += 1
        return v

    def raw(self, n):
        v = self.b[self.o:self.o + n]
        self.o += n
        return v

    def string(self):
        n = self.u32()
        s = self.b[self.o:self.o + n]
        self.o += n
        return s.rstrip(b'\x00').decode('utf-8', 'replace')


def parse(buf):
    c = CDR(buf)
    sec = c.i32()
    nsec = c.u32()
    frame_id = c.string()
    timebase = c.u64()
    point_num = c.u32()
    lidar_id = c.u8()
    rsvd = c.raw(3)
    n = c.u32()
    pts = []
    for _ in range(n):
        off = c.u32()
        x, y, z = c.f32(), c.f32(), c.f32()
        refl, tag, line = c.u8(), c.u8(), c.u8()
        if len(pts) < 4:
            pts.append((off, x, y, z, refl, tag, line))
    rest = len(buf) - c.o
    return dict(sec=sec, nsec=nsec, frame_id=frame_id, timebase=timebase,
                point_num=point_num, seq_len=n, lidar_id=lidar_id,
                rsvd=list(rsvd), pts=pts, rest=rest, total=len(buf))


def main():
    con = sqlite3.connect(BAG)
    tid = con.execute("select id from topics where name='/livox/lidar'").fetchone()[0]
    rows = con.execute(
        "select id,timestamp,data from messages where topic_id=? order by timestamp limit 5",
        (tid,)).fetchall()
    print('sampled %d messages from /livox/lidar' % len(rows))
    bad = 0
    for mid, ts, data in rows:
        try:
            r = parse(data)
        except Exception as e:
            print('  msg %s PARSE FAIL: %r' % (mid, e))
            bad += 1
            continue
        ok = (r['rest'] == 0)
        bad += 0 if ok else 1
        print('  msg %-7s ts=%d  bytes=%d  frame=%r  timebase=%d  point_num=%d  seq_len=%d  '
              'lidar_id=%d rsvd=%s  rest=%d %s'
              % (mid, ts, r['total'], r['frame_id'], r['timebase'], r['point_num'],
                 r['seq_len'], r['lidar_id'], r['rsvd'], r['rest'], 'OK' if ok else 'MISMATCH'))
        if r['pts']:
            offs = [p[0] for p in r['pts']]
            xs = [p[1] for p in r['pts']]
            print('       first pts: %s' % (r['pts'][:3],))
        print('       offset_time max=%d ns' % (max(p[0] for p in r['pts']),))
    print('\nresult: %d/%d messages parsed with zero remainder' % (len(rows) - bad, len(rows)))
    return 0 if bad == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
