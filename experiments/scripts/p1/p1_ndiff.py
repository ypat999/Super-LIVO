#!/usr/bin/env python3
"""两个 odom 记录的数值差异（按 header stamp 配对）：最大 Δp、最大 Δyaw。
用法: python3 /tmp/p1_ndiff.py <dirA> <dirB> [labelA] [labelB]
"""
import sqlite3
import struct
import sys
from pathlib import Path

import numpy as np


class CDR:
    """与 /tmp/eval_loop.py 一致的最小 CDR 读取器（对齐基准是 payload 起点 4）。"""

    def __init__(self, b):
        self.b, self.o = b, 4

    def _a(self, n):
        r = (self.o - 4) % n
        if r:
            self.o += n - r

    def i32(self):
        self._a(4)
        v = struct.unpack_from('<i', self.b, self.o)[0]
        self.o += 4
        return v

    def u32(self):
        self._a(4)
        v = struct.unpack_from('<I', self.b, self.o)[0]
        self.o += 4
        return v

    def f64(self, n=1):
        self._a(8)
        v = struct.unpack_from('<%dd' % n, self.b, self.o)
        self.o += 8 * n
        return np.array(v) if n > 1 else v[0]

    def string(self):
        n = self.u32()
        s = self.b[self.o:self.o + n]
        self.o += n
        return s.rstrip(b'\x00').decode('utf-8', 'replace')


def load(bag):
    db = sorted(Path(bag).glob('*.db3'))[0]
    con = sqlite3.connect(str(db))
    tid = con.execute("select id from topics where name='/lio/odom'").fetchone()[0]
    d = {}
    for (row,) in con.execute('select data from messages where topic_id=?', (tid,)):
        c = CDR(row)
        t = c.i32() + c.u32() * 1e-9
        c.string()
        c.string()
        p = c.f64(3)
        q = c.f64(4)
        d[round(t, 6)] = (p, q)
    con.close()
    return d


def yaw(q):
    x, y, z, w = q
    return np.degrees(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


A, B = load(sys.argv[1]), load(sys.argv[2])
la = sys.argv[3] if len(sys.argv) > 3 else Path(sys.argv[1]).name
lb = sys.argv[4] if len(sys.argv) > 4 else Path(sys.argv[2]).name
common = sorted(set(A) & set(B))
if not common:
    print('NDIFF %s vs %s 无公共 stamp' % (la, lb))
    sys.exit(0)
dp = np.array([np.linalg.norm(A[k][0] - B[k][0]) for k in common])
dy = np.array([abs((yaw(A[k][1]) - yaw(B[k][1]) + 180.0) % 360.0 - 180.0) for k in common])
print('NDIFF %s vs %s common=%d onlyA=%d onlyB=%d '
      'dp_max=%.6fm dp_mean=%.6fm dp_p99=%.6fm n_dp_gt_1e-6=%d n_dp_gt_1e-3=%d '
      'n_dp_gt_1e-2=%d n_identical=%d dyaw_max=%.5fdeg dyaw_mean=%.5fdeg'
      % (la, lb, len(common), len(set(A) - set(B)), len(set(B) - set(A)),
         dp.max(), dp.mean(), float(np.percentile(dp, 99)),
         int((dp > 1e-6).sum()), int((dp > 1e-3).sum()), int((dp > 1e-2).sum()),
         int((dp == 0.0).sum()), dy.max(), dy.mean()))
