#!/usr/bin/env python3
"""方案①安全性度量：从 ros2 bag 的 /lio/odom 提取逐帧位移/朝向跳变。

打印一行（供表格里 grep）：
  SAFE <label> frames=.. dt_med=.. dp_max=..@t dp_p99=.. dyaw_max=..@t
        n_over5cm=.. n_over10cm=.. n_over20cm=.. v_max=.. path=.. end=..
"""
import sqlite3
import struct
import sys
from pathlib import Path

import numpy as np


class CDR:
    """与 /tmp/eval_loop.py 完全一致的最小 CDR 读取器（对齐基准是 payload 起点 4）。"""

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


def read(bag):
    db = sorted(Path(bag).glob('*.db3'))[0]
    con = sqlite3.connect(str(db))
    tid = con.execute("select id from topics where name='/lio/odom'").fetchone()[0]
    ts, pos, quat = [], [], []
    for (d,) in con.execute('select data from messages where topic_id=? order by timestamp', (tid,)):
        c = CDR(d)
        t = c.i32() + c.u32() * 1e-9
        c.string()
        c.string()
        pos.append(c.f64(3))
        quat.append(c.f64(4))
        ts.append(t)
    con.close()
    return np.array(ts), np.array(pos), np.array(quat)


def yaw(q):
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.degrees(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


bag = sys.argv[1]
label = sys.argv[2] if len(sys.argv) > 2 else bag
t, p, q = read(bag)
if len(t) < 10:
    print('SAFE %s 数据太少(%d)' % (label, len(t)))
    sys.exit(0)

dt = np.diff(t)
dp = np.linalg.norm(np.diff(p, axis=0), axis=1)
dy = np.abs(wrap(np.diff(yaw(q))))
dmed = float(np.median(dt))
ok = dt < 1.5 * dmed          # 只看正常节拍的相邻帧，跳过丢帧造成的"合法大位移"
v = dp / np.maximum(dt, 1e-6)
dpn = dp[ok]
dtn = dt[ok]
dyp = dy[ok]
k = int(np.argmax(dpn))
ky = int(np.argmax(dyp))
print('SAFE %s frames=%d dt_med=%.3f 丢帧数=%d dp_max=%.3f@%.1fs dp_p99=%.3f dp_med=%.4f '
      'dyaw_max=%.1f@%.1fs n_over5cm=%d n_over10cm=%d n_over20cm=%d v_max=%.2f '
      'path=%.1f end=%.2f' % (
          label, len(t), dmed, int((~ok).sum()), dpn[k], t[k],
          float(np.percentile(dpn, 99)), float(np.median(dpn)), dyp[ky], t[ky],
          int((dpn > 0.05).sum()), int((dpn > 0.10).sum()), int((dpn > 0.20).sum()),
          float((dpn / dtn).max()), float(dp.sum()), float(np.linalg.norm(p[-1] - p[0]))))
# 正常节拍里最大的 5 个位移，判断是否集中在同一时刻（真跳变 vs 正常快速运动）
order = np.argsort(dpn)[::-1][:5]
print('  top5 dp: ' + '  '.join('%.3fm@%.1fs(dt=%.3f)' % (dpn[i], t[i], dtn[i])
                               for i in order))
