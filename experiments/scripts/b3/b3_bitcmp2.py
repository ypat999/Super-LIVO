#!/usr/bin/env python3
"""按 header stamp 配对的逐位比较。用法: b3_bitcmp2.py <bagdirA> <bagdirB>"""
import sqlite3
import struct
import sys
from pathlib import Path


def load(bag):
    db = sorted(Path(bag).glob('*.db3'))[0]
    con = sqlite3.connect(str(db))
    tid = con.execute("select id from topics where name='/lio/odom'").fetchone()[0]
    out = {}
    for (d,) in con.execute('select data from messages where topic_id=?', (tid,)):
        sec, nsec = struct.unpack_from('<iI', d, 4)
        out[(sec, nsec)] = d
    con.close()
    return out


a = load(sys.argv[1])
b = load(sys.argv[2])
print('A %s: %d 唯一 stamp' % (sys.argv[1], len(a)))
print('B %s: %d 唯一 stamp' % (sys.argv[2], len(b)))
common = sorted(set(a) & set(b))
print('公共 stamp: %d  A 独有 %d  B 独有 %d' % (len(common), len(set(a) - set(b)), len(set(b) - set(a))))
bad = [k for k in common if a[k] != b[k]]
if not bad:
    n = len(common)
    if n == len(a) == len(b):
        print('✅ 全部 %d 个 stamp 的 payload 逐字节相同' % n)
    else:
        print('⚠️ 公共 %d 个 stamp 全部逐字节相同，但两 bag 各自还有独有 stamp' % n)
else:
    print('❌ %d/%d 公共 stamp 的 payload 不同, 首个 stamp=%d.%09d' % (len(bad), len(common), *bad[0]))
    k = bad[0]
    da, db = a[k], b[k]
    diffs = [i for i in range(min(len(da), len(db))) if da[i] != db[i]]
    print('   长度 %d vs %d, 不同字节 %d 个, 偏移 %s' % (len(da), len(db), len(diffs), diffs[:12]))
    import numpy as np
    for tag, d in (('A', da), ('B', db)):
        x, y, z = struct.unpack_from('<3d', d, 24)
        qx, qy, qz, qw = struct.unpack_from('<4d', d, 48)
        print('   %s pos=[%.6f %.6f %.6f]' % (tag, x, y, z))
