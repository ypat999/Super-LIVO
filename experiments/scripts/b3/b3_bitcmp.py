#!/usr/bin/env python3
"""逐位比较两个 /lio/odom bag：消息 CDR data 按 timestamp 排序后逐字节比对。
用法: python3 b3_bitcmp.py <bagdirA> <bagdirB>"""
import sqlite3
import sys
from pathlib import Path


def rows(bag):
    db = sorted(Path(bag).glob('*.db3'))[0]
    con = sqlite3.connect(str(db))
    tid = con.execute("select id from topics where name='/lio/odom'").fetchone()[0]
    out = con.execute(
        'select timestamp, data from messages where topic_id=? order by timestamp, id',
        (tid,)).fetchall()
    con.close()
    return out


a = rows(sys.argv[1])
b = rows(sys.argv[2])
print('A %s: %d msgs' % (sys.argv[1], len(a)))
print('B %s: %d msgs' % (sys.argv[2], len(b)))
n = min(len(a), len(b))
first_bad = None
ndiff = 0
for i in range(n):
    if a[i][1] != b[i][1]:
        ndiff += 1
        if first_bad is None:
            first_bad = i
print('按消息顺序逐字节: %s (%d/%d 条不同)' % ('全部相同 ✅' if ndiff == 0 else '存在差异 ❌', ndiff, n))
if first_bad is not None:
    import struct
    for tag, r in (('A', a[first_bad]), ('B', b[first_bad])):
        d = r[1]
        sec, nsec = struct.unpack_from('<iI', d, 4)
        print('  first_diff idx=%d %s stamp=%d.%09d' % (first_bad, tag, sec, nsec))
    da, db2 = a[first_bad][1], b[first_bad][1]
    if len(da) == len(db2):
        diffs = [k for k in range(len(da)) if da[k] != db2[k]]
        print('  payload 长度相同 %d, 不同字节 %d, 前 8 个偏移 %s' % (len(da), len(diffs), diffs[:8]))
    else:
        print('  payload 长度不同: %d vs %d' % (len(da), len(db2)))
# 同样比较 timestamp 列（记录时刻，本就应不同，仅参考）
