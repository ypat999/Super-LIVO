#!/usr/bin/env bash
# 从序列化数据前 8 字节读 header.stamp（sec:int32, nanosec:uint32），
# 检查 FL2 包里 /cloud_registered_body 的 header 时间戳是否大量重复。
set -o pipefail
DB=/home/ywj/slam_data/official_vio_20261114/bag/bag_0.db3
python3 - "$DB" <<'EOF'
import sqlite3, struct, sys, collections
db = sqlite3.connect(sys.argv[1])
cur = db.cursor()
for topic in ("/cloud_registered_body", "/cloud_registered", "/aft_mapped_lidar_to_init"):
    row = cur.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
    if row is None:
        print(topic, "not found"); continue
    tn = row[0]
    stamps = []
    for (blob,) in cur.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY id", (tn,)):
        sec, nsec = struct.unpack_from("<iI", blob, 0)
        stamps.append(sec + nsec * 1e-9)
    uniq = len(set(stamps))
    d = [round(stamps[i+1] - stamps[i], 4) for i in range(len(stamps)-1)]
    print("%-26s n=%d unique_stamp=%d  first=%s  step[min/med/max]=%s/%s/%s" % (
        topic, len(stamps), uniq, stamps[0], min(d), sorted(d)[len(d)//2], max(d)))
    dup = collections.Counter(stamps).most_common(3)
    print("   最常见重复值:", dup)
EOF
echo DONE_STAMP_DUP
