#!/usr/bin/env bash
# 直接查 sqlite，看 FL2 录制包里 /cloud_registered_body 每条消息的实际字节数分布。
# 空 PointCloud2 序列化约 40~80 B；20102 点的 XYZI 云约 965 KB。
set -o pipefail
DB=/home/ywj/slam_data/official_vio_20261114/bag/bag_0.db3
python3 - "$DB" <<'EOF'
import sqlite3, sys, statistics
db = sqlite3.connect(sys.argv[1])
cur = db.cursor()
for name, n, mn, mx, avg in cur.execute("""
    SELECT t.name, COUNT(*), MIN(LENGTH(m.data)), MAX(LENGTH(m.data)), AVG(LENGTH(m.data))
    FROM messages m JOIN topics t ON m.topic_id = t.id
    GROUP BY t.name ORDER BY t.name"""):
    print("%-28s n=%-6d min=%-9d max=%-9d avg=%.0f" % (name, n, mn, mx, avg))

print("\n--- /cloud_registered_body 字节数分档 ---")
tn = cur.execute("SELECT id FROM topics WHERE name='/cloud_registered_body'").fetchone()[0]
sizes = [r[0] for r in cur.execute(
    "SELECT LENGTH(data) FROM messages WHERE topic_id=? ORDER BY id", (tn,))]
empty = sum(1 for s in sizes if s < 512)
print("total=%d  空(<512B)=%d  非空=%d" % (len(sizes), empty, len(sizes) - empty))
if sizes:
    print("前 12 条:", sizes[:12])
EOF
echo DONE_BAG_SIZES
