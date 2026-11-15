#!/usr/bin/env bash
# 验证 stamp 解析（raw 包被拆成两个 db3，先找出含图像的那个）。
set -o pipefail
python3 - <<'EOF'
import sqlite3, struct, glob

def find_topic(dbs, topic):
    for p in dbs:
        try:
            cur = sqlite3.connect(p).cursor()
            r = cur.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
        except sqlite3.OperationalError:
            continue
        if r:
            return p, r[0]
    return None, None

def probe(db_path, topic, tn, limit=None):
    rows = [b for (b,) in sqlite3.connect(db_path).execute(
        "SELECT data FROM messages WHERE topic_id=? ORDER BY id", (tn,))]
    if limit: rows = rows[:limit]
    print("  db=%s  %s  n=%d  len(first)=%d" % (db_path, topic, len(rows), len(rows[0])))
    print("    data[:12] =", rows[0][:12].hex())
    s0 = [struct.unpack_from("<iI", b, 0) for b in rows]
    s4 = [struct.unpack_from("<iI", b, 4) for b in rows]
    print("    off0 first=%d.%09d unique=%d" % (s0[0][0], s0[0][1], len(set(s0))))
    print("    off4 first=%d.%09d unique=%d" % (s4[0][0], s4[0][1], len(set(s4))))
    print("    off4 前 8 条:", s4[:8])
    return s4

raws = ["/home/ywj/slam_data/Retail_Street/Retail_Street.db3",
        "/home/ywj/slam_data/Retail_Street/bag_0.db3"]
db, tn = find_topic(raws, "/left_camera/image")
print("[RAW] /left_camera/image")
s_img = probe(db, "/left_camera/image", tn, 300) if db else []

print("[FL2] /cloud_registered_body")
db2, tn2 = find_topic(["/home/ywj/slam_data/official_vio_20261114/bag/bag_0.db3"],
                      "/cloud_registered_body")
s_body = probe(db2, "/cloud_registered_body", tn2, 60)

if s_img and s_body:
    print("\n小结：图像 header 步长(off4) 中位 %.4f s；body header 唯一值 %d / 总 %d" % (
        sorted([s_img[i+1][0] + s_img[i+1][1]*1e-9 - (s_img[i][0] + s_img[i][1]*1e-9)
                for i in range(len(s_img)-1)])[len(s_img)//2 - 1], len(set(s_body)), len(s_body)))
EOF
echo DONE_STAMP_VERIFY2
