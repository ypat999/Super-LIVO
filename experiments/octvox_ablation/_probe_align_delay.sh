#!/usr/bin/env bash
# 计算两个 bag 回放时的对齐延迟。
# 玩家时间轴由 bag timestamp 驱动：wall(h) = W_start - bagT_first - k + h，k = bagT - header。
# 要求图像(header = h-0.1) 与 点云(header = h) 的 wall 相同 => D_img - D_fl2 = 0.1 + (bagT_first_raw - bagT_first_fl2) + (k_img - k_body)
set -o pipefail
python3 - <<'EOF'
import sqlite3, struct

def stamp(blob):
    sec, ns = struct.unpack_from("<iI", blob, 4)   # 4 字节前缀后才是 Header
    return sec + ns * 1e-9

def info(db, topic):
    con = sqlite3.connect(db); cur = con.cursor()
    tn = cur.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()[0]
    t, blob = cur.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY id LIMIT 1", (tn,)).fetchone()
    # bag 首条消息时间（决定播放器时间轴原点）
    t0 = cur.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
    return t0, t, stamp(blob)

FL2 = "/home/ywj/slam_data/official_vio_20261114/bag/bag_0.db3"
RAW = "/home/ywj/slam_data/Retail_Street/Retail_Street.db3"

t0_b, t_b, h_b = info(FL2, "/cloud_registered_body")
t0_i, t_i, h_i = info(RAW, "/left_camera/image")
k_b = t_b - h_b
k_i = t_i - h_i
print("FL2  bagT_first=%.6f  body: bagT=%.6f header=%.6f  k=%.3f" % (t0_b, t_b, h_b, k_b))
print("RAW  bagT_first=%.6f  image: bagT=%.6f header=%.6f  k=%.3f" % (t0_i, t_i, h_i, k_i))
d_img = 0.1 + (t0_i - t0_b) + (k_i - k_b)
print("\n需要的对齐：D_img - D_fl2 = %.3f s" % d_img)
print("即：FL2 包先起播，raw 包用 --delay %.3f" % max(0.0, d_img))
print("（上次用的是 --delay 2.61，图像因此晚了约 %.2f s）" % (2.61 - d_img))
EOF
echo DONE_ALIGN
