#!/usr/bin/env bash
# 把 RTK-SLAM construction_seq2 里的 /livox/lidar（livox_ros_driver/msg/CustomMsg）
# 离线转换为本仓可订阅的 livox_ros_driver2/msg/CustomMsg。
#
# 依据：已用 tools/rtk_verify_custommsg.py 按 livox_ros_driver2 的定义逐字段解析
# 原始报文，5/5 条 rest=0（字节全部消耗完），两包名的 CDR 字节流完全等价。
# 因此不需要反序列化再重序列化，只改写类型字符串即可。
set -e

SRC=/home/ywj/slam_data/rtk_slam/construction_seq2
OUT=/home/ywj/slam_data/rtk_slam/construction_seq2_r2
SRCDB=$SRC/construction_seq2.db3
OUTDB=$OUT/construction_seq2_r2.db3

mkdir -p "$OUT"

if [ ! -f "$OUTDB" ] || [ "$(stat -c %s "$OUTDB")" != "$(stat -c %s "$SRCDB")" ]; then
  echo ">>> 复制 db3 ($(stat -c %s "$SRCDB") B) ..."
  cp --sparse=always "$SRCDB" "$OUTDB"
fi
echo ">>> 复制完成 $(stat -c %s "$OUTDB") B"

python3 - "$SRC" "$OUT" <<'PY'
import sqlite3, sys, yaml, os

src, out = sys.argv[1], sys.argv[2]
db = os.path.join(out, 'construction_seq2_r2.db3')

NEWTYPE = 'livox_ros_driver2/msg/CustomMsg'
con = sqlite3.connect(db)
cur = con.execute("select id,name,type from topics")
rows = cur.fetchall()
print('转换前:')
for r in rows:
    print('   ', r)

con.execute("update topics set type=? where name='/livox/lidar'", (NEWTYPE,))
# 类型描述哈希随类型变更而失效，置空（Humble 播放不使用该字段）
cols = [r[1] for r in con.execute("PRAGMA table_info(topics)")]
if 'type_description_hash' in cols:
    con.execute("update topics set type_description_hash='' where name='/livox/lidar'")
con.commit()

print('转换后:')
for r in con.execute("select id,name,type from topics"):
    print('   ', r)
con.close()

meta = yaml.safe_load(open(os.path.join(src, 'metadata.yaml')))
meta['rosbag2_bagfile_information']['ros_distro'] = 'humble'
info = meta['rosbag2_bagfile_information']
info['files'][0]['path'] = 'construction_seq2_r2.db3'
info['relative_file_paths'] = ['construction_seq2_r2.db3']
for t in info['topics_with_message_count']:
    tm = t['topic_metadata']
    if tm['name'] == '/livox/lidar':
        tm['type'] = NEWTYPE
        tm['type_description_hash'] = ''
yaml.safe_dump(meta, open(os.path.join(out, 'metadata.yaml'), 'w'),
               sort_keys=False, allow_unicode=True)
print('metadata.yaml 已写出')
PY

echo ">>> 新包: $OUT"
ros2() { :; }
echo ">>> bag info:"
source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true
ros2 bag info "$OUT" 2>&1 | sed -n '1,40p'
