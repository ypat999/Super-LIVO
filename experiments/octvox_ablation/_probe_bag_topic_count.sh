#!/usr/bin/env bash
DB=/home/ywj/slam_data/official_vio_20261114/bag/bag_0.db3
echo "=== topics ==="
sqlite3 "$DB" "select id,name from topics;"
echo "=== per-topic count / time span ==="
sqlite3 -separator ' | ' "$DB" "select t.name, count(m.id), (max(m.timestamp)-min(m.timestamp))/1e9 from messages m join topics t on t.id=m.topic_id group by m.topic_id;"
echo DONE_BAG_COUNT
