#!/usr/bin/env python3
import sqlite3
DB = "/home/ywj/slam_data/Retail_Street/Retail_Street.db3"
con = sqlite3.connect(DB)
cur = con.cursor()
names = dict(cur.execute("select id, name from topics").fetchall())
for tid, name in sorted(names.items(), key=lambda kv: kv[1]):
    n, t0, t1 = cur.execute(
        "select count(*), min(timestamp), max(timestamp) from messages where topic_id=?",
        (tid,)).fetchone()
    span = (t1 - t0) / 1e9 if t0 else 0.0
    rate = n / span if span > 0 else 0.0
    print(f"{name:45s} n={n:6d} span={span:8.3f}s rate={rate:6.2f}Hz")
print("DONE_RAW_TOPICS")
