#!/usr/bin/env python3
"""检查 Retail_Street bag 三个话题的 header 时间基准。"""
import sys

sys.path.insert(0, "/home/ywj/git/Super-LIVO/tools")
import singleframe_ab as sf  # noqa: E402

BAG = "/home/ywj/slam_data/Retail_Street"
r = sf._reader(BAG)
tt = {t.name: t.type for t in r.get_all_topics_and_types()}
first = {}
for _ in range(400):
    if not r.has_next():
        break
    tup = r.read_next()
    topic = tup[0]
    m = sf.deserialize_message(sf._payload(tup), sf._cls(tt[topic]))
    h = sf._hdr(m)
    if topic not in first:
        first[topic] = h
        print("%-24s first stamp %.6f" % (topic, h))
for k, v in first.items():
    print(k, "->", v)
