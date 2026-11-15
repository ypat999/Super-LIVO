#!/usr/bin/env python3
"""检查刚录制的 bag：各话题戳/点数分布，确认逐帧配对可行。"""
import sys

import numpy as np

sys.path.insert(0, "/home/ywj/git/Super-LIVO/tools")
import singleframe_ab as sf  # noqa: E402

BAG = "/home/ywj/slam_data/official_vio_20261114/bag"

r = sf._reader(BAG)
topics = {t.name: t.type for t in r.get_all_topics_and_types()}
print("topics:", topics)
stats = {}
for name in topics:
    stats[name] = {"n": 0, "stamps": [], "pts": []}

while r.has_next():
    tup = r.read_next()
    topic = tup[0]
    msg = sf.deserialize_message(sf._payload(tup), sf._cls(topics[topic]))
    st = stats[topic]
    st["n"] += 1
    h = sf._hdr(msg)
    st["stamps"].append(h)
    if hasattr(msg, "width"):
        st["pts"].append(int(msg.width) * int(msg.height))

for name, st in stats.items():
    s = np.array(st["stamps"], float)
    print("\n== %s n=%d" % (name, st["n"]))
    print("  stamp range %.3f .. %.3f, span %.2f" % (s.min(), s.max(), s.max() - s.min()))
    d = np.diff(np.sort(s))
    if len(d):
        print("  median dt %.4f, zeros %d" % (np.median(d), int((d < 1e-9).sum())))
    if st["pts"]:
        p = np.array(st["pts"])
        print("  points: min %d max %d median %d, zeros %d" %
              (p.min(), p.max(), int(np.median(p)), int((p == 0).sum())))
