#!/bin/bash
python3 - <<'PY'
import gzip
src = "/home/ywj/slam_data/singleframe/blackframe_replay/voxel_observations.csv.gz"
tmp = src + ".tmp"
with gzip.open(src, "rt") as fi, gzip.open(tmp, "wt") as fo:
    fo.write(fi.readline())
    for line in fi:
        fo.write(" ".join(line.split()).replace(" ", ",") + "\n")
import os
os.replace(tmp, src)
print("converted")
PY
zcat /home/ywj/slam_data/singleframe/blackframe_replay/voxel_observations.csv.gz | head -3
