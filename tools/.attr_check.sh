#!/bin/bash
python3 - <<'PY'
import json
s = json.load(open('/home/ywj/slam_data/singleframe/blackframe_replay/summary.json'))
print('keys:', list(s.keys()))
print('per_frame[0]:', s['per_frame'][0])
print('n frames:', len(s['per_frame']))
import matplotlib
print('mpl', matplotlib.__version__)
PY
zcat /home/ywj/slam_data/singleframe/blackframe_replay/voxel_observations.csv.gz | head -3
zcat /home/ywj/slam_data/singleframe/blackframe_replay/voxel_observations.csv.gz | wc -l
