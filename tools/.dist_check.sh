#!/bin/bash
python3 - <<'PY'
import sys
sys.path.insert(0, "/home/ywj/git/Super-LIVO/tools")
import numpy as np
from voxel_attribution import load_obs, voxel_table, adjacency, OBS

vid, fr, P, RGB, EL = load_obs(OBS)
tb = voxel_table(vid[EL], fr[EL], P[EL], RGB[EL])
n = tb["n"]; adj, empty, cshift = adjacency(tb)
print("体素总数", len(n))
for k in (1, 2, 3, 4, 5, 6, 8):
    print(f"n_obs>={k}: {(n>=k).sum():,}")
for nn in (3, 4):
    for s in (10, 15, 25):
        for j in (30, 40):
            print(f"n>={nn} lstd>={s} adj>{j}: {((n>=nn)&(tb['lstd']>=s)&(adj>j)).sum():,}")
print("n>=4 adj>40:", ((n>=4)&(adj>40)).sum())
print("n>=3 adj>40:", ((n>=3)&(adj>40)).sum())
print("n>=4 adj>40 lstd>=25:", ((n>=4)&(adj>40)&(tb['lstd']>=25)).sum())
print("n>=4 adj>40 且 (lstd>=10 或 nframes>=3):", ((n>=4)&(adj>40)&((tb['lstd']>=10)|(tb['nframes']>=3))).sum())
PY
