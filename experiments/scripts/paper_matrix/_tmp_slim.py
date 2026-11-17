#!/usr/bin/env python3
"""临时：打印 latency_summary 的关键字段（用完删除）。"""
import json
import subprocess
import sys

paths = sys.argv[1:]
subprocess.run([sys.executable, "/home/ywj/git/Super-LIVO/tools/latency_summary.py",
                "--json", "/tmp/_tmp_slim.json"] + paths,
               stdout=subprocess.DEVNULL, check=True)
for d in json.load(open("/tmp/_tmp_slim.json")):
    print(d["label"], {
        "n_lidar": d["n_lidar"], "n_odom": d["n_odom"],
        "match_ratio": round(d["match_ratio"], 3),
        "lidar_period_med": round(d["lidar_period_ms_med"], 2),
        "lidar_gaps_gt150": d["lidar_gaps_gt150"],
        "clock_ok": d["clock_ok"],
        "transport_med": d["transport_ms_med"],
    })
