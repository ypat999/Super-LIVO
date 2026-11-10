#!/usr/bin/env python3
"""统计方案A 逐帧配对时间稳定性（gpt排查.md 的 P0 第二项）。
从 visual_a 日志里抽 t_cloud / t_pair / t_img，输出三个量的分布与漂移。"""
import re
import sys
import numpy as np

pat = re.compile(r"t_cloud=([\d.]+) t_pair=([\d.]+) t_img=([\d.]+)")
rows = []
for line in open(sys.argv[1], errors="ignore"):
    m = pat.search(line)
    if m:
        rows.append([float(x) for x in m.groups()])
A = np.array(rows)
if len(A) < 5:
    print("样本不足", len(A)); sys.exit()
t_cloud, t_pair, t_img = A[:, 0], A[:, 1], A[:, 2]
dtpair = t_pair - t_cloud
dtimg = t_img - t_cloud
print(f"有效配对帧 {len(A)}   时间跨度 {t_cloud[-1]-t_cloud[0]:.1f} s")
for tag, v in (("dt_pair = t_pair - t_cloud", dtpair), ("dt_img = t_img - t_cloud", dtimg)):
    print(f"  {tag:<28} 中位 {np.median(v)*1000:8.2f} ms  均值 {v.mean()*1000:8.2f}  "
          f"P5 {np.percentile(v,5)*1000:8.2f}  P95 {np.percentile(v,95)*1000:8.2f}  "
          f"std {v.std()*1000:6.2f} ms")
# 漂移：把窗口分 10 段看中位数趋势
seg = np.array_split(np.arange(len(A)), 10)
print("  分段时间中位数(ms):", " ".join(f"{np.median(dtimg[s])*1000:.1f}" for s in seg))
print("  dt_img 绝对漂移(最大-最小中位) = "
      f"{(np.median(dtimg[seg[-1]]) - np.median(dtimg[seg[0]]))*1000:.2f} ms")
# 配对新鲜度：t_pair 是否总是同一张图
print(f"  唯一 t_pair 数 {len(np.unique(t_pair))} / 帧数 {len(A)}  "
      f"平均每图配 {len(A)/max(len(np.unique(t_pair)),1):.2f} 帧")
