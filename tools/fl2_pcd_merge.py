#!/usr/bin/env python3
"""合并 FAST-LIVO2 分片落盘的彩色地图（pcd_save.interval > 0 的产物）。

背景：interval=-1（官方默认"结束时一次性落盘"）会把整轮 RGB 云堆在
pcl_wait_save 里，599 s 的 RTK construction_seq2 实测在累积阶段就被
OOM 杀掉（anon-rss 19.5 GB，0 个 pcd）。改成 interval>0 后每 N 帧 flush
一个 Log/pcd/<lio_time>.pcd 分片并清空缓冲，内存上界只有几十 MB/片，
代价就是产出是几十个分片，需要在这里离线合并。

分片都是**世界系**坐标且 flush 后缓冲被 swap 清空，所以合并 = 纯拼接，
不需要任何位姿变换。

用法：
  python3 tools/fl2_pcd_merge.py \
      --in /home/ywj/slam_data/rtk_slam/run_fl2_seq2/pcd \
      --out /home/ywj/slam_data/rtk_slam/run_fl2_seq2/Cloud_merged.pcd \
      --voxel 0.05

 voxel 下采样是**逐分片**做的（省内存：分片处理完即释放）。体素网格按
 各分片自身的最小点起算，边界与全局同尺寸网格略有错位，结果是全局下采样的
 近似超集（点多几成，密度一致），对"偏差 vs 距离"这类几何评测无影响。
 要完全等价于全局一次下采样，就 --voxel 0 先合并、再单独跑一次本脚本。
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in', dest='indir', required=True,
                    help='分片目录（*.pcd）或单个 pcd 文件')
    ap.add_argument('--out', required=True)
    ap.add_argument('--voxel', type=float, default=0.05,
                    help='体素边长(m)，0 = 不下采样')
    args = ap.parse_args()

    src = Path(args.indir)
    files = sorted(src.glob('*.pcd')) if src.is_dir() else [src]
    # 按 LIO 时刻（文件名）排序，保证逐片下采样的顺序稳定
    if not files:
        raise SystemExit('没有 pcd 分片：%s' % src)
    print('分片 %d 个' % len(files))

    kept = []
    n_raw = 0
    for i, f in enumerate(files):
        pc = o3d.io.read_point_cloud(str(f))
        pts = np.asarray(pc.points, dtype=np.float32)
        if pts.size == 0:
            print('  跳过空分片 %s' % f.name)
            continue
        n_raw += len(pts)
        if args.voxel > 0:
            pc = pc.voxel_down_sample(args.voxel)
        rgb = np.asarray(pc.colors, dtype=np.float32)
        if rgb.size == 0:
            raise SystemExit('%s 没有 RGB 通道，检查 config（type:0 + slam_mode LIVO）' % f.name)
        kept.append((np.asarray(pc.points, dtype=np.float32), rgb))
        print('  [%2d/%2d] %-22s %9d -> %8d  rgb[%.3f,%.3f]'
              % (i + 1, len(files), f.name, len(pts), len(kept[-1][0]),
                 rgb.min(), rgb.max()))

    xyz = np.concatenate([a for a, _ in kept])
    col = np.concatenate([b for _, b in kept])
    del kept

    if xyz.min() < -1e4 or np.isnan(xyz).any():
        print('!! 坐标异常（存在 nan/超大量级），不写出', file=sys.stderr)
        return 2
    if col.min() < 0 or col.max() > 1.001:
        print('!! RGB 不在 [0,1]，多半是 open3d 没解出 PCL 打包色，'
              '分片里的 rgb 字段要按 uint32 手工拆', file=sys.stderr)
        return 3

    out = o3d.geometry.PointCloud()
    out.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
    out.colors = o3d.utility.Vector3dVector(col.astype(np.float64))
    del xyz, col
    o3d.io.write_point_cloud(args.out, out, write_ascii=False)
    print('原始 %d 点 -> 写出 %d 点：%s'
          % (n_raw, len(out.points), args.out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
