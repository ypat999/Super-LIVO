#!/usr/bin/env python3
"""一次性时基仪器化：把官方 FAST-LIVO2 发布的 odom header 从 node 墙钟改回 lidar 帧时刻。

只改 LIVMapper.cpp:1561 那一行（上游注释里本来就写了 fromSec(last_timestamp_lidar)，
是本仓库被注释掉后留下的临时实现），**不动任何算法逻辑**。

改这一行的理由（见 docs/HANDOVER_2026-11-08.md §18.2 / §18.4）：
  /aft_mapped_to_init.header.stamp = node->now() 含 0~110 ms 的发布延迟，
  超过一整个帧周期（99.85 ms），因此由墙钟反推 lidar 时基会产生 ±1 帧错配，
  在 0.5 m/s 下每错一帧引入 55 mm 假差，node.log 前缀乱序时甚至会错配更多帧
  —— 此前记录的「官方 LIO-only 帧间位移 P99 559 mm」「LIVO vs LIO-only 差 0.359 m」
  全部是这个伪影。改完后 odom 与 Log/result/*.txt 落在同一时基，无需再反推。

用法：python3 02_patch_odom_timebase.py [--revert]
"""
import shutil
import sys

SRC = "/home/ywj/fastlivo2_ws/src/fast_livo/src/LIVMapper.cpp"
OLD = ("  odomAftMapped.header.stamp = this->node->get_clock()->now(); "
       "//.ros::Time()fromSec(last_timestamp_lidar);")
NEW = ("  // [Super-LIVO 消融仪器化] 发布时刻改用 lidar 帧时刻（与 Log/result/*.txt 同基）。\n"
       "  // 原始实现见 .orig 备份：node->get_clock()->now()（墙钟，含 0~110ms 发布延迟）。\n"
       "  odomAftMapped.header.stamp = sec2Stamp(LidarMeasures.last_lio_update_time);")


def main():
    revert = "--revert" in sys.argv
    txt = open(SRC, encoding="utf-8").read()
    if revert:
        if NEW not in txt:
            print("已经是原始实现，无需还原")
            return
        shutil.copyfile(SRC, SRC + ".patched")
        open(SRC, "w", encoding="utf-8").write(txt.replace(NEW, OLD))
        print("已还原为墙钟实现（改动前副本存于 .patched）")
        return
    if NEW in txt:
        print("补丁已生效，无需重复")
        return
    if OLD not in txt:
        sys.exit(f"未找到目标行，请人工核对 {SRC} 第 1561 行附近")
    shutil.copyfile(SRC, SRC + ".orig")
    open(SRC, "w", encoding="utf-8").write(txt.replace(OLD, NEW))
    print(f"已打补丁（原文件备份 {SRC}.orig）")


if __name__ == "__main__":
    main()
