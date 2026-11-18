#!/usr/bin/env python3
"""按 header 时间重排 rosbag2(mcap) 的发布时刻（只改 log_time/publish_time）。

背景（2026-11-18，test3_move / test4_move / test20261118）：
  这三个 bag 录制时激光链路的投递延迟在漂——同一帧的 (录制时刻 − header) 相对中位数
  偏移达 −0.99~+0.50 s（test3_move）、−0.07~+1.65 s（test20261118），而 IMU 抖动
  < 0.05 s、图像 < 0.18 s。ros2 bag play 按"录制时刻"发布，下游于是看到 IMU 的
  header 跑到 lidar header 前面 1 s 多、lidar 帧成串迟到。Super-LIO 的 ESKF 前向
  积分基准被反复重置（实测累计 451 次、单次断档 1.0~1.4 s），A/B 两臂都从数据
  t≈10 s 开始超走（test3_move：A 55.5 s 走了 82.4 m，FL2 同期 13.2 m）。

做法：
  new_time = header + median(录制时刻 − header)   （每个 topic 各自取中位数）
  把每条消息排到它自己时间戳对应的真实时刻（跨 topic 对齐、topic 内单调），
  等价于"设备本来该有的到达顺序"。
  ⚠️ header 本身不动 ⇒ A/B 两进程链路的 IMG_OFFSET_SEC 沿用原值（就是上面那个中位数）。

用法:
  python3 20_restamp_mcap.py SRC_BAG_DIR [-o OUT_DIR] [--lookahead 5.0]
  默认输出 <SRC_BAG_DIR>_tfix/（重排后的 .mcap + patch 过的 metadata.yaml）
"""
import argparse
import heapq
import itertools
import re
import struct
import sys
import time
from pathlib import Path
from statistics import median

from mcap.reader import make_reader
from mcap.writer import Writer

NS = 1_000_000_000
HDR = struct.Struct("<iI")  # std_msgs/Header 的前两个字段：sec(int32) nanosec(uint32)


def header_ns(data: bytes) -> int:
    sec, nsec = HDR.unpack_from(data, 4)
    return sec * NS + nsec


def scan(src: Path):
    """第一遍：每个 topic 的 median(录制时刻 − header) 与抖动范围。"""
    diff = {}
    for _, ch, msg in make_reader(open(src, "rb")).iter_messages():
        diff.setdefault(ch.topic, []).append(msg.log_time - header_ns(msg.data))
    off = {t: int(median(v)) for t, v in diff.items()}
    print(f"[scan] {src.name}")
    for t in sorted(off):
        j = [(x - off[t]) / NS for x in diff[t]]
        print(f"  {t:22s} n={len(j):6d} median(rec-hdr)={off[t]/NS:.4f}s "
              f"抖动 {min(j):+.3f}~{max(j):+.3f}s")
    return off


def restamp(src: Path, off, out_mcap: Path, lookahead: float):
    emit_nt, emit_seq = [], 0
    heap, seq, max_seen, max_emitted, viol = [], 0, -1 << 62, -1 << 62, 0
    t0 = time.time()
    with open(out_mcap, "wb") as ofh, open(src, "rb") as ifh:
        w = Writer(ofh)
        w.start(profile="ros2", library="20_restamp_mcap.py")
        rdr = make_reader(ifh)
        s = rdr.get_summary()
        if s is None:
            sys.exit(f"[ERR] {src} 缺少 summary/index，无法重排")
        smap = {sid: w.register_schema(sc.name, sc.encoding, sc.data)
                for sid, sc in s.schemas.items()}
        cid_by_topic = {}
        for c in s.channels.values():
            cid_by_topic[c.topic] = w.register_channel(
                c.topic, c.message_encoding, smap[c.schema_id], c.metadata)

        def flush(item):
            nonlocal max_emitted, emit_seq
            nt, _, topic, data = item
            if nt < max_emitted:
                raise RuntimeError(f"输出非单调：{nt} < {max_emitted}")
            max_emitted = nt
            w.add_message(cid_by_topic[topic], nt, data, nt, emit_seq)
            emit_seq += 1

        for _, ch, msg in rdr.iter_messages():
            nt = header_ns(msg.data) + off[ch.topic]
            if nt < max_emitted:
                viol += 1
            heapq.heappush(heap, (nt, seq, ch.topic, msg.data))
            seq += 1
            if nt > max_seen:
                max_seen = nt
            limit = max_seen - int(lookahead * NS)
            while heap and heap[0][0] < limit:
                flush(heapq.heappop(heap))
            if seq % 20000 == 0:
                print(f"  ... {seq} 条, 缓冲 {len(heap)}, {time.time()-t0:.0f}s", flush=True)
        while heap:
            flush(heapq.heappop(heap))
        w.finish()
    print(f"[restamp] 写出 {emit_seq} 条 / 读入 {seq} 条, 越界(缓冲不足)={viol}, "
          f"{time.time()-t0:.0f}s")
    return max_emitted


def patch_meta(src_dir: Path, out_dir: Path, mcap_name: str, t_end_ns: int):
    src_meta = (src_dir / "metadata.yaml").read_text()
    new = re.sub(rf"{src_dir.name}_\d+\.mcap", mcap_name, src_meta)
    t_start = min(int(x) for x in
                  re.findall(r"nanoseconds_since_epoch: (\d+)", new))
    dur = t_end_ns - t_start
    # 逐处替换：starting_time(顶层 + per-file) 与 duration(顶层 + per-file)
    it_start = itertools.cycle([t_start])
    it_dur = itertools.cycle([dur])
    new = re.sub(r"(nanoseconds_since_epoch: )\d+",
                 lambda m: m.group(1) + str(next(it_start)), new)
    new = re.sub(r"(    nanoseconds: )\d+",
                 lambda m: m.group(1) + str(next(it_dur)), new)
    (out_dir / "metadata.yaml").write_text(new)
    print(f"[meta] starting_time={t_start} duration={dur/NS:.3f}s -> {out_dir/'metadata.yaml'}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--lookahead", type=float, default=5.0)
    a = ap.parse_args()
    src_dir = Path(a.bag).resolve()
    mcaps = sorted(src_dir.glob("*.mcap"))
    if len(mcaps) != 1:
        sys.exit(f"[ERR] 本工具只处理单文件 bag，{src_dir} 有 {len(mcaps)} 个 .mcap")
    out_dir = Path(a.out) if a.out else src_dir.parent / (src_dir.name + "_tfix")
    out_dir.mkdir(parents=True, exist_ok=True)
    off = scan(mcaps[0])
    out_mcap = out_dir / (src_dir.name + "_tfix_0.mcap")
    t_end = restamp(mcaps[0], off, out_mcap, a.lookahead)
    patch_meta(src_dir, out_dir, out_mcap.name, t_end)
    print(f"[done] {out_dir}")


if __name__ == "__main__":
    main()
