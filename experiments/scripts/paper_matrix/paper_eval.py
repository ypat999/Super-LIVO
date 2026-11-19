#!/usr/bin/env python3
"""四臂论文级对比：轨迹几何 / 绝对精度 / 实时性 / 地图规模。

运行前需 source ROS 环境（读 odom 的 rosbag2 需要 rosbag2_py）：

  bash -lc 'source /opt/ros/humble/setup.bash; \
            source third_party/Super-LIO/install/setup.bash; \
            python3 experiments/scripts/paper_matrix/paper_eval.py'

产物默认写到仓库 experiments/paper_compare/：
  metrics.csv / REPORT.md / figs/*.png

口径：
  - 两个结果根，按 ROOTS 顺序取第一个有 done 的：
    1) paper_20261119：B 臂按当前代码重跑（odom 切 10 Hz `/lio/odom` + `odom_robo⁻¹`
       换回 IMU 系、注入门控 0.4 s）。
    2) paper_sync：`lio.extrinsic.odom_robo` 回单位阵后的那轮重跑，FL2/SL/A 取这里
       （B 只是回退兜底，若用到会在报告 §0 里点名）。
  - move 用 move_ros2（图像 header 为主机时钟，A/B 以 -p img_time_offset 覆盖），
    原 move_livoxclk 副本已不在磁盘。
  - rtk 绝对精度走官方 RTK-SLAM 评测协议（tools/rtk_traj_prep.py + eval_ref）。
  - ntu01/ntu03 走 tools/odom_vs_gt.py（ATE 最近邻 + 区间旋转 RPE）。
  - 实时性走 tools/latency_summary.py（外部探针，generic subscription）。
"""
import argparse
import importlib.util
import json
import math
import os
import re
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path('/home/ywj/git/Super-LIVO')
RTK = Path('/home/ywj/slam_data/rtk_slam')
NTU = Path('/home/ywj/slam_data/ntu_viral')
ROOTS = [Path('/home/ywj/slam_data/paper_20261119'),
         Path('/home/ywj/slam_data/paper_sync')]
SEQS = ['move', 'retail', 'room', 'ntu03', 'ntu01', 'rtk']
ARMS = ['fl2', 'sl', 'a', 'b']
TOPIC = {'fl2': '/aft_mapped_to_init', 'sl': '/lio/odom',
         'a': '/super_livo_visual_a/odom', 'b': '/super_livo_visual_b/odom'}


def run_dir(seq, arm):
    for root in ROOTS:
        d = root / seq / arm
        if (d / 'done').exists():
            return d
    return None


def read_odom(bag, topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    rows = []
    while r.has_next():
        _, data, _ = r.read_next()
        m = deserialize_message(data, Odometry)
        p = m.pose.pose.position
        rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                     p.x, p.y, p.z))
    # 必须按 header 时间戳排序：bag 里记录顺序是**发布时刻**，负载抖动会让它
    # 与位姿时刻错序（实测上百处倒退、最大数秒），直接按记录顺序算路径长度会被
    # 假瞬移污染（room 会出现 1300 m 的荒谬路径）。同一时刻重复发布只保留最后一条。
    rows.sort(key=lambda r: r[0])
    dedup = []
    for r in rows:
        if dedup and abs(r[0] - dedup[-1][0]) < 1e-9:
            dedup[-1] = r
        else:
            dedup.append(r)
    return np.array(dedup) if dedup else np.zeros((0, 4))


def traj_stats(rows):
    if len(rows) < 2:
        return {}
    t, p = rows[:, 0], rows[:, 1:4]
    d = np.linalg.norm(np.diff(p, axis=0), axis=1)
    return {'frames': len(rows), 'dur_s': round(t[-1] - t[0], 1),
            'path_m': round(d.sum(), 3),
            'closure_m': round(float(np.linalg.norm(p[-1] - p[0])), 3),
            'z_min': round(p[:, 2].min(), 2), 'z_max': round(p[:, 2].max(), 2),
            'end_x': round(p[-1, 0], 2), 'end_y': round(p[-1, 1], 2),
            '_p': p}


def pcd_stats(d):
    pts, size = 0, 0
    for f in sorted(Path(d).rglob('*.pcd')):
        size += f.stat().st_size
        with open(f, 'rb') as fh:
            head = fh.read(600).decode('ascii', 'ignore')
        m = re.search(r'POINTS\s+(\d+)', head)
        if m:
            pts += int(m.group(1))
    return pts, size


def ntu_eval(rd, seq):
    bag = NTU / 'bags2' / ('eee_' + seq[-2:])
    gt = NTU / 'gt' / ('eee_' + seq[-2:]) / 'ground_truth.csv'
    cmd = [sys.executable, str(REPO / 'tools/odom_vs_gt.py'),
           '--gt', str(gt), '--bag', str(rd / 'odom'),
           '--topic', TOPIC[arm_of(rd)], '--label', 'x',
           '--imu-bag', str(bag), '--imu-topic', '/imu/imu', '--rot-dt', '0.1']
    out = subprocess.run(cmd, capture_output=True, text=True).stdout
    res = {}
    m = re.search(r'ATE\(RMSE\)=\s*([\d.]+)', out)
    if m:
        res['ate_rmse'] = float(m.group(1))
    m = re.search(r'中位=\s*([\d.]+).*?P95=\s*([\d.]+)', out)
    if m:
        res['ate_med'], res['ate_p95'] = float(m.group(1)), float(m.group(2))
    m = re.search(r'IMU 参照.*?\|e\| RMS=([\d.]+)°.*?中位=([\d.]+)°.*?P90=([\d.]+)°',
                  out)
    if m:
        res['rot_rms_deg'] = float(m.group(1))
        res['rot_med_deg'] = float(m.group(2))
        res['rot_p90_deg'] = float(m.group(3))
    m = re.search(r'\|e_z\|\(偏航\) RMS=([\d.]+)°', out)
    if m:
        res['yaw_rms_deg'] = float(m.group(1))
    return res


def arm_of(d):
    return Path(d).name


def rtk_eval(rd, arm):
    method = 'paper_' + arm
    out_raw = Path('/tmp/rtk_%s_imu.txt' % arm)
    cmd = [sys.executable, str(REPO / 'tools/rtk_traj_prep.py'),
           '--odom-bag', str(rd / 'odom'), '--odom-topic', TOPIC[arm],
           '--src-bag', str(RTK / 'construction_seq2_r2'),
           '--method', method, '--seq', 'construction_seq2',
           '--out-raw', str(out_raw)]
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0:
        return {'rtk_err': (p.stderr or p.stdout).strip().splitlines()[-1:]}
    spec = importlib.util.spec_from_file_location(
        'rtk_eval_mod', str(RTK / 'eval_ref/eval/eval.py'))
    ev = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(RTK / 'eval_ref/eval'))
    spec.loader.exec_module(ev)
    rows = ev.run_one(method, 'construction_seq2')
    res = {}
    for ver, met, err in rows:
        if ver == 'offline' and met:
            res['rtk_off_m'] = round(met['rmse_3d'], 3)
        if ver == 'offline_rel' and met:
            res['rtk_se3_m'] = round(met['rmse_3d_rel'], 3)
    if 'rtk_off_m' in res and 'rtk_se3_m' in res and res['rtk_off_m'] > 0:
        res['rtk_gap_pct'] = round(
            (res['rtk_off_m'] - res['rtk_se3_m']) / res['rtk_off_m'] * 100, 1)
    return res


def latency(seq, dirs):
    out = Path('/tmp/lat_%s.json' % seq)
    cmd = [sys.executable, str(REPO / 'tools/latency_summary.py')]
    cmd += [str(d) for d in dirs]
    cmd += ['--json', str(out)]
    subprocess.run(cmd, capture_output=True, text=True)
    res = {}
    if out.exists():
        for e in json.loads(out.read_text()):
            lab = Path(e['csv']).parent.name
            res[lab] = e
    return res


def fmt(v, nd=3):
    return '—' if v is None else ('%.*f' % (nd, v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=str(REPO / 'experiments/paper_compare'))
    args = ap.parse_args()
    outdir = Path(args.out)
    (outdir / 'figs').mkdir(parents=True, exist_ok=True)

    table, trajs = {}, {}
    for seq in SEQS:
        dirs = [run_dir(seq, a) for a in ARMS]
        dirs = [d for d in dirs if d]
        lat = latency(seq, dirs)
        for arm in ARMS:
            rd = run_dir(seq, arm)
            if not rd:
                continue
            rec = {'seq': seq, 'arm': arm, 'root': rd.parent.parent.name}
            rows = read_odom(rd / 'odom', TOPIC[arm])
            st = traj_stats(rows)
            if '_p' in st:
                trajs[(seq, arm)] = st.pop('_p')
            rec.update(st)
            pts, size = pcd_stats(rd / 'pcd')
            rec['pcd_pts'], rec['pcd_mb'] = pts, round(size / 1e6, 1)
            e = lat.get(arm, {})
            rec['odom_hz'] = round(1000.0 / e.get('odom_period_ms_med'), 2) \
                if e.get('odom_period_ms_med') else None
            rec['algo_ms_med'] = round(e.get('algo_ms_med', math.nan), 1) \
                if e else None
            rec['algo_ms_p95'] = round(e.get('algo_ms_p95', math.nan), 1) \
                if e else None
            rec['odom_gaps_gt150'] = e.get('odom_gaps_gt150')
            # 有效性：整段存在秒级以上积压时，algo 延时不再是「每帧处理耗时」，
            # 而是回放追赶产生的队列时间，不能进表。配对率偏低只记入脚注。
            if e and (e.get('algo_ms_med') or 0) > 2000:
                rec['algo_ms_med'] = rec['algo_ms_p95'] = None
                rec['lat_invalid'] = (e.get('match_ratio', 0), e.get('algo_ms_med'))
            else:
                rec['lat_invalid'] = None
            rec['lat_lowmatch'] = (e.get('match_ratio') < 0.9) if e else False
            if seq in ('ntu01', 'ntu03'):
                rec.update(ntu_eval(rd, seq))
            if seq == 'rtk':
                rec.update(rtk_eval(rd, arm))
            table[(seq, arm)] = rec
            print('[ok]', seq, arm, rec.get('path_m'), rec.get('ate_rmse'),
                  rec.get('rtk_off_m'), flush=True)

    # ---------------- CSV ----------------
    keys = ['seq', 'arm', 'root', 'frames', 'dur_s', 'path_m', 'closure_m',
            'z_min', 'z_max', 'ate_rmse', 'ate_med', 'ate_p95',
            'rot_rms_deg', 'rot_med_deg', 'rot_p90_deg', 'yaw_rms_deg',
            'rtk_off_m', 'rtk_se3_m', 'rtk_gap_pct',
            'odom_hz', 'algo_ms_med', 'algo_ms_p95', 'odom_gaps_gt150',
            'pcd_pts', 'pcd_mb']
    with open(outdir / 'metrics.csv', 'w') as f:
        f.write(','.join(keys) + '\n')
        for seq in SEQS:
            for arm in ARMS:
                r = table.get((seq, arm))
                if not r:
                    continue
                f.write(','.join(
                    '' if r.get(k) is None else str(r.get(k)) for k in keys) + '\n')

    make_figs(table, trajs, outdir)
    make_report(table, outdir)
    print('written ->', outdir)


def make_figs(table, trajs, outdir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors = {'fl2': '#d62728', 'sl': '#7f7f7f', 'a': '#1f77b4', 'b': '#2ca02c'}

    for seq in SEQS:
        ps = {a: trajs[(seq, a)] for a in ARMS if (seq, a) in trajs}
        if not ps:
            continue
        fig, ax = plt.subplots(figsize=(6, 5.2), dpi=150)
        for arm, p in ps.items():
            ax.plot(p[:, 0], p[:, 1], color=colors[arm], lw=1.1,
                    label='%s (%.2f m)' % (arm.upper(), np.linalg.norm(
                        np.diff(p, axis=0), axis=1).sum()))
            ax.plot(p[0, 0], p[0, 1], 'o', color=colors[arm], ms=4)
        ax.set_aspect('equal')
        ax.grid(alpha=0.3, lw=0.4)
        ax.set_xlabel('x [m]'), ax.set_ylabel('y [m]')
        ax.set_title('%s: trajectory (top view)' % seq)
        ax.legend(fontsize=7, loc='best')
        fig.tight_layout()
        fig.savefig(outdir / 'figs' / ('traj_%s.png' % seq))
        plt.close(fig)

    # ATE 柱状（有真值的序列）
    seqs = [s for s in ('ntu03', 'ntu01', 'rtk') if any(
        (s, a) in table for a in ARMS)]
    if seqs:
        fig, ax = plt.subplots(figsize=(7, 3.4), dpi=150)
        w = 0.2
        for i, arm in enumerate(ARMS):
            vals = []
            for s in seqs:
                r = table.get((s, arm), {})
                vals.append(r.get('ate_rmse') if s != 'rtk'
                            else r.get('rtk_off_m'))
            xs = np.arange(len(seqs)) + (i - 1.5) * w
            ax.bar(xs, [v if v is not None else 0 for v in vals], w,
                   color=colors[arm], label=arm.upper())
        ax.set_xticks(range(len(seqs)))
        ax.set_xticklabels([s + (' (abs.)' if s == 'rtk' else ' (ATE)')
                            for s in seqs])
        ax.set_ylabel('error [m]')
        ax.grid(axis='y', alpha=0.3, lw=0.4)
        ax.legend(fontsize=7)
        ax.set_title('Absolute / aligned trajectory error')
        fig.tight_layout()
        fig.savefig(outdir / 'figs' / 'ate.png')
        plt.close(fig)

    # 时延
    fig, ax = plt.subplots(figsize=(7.5, 3.2), dpi=150)
    xs = np.arange(len(SEQS))
    w = 0.2
    for i, arm in enumerate(ARMS):
        vals = [table.get((s, arm), {}).get('algo_ms_med') or 0 for s in SEQS]
        ax.bar(xs + (i - 1.5) * w, vals, w, color=colors[arm], label=arm.upper())
    ax.set_yscale('log')
    ax.set_xticks(xs), ax.set_xticklabels(SEQS)
    ax.set_ylabel('median algo latency [ms]')
    ax.grid(axis='y', alpha=0.3, lw=0.4)
    ax.legend(fontsize=7, ncol=4)
    ax.set_title('Per-frame odometry latency (log scale)')
    fig.tight_layout()
    fig.savefig(outdir / 'figs' / 'latency.png')
    plt.close(fig)


def make_report(table, outdir):
    def cell(v, nd=3):
        return '—' if v is None else ('%.*f' % (nd, v))

    def _g(s, a, k):
        return (table.get((s, a)) or {}).get(k)

    _stale_b = ['%s/B←%s' % (s, _g(s, 'b', 'root')) for s in SEQS
                if _g(s, 'b', 'root') not in (None, 'paper_20261119')]

    L = ['# 四臂论文级对比（move / retail / room / ntu03 / ntu01 / rtk）', '']
    L += ['> 生成脚本 `experiments/scripts/paper_matrix/paper_eval.py`；'
          '原始表 `metrics.csv`，图在 `figs/`。', '']
    L += ['## 0. 口径', '',
          '- 四臂：**FL2** = 官方 FAST-LIVO2；**SL** = 官方 Super-LIO'
          '（rtk/room 用自带视觉配置，其余数据集 LIO-only）；'
          '**A** = Super-LIO 主干 + 轻量视觉；**B** = Super-LIO 主干 + 紧耦合视觉。',
          '- 统一 1× 实时播放、同一 FastDDS / ROS_DOMAIN_ID、同一外部探针；'
          '延时为「odom 到达 − 对应雷达帧到达」的 algo 项。',
          '- 结果根：**B 取 `paper_20261119`**（本轮当前代码：`/lio/odom` 为 10 Hz、'
          '经 `odom_robo⁻¹` 换回 IMU 机体系、`poseFromSamples` 插值取相机位姿；'
          '本条跑批用 `visual_b_node`，注入门控 `app.visual_inject_max_dt` 只在 '
          '`visual_b_app` 单进程路径生效），'
          '**FL2/SL/A 取 `paper_sync`**（`lio.extrinsic.odom_robo` 回单位阵后的重跑）。',
          '- ⚠ **2026-11-19 起 b 臂已改为单体 `visual_b_app`，且 `app.visual_inject` 默认开'
          '（视觉约束真的写回 ESKF）** ⇒ 本表 B 行仍是双进程（无注入）口径，两者不可混比；'
          '矩阵 B 行需用单体形态重跑后才能与其余三臂并列'
          '（见 `exp_records.md` §39、`solution_b_tight_coupled/README.md` §9.11）。',
          '- move 序列：B 用 `move_ros2`（图像 header 保留主机时钟，以 '
          '`-p img_time_offset:=-204262818.9974` 覆盖）；原 `move_livoxclk`'
          '（图像 header 已改写为 Livox 时钟）副本已不在磁盘，FL2/SL/A 的 move 行'
          '仍来自 `paper_sync` 的该副本——两者只差图像 header 的时钟域，'
          '雷达/IMU 数据完全相同，对 LIO/轨迹无影响。',
          '- rtk 用 RTK-SLAM 官方绝对评测（起点+前 30 s 定向，不做全轨迹对齐），'
          '同时给出 SE3 对齐 ATE 与 Gap；ntu 用 ATE（最近邻）与旋转区间 RPE。', '']
    if _stale_b:
        L += ['> ⚠ B 行未全部来自 `paper_20261119`，以下为回退到旧根的条目：'
              + '、'.join(_stale_b) + '。', '']


    def _rng(vals, nd=2):
        v = [x for x in vals if isinstance(x, (int, float))]
        return ('%.*f–%.*f' % (nd, min(v), nd, max(v))) if v else '—'

    _mv = [table[('move', a)] for a in ARMS if ('move', a) in table]
    _cl = _rng([r.get('closure_m') for r in _mv], 3)
    _pa = _rng([r.get('path_m') for r in _mv], 2)
    _zmin = min([r.get('z_min') for r in _mv if r.get('z_min') is not None] or [0])
    _zmax = max([r.get('z_max') for r in _mv if r.get('z_max') is not None] or [0])
    _ntf = [table[(s, 'fl2')]['ate_rmse'] for s in ('ntu03', 'ntu01')
            if table.get((s, 'fl2'), {}).get('ate_rmse') is not None]
    _ntb = [table[(s, 'b')]['ate_rmse'] for s in ('ntu03', 'ntu01')
            if table.get((s, 'b'), {}).get('ate_rmse') is not None]
    _nts = [table[(s, 'sl')]['ate_rmse'] for s in ('ntu03', 'ntu01')
            if table.get((s, 'sl'), {}).get('ate_rmse') is not None]
    _nta = [table[(s, 'a')]['ate_rmse'] for s in ('ntu03', 'ntu01')
            if table.get((s, 'a'), {}).get('ate_rmse') is not None]
    _rot = [_g(s, a, 'rot_rms_deg') for s in ('ntu03', 'ntu01') for a in ARMS
            if _g(s, a, 'rot_rms_deg') is not None]
    _rt = [table[('rtk', a)].get('rtk_off_m') for a in ARMS
           if table.get(('rtk', a), {}).get('rtk_off_m') is not None]
    _rm = _rng([_g('room', a, 'path_m') for a in ARMS], 1)
    _rcl = _rng([_g('room', a, 'closure_m') for a in ARMS], 2)
    _lat = [_g(s, a, 'algo_ms_med') for s in SEQS for a in ARMS
            if _g(s, a, 'algo_ms_med') is not None]
    _lat_bad = ['%s/%s' % (s, a.upper()) for s in SEQS for a in ARMS
                if _g(s, a, 'lat_invalid')]

    L += ['## 0.5 关键结论', '',
          '1. **move（小场景闭环）**：四臂路径 %s m、闭环 %s m、z 在 %.2f ~ %.2f m，'
          '几何一致；A/B 地图与 FL2 同量级。' % (_pa, _cl, _zmin, _zmax),
          '2. **ntu（有真值室外）**：ATE FL2 %s m、SL %s m、B %s m、A %s m；'
          '旋转区间 RPE RMS %s°（逐行见 §2）。'
          % (_rng(_ntf), _rng(_nts), _rng(_ntb), _rng(_nta), _rng(_rot)),
          '3. **room（无真值，仅几何自洽）**：路径 %s m、闭环 %s m；无 FL2 行'
          '（该 bag 的 `/livox/lidar` 为 PointCloud2，FL2 臂不支持）。' % (_rm, _rcl),
          '4. **rtk（长时 + 室内约 461 s 无 GNSS + 无回环优化）**：绝对 RMSE %s m，'
          '主要来自室内累积漂移；官方基线（0.347–0.561 m）含离线优化，'
          '不可直接比绝对值。逐检查点分化见 §2。' % _rng(_rt, 1),
          '5. **实时性**：每帧延时中位 %s ms；%s。'
          % (_rng(_lat, 0),
             ('整段积压作废行：' + '、'.join(_lat_bad) + '（见 §3）') if _lat_bad
             else '无整段积压作废行'), '']

    L += ['## 1. 轨迹几何', '',
          '| 序列 | 臂 | 帧数 | 时长 [s] | 路径 [m] | 闭环 [m] | z 范围 [m] |',
          '|---|---|---:|---:|---:|---:|---|']
    for s in SEQS:
        for a in ARMS:
            r = table.get((s, a))
            if not r:
                continue
            L.append('| %s | %s | %s | %s | %s | %s | %s ~ %s |' % (
                s, a.upper(), r.get('frames'), cell(r.get('dur_s'), 1),
                cell(r.get('path_m')), cell(r.get('closure_m')),
                cell(r.get('z_min'), 2), cell(r.get('z_max'), 2)))
    _big = ['%s/%s（路径 %.1f m、闭环 %.2f m）'
            % (s, a.upper(), _g(s, a, 'path_m') or 0, _g(s, a, 'closure_m') or 0)
            for s in SEQS for a in ARMS if (_g(s, a, 'closure_m') or 0) > 5.0]
    if _big:
        L += ['', '> 闭环 >5 m 的行：' + '、'.join(_big)
              + '——路径长与 z 范围可能被瞬时跳变放大，不代表实际运动。']
    L += ['> `ntu` 的 FL2 高度与其他三臂符号相反（`/aft_mapped_to_init` 的 z 轴取向不同），'
          '比较高度时按 |z| 看待。']

    L += ['', '## 2. 绝对精度（有真值的数据集）', '',
          '| 序列 | 臂 | ATE RMSE [m] | ATE 中位 [m] | ATE P95 [m] | '
          '旋转 RPE RMS [°] | 偏航 RMS [°] |',
          '|---|---|---:|---:|---:|---:|---:|']
    for s in ('ntu03', 'ntu01'):
        for a in ARMS:
            r = table.get((s, a))
            if not r or 'ate_rmse' not in r:
                continue
            L.append('| %s | %s | %s | %s | %s | %s | %s |' % (
                s, a.upper(), cell(r.get('ate_rmse')), cell(r.get('ate_med')),
                cell(r.get('ate_p95')), cell(r.get('rot_rms_deg'), 2),
                cell(r.get('yaw_rms_deg'), 2)))
    L += ['', '### RTK-SLAM construction_seq2（官方协议）', '',
          '| 臂 | 绝对 RMSE [m] | SE3 ATE [m] | Gap [%] |', '|---|---:|---:|---:|']
    for a in ARMS:
        r = table.get(('rtk', a))
        if not r or 'rtk_off_m' not in r:
            continue
        L.append('| %s | %s | %s | %s |' % (
            a.upper(), cell(r.get('rtk_off_m')), cell(r.get('rtk_se3_m')),
            cell(r.get('rtk_gap_pct'), 1)))
    L += ['', '> **rtk 数值口径注意事项（重要）**',
          '> 1. 该序列 `/gnss/fix` 只在 t≈0–112 s 与 t≈573–599 s 有两个有效段，'
          '中间约 461 s 在室内、无任何有效定位；检查点覆盖整段，因此中后段误差是'
          '**纯里程计累积漂移**，不是定轨误差。',
          '> 2. 本仓库链路只做在线 LIO（结束时导出 `odom_poses.txt` 关键帧供外部 '
          'SC-PGO），**未做回环/全局优化**；`eval_ref` 里官方基线'
          '（super_lio 0.347 m、visoff 0.348 m、fast_livo2 0.561 m）是离线/优化后轨迹，'
          '与本表不可直接比绝对值。四臂之间同口径，可横向比较。',
          '> 3. 逐检查点看：室外段（t≈43/85 s）各臂都在 1–2 m 内；进入室内'
          '（t≥162 s）后各臂分化，该部分误差是「室内抗漂移/异常恢复」能力的排序，'
          '不反映室外定轨精度（逐检查点误差见 `node.log` 的 RTK 探针与 '
          '`/tmp/rtk_<arm>_imu.txt`）。', '']

    L += ['', '## 3. 实时性', '',
          '| 序列 | 臂 | odom 频率 [Hz] | 每帧延时中位 [ms] | 延时 P95 [ms] | '
          '>150 ms 断流 |',
          '|---|---|---:|---:|---:|---:|']
    bad = []
    for s in SEQS:
        for a in ARMS:
            r = table.get((s, a))
            if not r:
                continue
            L.append('| %s | %s | %s | %s | %s | %s |' % (
                s, a.upper(), cell(r.get('odom_hz'), 2),
                cell(r.get('algo_ms_med'), 1), cell(r.get('algo_ms_p95'), 1),
                cell(r.get('odom_gaps_gt150'), 0)))
            if r.get('lat_invalid'):
                ratio, ms = r['lat_invalid']
                bad.append('%s/%s（配对率 %.2f，整段中位 %.1f s）'
                           % (s, a.upper(), ratio, (ms or 0) / 1e3))
    if bad:
        L += ['', '> **实时性作废行**：' + '；'.join(bad) + '。',
              '> 这些运行存在整段级回放积压（处理速度跟不上 1× 播放，队列持续增长，'
              '探针抓到的雷达帧数也明显少于 bag 内帧数），表里的「每帧延时」'
              '已经变成排队等待时间，不能当作算法耗时；保留 odom 频率与断流计数作参考。'
              '需降速（`RATE=0.5`）重跑后才能给出实时性结论。']
    low = ['%s/%s' % (s, a.upper()) for s in SEQS for a in ARMS
           if table.get((s, a), {}).get('lat_lowmatch')]
    if low:
        L += ['> 另有 %s 的帧配对率 < 0.9（探针帧集与 odom 时间戳不完全对齐，'
              '常见于 20 Hz 视觉帧 / 帧尾时间戳），其延时中位数只作量级参考。'
              % '、'.join(low)]

    L += ['', '## 4. 地图规模', '',
          '| 序列 | 臂 | 点数 | 体积 [MB] |', '|---|---|---:|---:|']
    for s in SEQS:
        for a in ARMS:
            r = table.get((s, a))
            if not r:
                continue
            L.append('| %s | %s | %s | %s |' % (
                s, a.upper(), r.get('pcd_pts'), cell(r.get('pcd_mb'), 1)))
    L += ['', '> SL 臂导出的是主干自带地图（较重体素降采样），与 A/B/FL2 的'
          '全量彩色点云不是同一密度口径，规模不可直接横比；A/B 与 FL2 之间可比。']

    L += ['', '## 5. 图', '',
          '- `figs/traj_<seq>.png`：各序列四臂俯视轨迹（图例含路径长）；',
          '- `figs/ate.png`：ntu03 / ntu01 的 ATE 与 rtk 绝对 RMSE；',
          '- `figs/latency.png`：各臂每帧延时中位（对数轴）。', '']
    (outdir / 'REPORT.md').write_text('\n'.join(L))


if __name__ == '__main__':
    main()
