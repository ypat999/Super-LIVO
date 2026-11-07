#!/usr/bin/env python3
"""性能/耗时对比：解析方案 A/B 视觉节点逐帧耗时行，与官方 FAST-LIVO2 VIO 耗时。

A/B 日志格式：  "... 耗时 ms: 前端=.. 深度=.. ... 合计=.."
官方日志格式：  "| Current Total Time            | 0.038957 |"（秒；VIO 帧与纯 LIO 帧交替）

用法:
  perf_report.py [--budget-ms 100] [--a-log PATH] [--b-log PATH] [--official-log PATH]
缺省路径为本地跑 bag 时的临时日志位置。
"""
import argparse
import re
import numpy as np


def stats(name, arr, unit='ms'):
    a = np.asarray(arr, np.float64)
    if not len(a):
        print('  %-14s 无数据' % name)
        return None
    print('  %-14s n=%-4d 均值=%6.1f 中位=%6.1f P95=%6.1f 最大=%6.1f %s'
          % (name, len(a), a.mean(), np.median(a), np.percentile(a, 95), a.max(), unit))
    return a


def parse_stages(path):
    pat = re.compile(r'耗时 ms:\s*(.+)$')
    out = {}
    try:
        with open(path, 'r', errors='ignore') as f:
            for line in f:
                m = pat.search(line)
                if not m:
                    continue
                for k, v in re.findall(r'(\S+?)=([0-9.]+)', m.group(1)):
                    out.setdefault(k, []).append(float(v))
    except FileNotFoundError:
        print('  (缺日志: %s)' % path)
    return out


def parse_official(path):
    pat = re.compile(r'Current Total Time\s*\|\s*([0-9.]+)')
    vals = []
    try:
        with open(path, 'r', errors='ignore') as f:
            for line in f:
                m = pat.search(line)
                if m:
                    vals.append(float(m.group(1)) * 1000.0)
    except FileNotFoundError:
        print('  (缺日志: %s)' % path)
    return np.asarray(vals)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--a-log', default='/tmp/vis_a_full.log')
    ap.add_argument('--b-log', default='/tmp/vis_b_full.log')
    ap.add_argument('--official-log', default='/tmp/fastlivo_retail.log')
    ap.add_argument('--budget-ms', type=float, default=100.0, help='实时预算(图像10Hz=100ms)')
    args = ap.parse_args()

    print('=' * 78)
    print('方案 A 视觉节点逐帧耗时'); print('=' * 78)
    A = parse_stages(args.a_log)
    for k in ('前端', '深度', '重投影', '门控', '更新', '发布', '彩色', '合计'):
        if k in A:
            stats(k, A[k])

    print('\n' + '=' * 78)
    print('方案 B 视觉节点逐帧耗时'); print('=' * 78)
    B = parse_stages(args.b_log)
    for k in ('前端', '约束', '更新', '彩色', '合计'):
        if k in B:
            stats(k, B[k])

    print('\n' + '=' * 78)
    print('官方 FAST-LIVO2 逐帧耗时（retrieve+Jacobian+EKF+建点）'); print('=' * 78)
    off = parse_official(args.official_log)
    if len(off):
        vio = off[off > 15]
        lio = off[off <= 15]
        stats('全部帧', off)
        stats('VIO 帧', vio)
        stats('LIO 帧', lio)

    print('\n' + '=' * 78)
    print('汇总（每帧处理耗时，实时预算 %.0fms）' % args.budget_ms); print('=' * 78)
    a_t = np.mean(A['合计']) if '合计' in A else float('nan')
    b_t = np.mean(B['合计']) if '合计' in B else float('nan')
    o_t = np.mean(off[off > 15]) if len(off) else float('nan')
    a_c = np.mean(A['彩色']) if '彩色' in A else float('nan')
    b_c = np.mean(B['彩色']) if '彩色' in B else float('nan')
    print('  A 合计 %5.1f ms (着色 %.1f)  占预算 %.0f%%' % (a_t, a_c, 100 * a_t / args.budget_ms))
    print('  B 合计 %5.1f ms (着色 %.1f)  占预算 %.0f%%' % (b_t, b_c, 100 * b_t / args.budget_ms))
    print('  官方 VIO %5.1f ms            占预算 %.0f%%' % (o_t, 100 * o_t / args.budget_ms))
    print('  注：Super-LIO 的 LIO 耗时为 A/B 公共项，未计入；官方 LIO 帧约 %.1f ms。'
          % (np.mean(off[off <= 15]) if len(off) else float('nan')))


if __name__ == '__main__':
    main()
