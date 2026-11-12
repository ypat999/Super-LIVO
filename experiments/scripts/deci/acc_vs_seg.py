#!/usr/bin/env python3
"""把 [Degeneracy]/[VisRot]/[VisB2] 三类日志按时间对齐，输出分段接受率。

用法: python3 /tmp/acc_vs_seg.py <log>
判据（实验记录 §7.3）：退化段接受率 ≫ 健康段 且 漂移斜率变小 → B 的退化安全收益成立。
这里用 Super-LIO 自己的退化信号（cond / active）当分段依据，不自造检测。
"""
import re
import sys

COND_T = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0

ts_re = re.compile(r'^[IW](\d{4}) (\d\d:\d\d:\d\d\.\d+)')
deg_re = re.compile(r'\[Degeneracy\] cond=([-\d.eE+]+) axes=(\d+) active=(\d+)')
acc_re = re.compile(r'\[VisRot\] inl=(\S+) delta=([-\d.]+)ms.*?chi2=([-\d.eE+]+) thresh=([-\d.eE+]+) acc=')
rej_re = re.compile(r'\[VisB2\] reject chi2=([-\d.eE+]+) > [-\d.eE+]+ t=([-\d.eE+]+).*?Pstd=\[([^\]]+)\]')
ok_re = re.compile(r'\[VisRot\] inl=.*?Pstd=\[\s*([-\d.eE+]+)')
none_re = re.compile(r'\[VisRot\] none\(([^)]*)\).*?imgs=(\d+) pairs=(\d+)')

def sec(hhmmss):
    h, m, s = hhmmss.split(':')
    return int(h) * 3600 + int(m) * 60 + float(s)

deg = []      # (t, cond, active)
acc = []      # (t, chi2, pz)
rej = []      # (t, chi2, pz)
none = []
for line in open(sys.argv[1], errors='ignore'):
    m = ts_re.match(line)
    if not m:
        continue
    t = sec(m.group(2))
    d = deg_re.search(line)
    if d:
        deg.append((t, float(d.group(1)), int(d.group(3))))
        continue
    a = acc_re.search(line)
    if a:
        p = ok_re.search(line)
        acc.append((t, float(a.group(3)), float(p.group(1)) if p else float('nan')))
        continue
    r = rej_re.search(line)
    if r:
        pz = [float(x) for x in r.group(3).split()]
        rej.append((t, float(r.group(1)), abs(pz[2])))
        continue
    n = none_re.search(line)
    if n:
        none.append((t, n.group(1), int(n.group(2))))

if not deg:
    print('无 [Degeneracy] 行，无法分段')
    sys.exit(1)

deg.sort()
conds = sorted(c for _, c, _ in deg)
n_active = sum(1 for _, _, a in deg if a)
print(f'Degeneracy: n={len(deg)} active={n_active} cond min={conds[0]:.3f} '
      f'med={conds[len(conds)//2]:.3f} max={conds[-1]:.3f}')

# 分段：按相邻 Degeneracy 报告的 cond 阈值切
def seg_of(t):
    cur = deg[0]
    for d in deg:
        if d[0] <= t:
            cur = d
        else:
            break
    return cur

bad = [d for d in deg if d[1] >= COND_T or d[2] == 1]
print(f'退化段判据: cond>={COND_T} 或 active=1 -> {len(bad)}/{len(deg)} 个采样点')

rows = {}
for t, chi2, pz in acc:
    key = 'deg' if seg_of(t)[1] >= COND_T or seg_of(t)[2] == 1 else 'ok'
    r = rows.setdefault(key, [0, 0, []])
    r[0] += 1
    r[2].append(chi2)
for t, chi2, pz in rej:
    key = 'deg' if seg_of(t)[1] >= COND_T or seg_of(t)[2] == 1 else 'ok'
    r = rows.setdefault(key, [0, 0, []])
    r[1] += 1

for key, name in (('ok', '健康段'), ('deg', '退化段')):
    if key not in rows:
        print(f'{name}: 无视觉帧')
        continue
    a, r, c = rows[key]
    tot = a + r
    med = sorted(c)[len(c) // 2] if c else float('nan')
    print(f'{name}: 视觉帧={tot} 接受={a} 拒绝={r} 接受率={100.0*a/max(tot,1):.1f}% chi2_med={med:.1f}')

print(f'none 行={len(none)} 首帧 imgs={none[0][2] if none else 0} '
      f'末帧 imgs={none[-1][2] if none else 0}')
