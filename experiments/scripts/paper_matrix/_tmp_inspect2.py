import csv, sys, bisect, statistics as st

p = sys.argv[1]
lid, odo = [], []
for row in csv.reader(open(p, newline="", errors="ignore")):
    if len(row) != 3:
        continue
    if row[0] == "lidar":
        lid.append((int(row[2]), int(row[1])))
    elif row[0] == "odom":
        odo.append((int(row[2]), int(row[1])))

lid.sort(); odo.sort()
ls = [s for s, _ in lid]
lr = {s: r for s, r in lid}
algo, off = [], []
for st, rc in odo:
    j = bisect.bisect_left(ls, st)
    cand = [x for x in (ls[j] if j < len(ls) else None, ls[j - 1] if j else None) if x]
    near = min(cand, key=lambda x: abs(x - st))
    algo.append(((rc - lr[near]) / 1e6, (st - near) / 1e6))

a = [x[0] for x in algo]
o = [x[1] for x in algo]
def q(v, p):
    s = sorted(v); return s[min(len(s) - 1, int(round(p / 100 * (len(s) - 1))))]
print("algo  p1 %.0f p25 %.0f med %.0f p75 %.0f p95 %.0f p99 %.0f" % (q(a,1),q(a,25),q(a,50),q(a,75),q(a,95),q(a,99)))
print("off   p1 %.0f p25 %.0f med %.0f p75 %.0f p95 %.0f p99 %.0f" % (q(o,1),q(o,25),q(o,50),q(o,75),q(o,95),q(o,99)))
print("neg_algo", sum(1 for x in a if x < 0), "of", len(a))
# worst rows by algo
idx = sorted(range(len(a)), key=lambda i: a[i])[:6] + sorted(range(len(a)), key=lambda i: -a[i])[:6]
for i in idx:
    print("bad i=%d odom_stamp=%d algo=%.0f off=%.0f" % (i, odo[i][0], a[i], o[i]))
