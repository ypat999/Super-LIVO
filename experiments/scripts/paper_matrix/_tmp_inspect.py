import csv, sys, bisect

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
print("n_lidar", len(lid), "n_odom", len(odo))
print("lidar[0:3]", lid[:3])
print("odom[0:3]", odo[:3])
print("lidar_span_s", (lid[-1][0] - lid[0][0]) / 1e9 if lid else 0)
print("odom_span_s", (odo[-1][0] - odo[0][0]) / 1e9 if odo else 0)
print("lidar_recv_span_s", (lid[-1][1] - lid[0][1]) / 1e9 if lid else 0)

ls = [s for s, _ in lid]
lr = {s: r for s, r in lid}
print("\n# i  odom_stamp  off_ms  algo_ms   (first 12)")
for i, (st, rc) in enumerate(odo[:12]):
    j = bisect.bisect_left(ls, st)
    cand = [x for x in (ls[j] if j < len(ls) else None, ls[j - 1] if j else None) if x]
    near = min(cand, key=lambda x: abs(x - st)) if cand else None
    print(i, st, (st - near) / 1e6 if near else None, (rc - lr[near]) / 1e6 if near else None)
