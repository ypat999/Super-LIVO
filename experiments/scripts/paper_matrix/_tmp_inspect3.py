import csv, sys

for p in sys.argv[1:]:
    lid, odo = [], []
    for row in csv.reader(open(p, newline="", errors="ignore")):
        if len(row) != 3 or not row[0] in ("lidar", "odom"):
            continue
        (lid if row[0] == "lidar" else odo).append((int(row[2]), int(row[1])))
    lid.sort(); odo.sort()
    print("###", p.split("paper_20261117/")[-1], "n_lidar", len(lid), "n_odom", len(odo))
    if not lid:
        continue
    t0 = lid[0][0]
    print("  lidar: first stamp-rel %.0fs recv-stamp %.1fs ; last stamp-rel %.0fs recv-stamp %.1fs"
          % ((lid[0][0] - t0) / 1e9, (lid[0][1] - lid[0][0]) / 1e9,
             (lid[-1][0] - t0) / 1e9, (lid[-1][1] - lid[-1][0]) / 1e9))
    if odo:
        print("  odom : first stamp-rel %.0fs recv-stamp %.1fs ; last stamp-rel %.0fs recv-stamp %.1fs"
              % ((odo[0][0] - t0) / 1e9, (odo[0][1] - odo[0][0]) / 1e9,
                 (odo[-1][0] - t0) / 1e9, (odo[-1][1] - odo[-1][0]) / 1e9))
        print("  odom recv span %.1fs ; lidar recv span %.1fs"
              % ((odo[-1][1] - odo[0][1]) / 1e9, (lid[-1][1] - lid[0][1]) / 1e9))
