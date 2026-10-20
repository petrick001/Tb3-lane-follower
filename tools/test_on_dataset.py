#!/usr/bin/env python3
"""Run the detector on the course dataset (no ROS needed).
Usage: python3 tools/test_on_dataset.py /path/to/turtlebot3_autorace_dataset [out_dir]
Prints per-condition detection stats and saves a montage of sample results."""
import sys, glob, os, random, collections
import cv2, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from tb3_lane_follower.lane_detector import detect, DetectorParams

root = sys.argv[1]
out = sys.argv[2] if len(sys.argv) > 2 else "."
p = DetectorParams()
files = sorted(glob.glob(os.path.join(root, "**", "*.png"), recursive=True))
stats = collections.defaultdict(collections.Counter)
widths = []
dbg_imgs = {}
for f in files:
    cond = os.path.basename(os.path.dirname(f))
    p.yellow_on_left = "inverse" not in cond
    r = detect(cv2.imread(f), p)
    stats[cond][r.mode] += 1
    if r.mode == "both":
        widths.append((r.white_x - r.yellow_x) / p.proc_width)
    dbg_imgs.setdefault(cond, []).append(r.debug)
print("%-45s both yellow white none" % "condition")
for c, s in sorted(stats.items()):
    print("%-45s %4d %6d %5d %4d" % (c, s["both"], s["yellow"], s["white"], s["none"]))
tot = sum(sum(s.values()) for s in stats.values())
print("total images:", tot, " lost:", sum(s["none"] for s in stats.values()))
if widths:
    print("median lane width / image width when both lines seen: %.2f" % np.median(widths))
random.seed(1)
tiles = [random.choice(dbg_imgs[c]) for c in sorted(dbg_imgs)]
while len(tiles) % 4:
    tiles.append(np.zeros_like(tiles[0]))
rows = [np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)]
cv2.imwrite(os.path.join(out, "montage.jpg"), np.vstack(rows))
