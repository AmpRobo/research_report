#!/usr/bin/env python3
"""Summarize a Gazebo run from the controller's odometry/scan log."""
import argparse
import csv
import json
import math
from pathlib import Path


def clearance(x, y, cx, cy, sx, sy):
    dx = max(abs(x-cx)-sx/2, 0)
    dy = max(abs(y-cy)-sy/2, 0)
    return math.hypot(dx, dy)-0.24


def main():
    p = argparse.ArgumentParser()
    p.add_argument("csv")
    p.add_argument("--out")
    a = p.parse_args()
    with open(a.csv, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit("No odometry/control samples in log")
    pts = [(float(r["x"]), float(r["y"])) for r in rows]
    path_m = sum(math.dist(pts[i-1], pts[i]) for i in range(1, len(pts)))
    obstacles = [(-0.1, 0.30, 0.65, 0.80), (1.15, -0.30, 0.50, 0.55)]
    min_clearance = min(clearance(x, y, *box) for x, y in pts for box in obstacles)
    result = {
        "samples": len(rows), "sim_duration_s": round(float(rows[-1]["time"])-float(rows[0]["time"]), 1),
        "start_xy": pts[0], "finish_xy": pts[-1],
        "path_length_m": round(path_m, 2),
        "final_goal_distance_m": float(rows[-1]["distance_to_goal"]),
        "goal_reached": any(r["done"] == "1" for r in rows),
        "min_estimated_obstacle_clearance_m": round(min_clearance, 3),
        "collision_by_geometry": min_clearance < -0.02,
        "neural_turn_nonzero_fraction": round(sum(abs(float(r["neural_turn"])) > 0.05 for r in rows)/len(rows), 3),
    }
    if a.out:
        Path(a.out).write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
