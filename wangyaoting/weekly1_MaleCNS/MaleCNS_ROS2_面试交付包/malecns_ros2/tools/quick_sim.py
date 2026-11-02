#!/usr/bin/env python3
"""Fast 2-D diagnostic of the same local controller before Gazebo validation."""
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from malecns_robot.connectome import Connectome


BOXES = [(-0.1, 0.30, 0.65, 0.80), (1.15, -0.30, 0.50, 0.55),
         (0, 3.0, 9, 0.2), (0, -3.0, 9, 0.2)]


def rays(x, y, yaw):
    angles = yaw + np.linspace(-1.57, 1.57, 181)
    dx, dy = np.cos(angles), np.sin(angles)
    distance = np.full(len(angles), 8.0)
    for cx, cy, sx, sy in BOXES:
        bounds = (cx-sx/2, cx+sx/2, cy-sy/2, cy+sy/2)
        for b, coordinate, other, lo, hi in [
            (bounds[0], dx, dy, bounds[2], bounds[3]),
            (bounds[1], dx, dy, bounds[2], bounds[3]),
            (bounds[2], dy, dx, bounds[0], bounds[1]),
            (bounds[3], dy, dx, bounds[0], bounds[1]),
        ]:
            base = x if coordinate is dx else y
            orth = y if coordinate is dx else x
            t = np.where(np.abs(coordinate) > 1e-8, (b-base)/coordinate, 1e9)
            at = orth + t*other
            distance = np.minimum(distance, np.where((t > 0) & (at >= lo) & (at <= hi), t, 8.0))
    return distance


def main():
    base = Path(__file__).resolve().parents[1] / 'malecns_robot' / 'data'
    net = Connectome(base/'circuit.npz', base/'readout.npz')
    x, y, yaw = -2.5, 0.0, 0.0
    path = []
    min_clearance = 99.0
    min_geometry = 99.0
    for step in range(3600):
        scan = rays(x, y, yaw)
        left = scan[107:159].min()
        front = scan[73:108].min()
        right = scan[22:74].min()
        angle = math.atan2(math.sin(math.atan2(-y, 2.5-x)-yaw),
                           math.cos(math.atan2(-y, 2.5-x)-yaw))
        obstacle = np.clip((1.7-np.array([left,front,right]))/1.5,0,1)
        goal = [np.clip(angle/1.57,0,1),np.clip(1-abs(angle)/1.57,0,1),np.clip(-angle/1.57,0,1)]
        speed, turn = net.predict(np.r_[obstacle,goal])
        speed = float(np.clip(speed,0,0.26));turn=float(np.clip(turn,-1.5,1.5))
        if front<0.5:
            speed=0.0;turn=1.0 if left>=right else -1.0
        x += speed*math.cos(yaw)*0.1
        y += speed*math.sin(yaw)*0.1
        yaw += turn*0.1
        if step%10==0:path.append((round(step*0.1,1),round(x,2),round(y,2),round(front,2),round(turn,2)))
        min_clearance = min(min_clearance, front)
        for cx,cy,sx,sy in BOXES[:2]:
            dx=max(abs(x-cx)-sx/2,0);dy=max(abs(y-cy)-sy/2,0)
            min_geometry=min(min_geometry,math.hypot(dx,dy)-0.24)
        if math.hypot(2.5-x,y)<0.35:break
    print(json.dumps({'reached':math.hypot(2.5-x,y)<0.35,'duration_s':round(step*0.1,1),
                      'end':[round(x,2),round(y,2)],'min_front_m':round(min_clearance,2),
                      'min_obstacle_clearance_m':round(min_geometry,2),
                      'path_sample':path[::10]},indent=2))


if __name__=='__main__':main()
