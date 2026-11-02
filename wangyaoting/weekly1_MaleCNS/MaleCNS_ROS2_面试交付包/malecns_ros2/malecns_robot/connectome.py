"""A rate model whose directed edges and strengths come from MaleCNS v1.0.

The LiDAR/goal encoder, rate dynamics, and wheel readout are engineering choices;
the data do not imply that a fly naturally controls a wheeled robot.
"""
from pathlib import Path

import numpy as np


class Connectome:
    def __init__(self, circuit_path, readout_path=None):
        data = np.load(Path(circuit_path), allow_pickle=False)
        self.ids = data["ids"]
        self.roles = data["roles"]
        self.sides = data["sides"]
        self.output_indices = np.flatnonzero(np.isin(self.roles, [3, 4, 5]))
        lookup = {int(body): i for i, body in enumerate(self.ids)}
        src = np.array([lookup[int(v)] for v in data["source"]])
        dst = np.array([lookup[int(v)] for v in data["target"]])
        nt = data["neurotransmitters"]
        sign = np.ones(len(self.ids), dtype=np.float32)
        sign[np.isin(nt, ["gaba", "glutamate"])] = -1.0
        weight = sign[src] * np.log1p(data["synapses"].astype(np.float32))
        self.W = np.zeros((len(self.ids), len(self.ids)), dtype=np.float32)
        np.add.at(self.W, (dst, src), weight)
        norm = np.maximum(np.abs(self.W).sum(axis=1, keepdims=True), 1.0)
        self.W = 1.8 * self.W / norm

        left = np.flatnonzero((self.roles == 1) & (self.sides == "L"))
        right = np.flatnonzero((self.roles == 1) & (self.sides == "R"))
        if len(left) != 36 or len(right) != 36:
            raise ValueError("Circuit must have 36 left and 36 right visual projection neurons")
        self.input_groups = [left[:12], np.r_[left[12:18], right[12:18]],
                             right[:12], left[18:30],
                             np.r_[left[30:36], right[30:36]], right[18:30]]
        self.drive_matrix = np.zeros((len(self.ids), 6), dtype=np.float32)
        for channel, group in enumerate(self.input_groups):
            self.drive_matrix[group, channel] = 1.6
        self.state = np.zeros(len(self.ids), dtype=np.float32)
        self.theta = None
        if readout_path:
            self.theta = np.load(readout_path, allow_pickle=False)["theta"]

    def reset(self):
        self.state.fill(0.0)

    def step(self, input_values, iterations=18):
        u = np.clip(np.asarray(input_values, dtype=np.float32), 0, 1)
        drive = self.drive_matrix @ u
        for _ in range(iterations):
            activity = np.clip(self.W @ self.state + drive, 0, 1)
            self.state = 0.55 * self.state + 0.45 * activity
        return self.state[self.output_indices].copy()

    def predict(self, input_values):
        if self.theta is None:
            raise RuntimeError("Readout calibration is missing")
        features = np.r_[self.step(input_values), 1.0]
        return features @ self.theta
