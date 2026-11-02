#!/usr/bin/env python3
"""Fit only the robot motor readout; MaleCNS topology and weights stay fixed."""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy import sparse

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from malecns_robot.connectome import Connectome


def goal_encoding(angle):
    left = np.clip(angle / 1.57, 0, 1)
    right = np.clip(-angle / 1.57, 0, 1)
    front = np.clip(1 - np.abs(angle) / 1.57, 0, 1)
    return np.stack([left, front, right], axis=1)


def expert(u):
    ol, oc, or_, gl, gf, gr = u.T
    turn = 1.15 * (gl - gr) + 1.25 * (or_ - ol) + 0.65 * oc * (or_ - ol + 0.22)
    speed = 0.24 * (1 - 0.92 * oc) * (1 - 0.45 * np.abs(gl - gr))
    return np.stack([speed.clip(0, 0.25), turn.clip(-1.5, 1.5)], axis=1)


def features(circuit, u):
    W = sparse.csr_matrix(circuit.W)
    state = np.zeros((len(u), len(circuit.ids)), dtype=np.float32)
    drive = u @ circuit.drive_matrix.T
    for _ in range(18):
        activity = np.clip(W.dot(state.T).T + drive, 0, 1)
        state = 0.55 * state + 0.45 * activity
    return np.concatenate([state[:, circuit.output_indices], np.ones((len(u), 1))], axis=1)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--circuit", required=True)
    p.add_argument("--out", required=True)
    a = p.parse_args()
    rng = np.random.default_rng(2701)
    n = 12000
    obstacle = rng.beta(1.1, 1.8, (n, 3)).astype(np.float32)
    obstacle[:2500] = 0.0
    obstacle[2500:5000] = 0.0
    obstacle[np.arange(2500,5000),rng.integers(0,3,2500)] = rng.uniform(0.1,1.0,2500)
    angle = rng.uniform(-2.8, 2.8, n).astype(np.float32)
    angle[:3000] = rng.normal(0,0.4,3000)
    angle[:600] = 0.0
    shuffle = rng.permutation(n)
    obstacle, angle = obstacle[shuffle], angle[shuffle]
    u = np.concatenate([obstacle, goal_encoding(angle)], axis=1).astype(np.float32)
    y = expert(u)
    circuit = Connectome(a.circuit)
    x = features(circuit, u)
    split = 10000
    scale = np.std(x[:split], axis=0)
    scale[scale < 0.01] = 1.0
    # The bias term is unscaled.
    scale[-1] = 1.0
    xs = x / scale
    reg = np.eye(xs.shape[1], dtype=np.float32) * 1.0
    reg[-1, -1] = 0
    theta = np.linalg.solve(xs[:split].T @ xs[:split] + reg, xs[:split].T @ y[:split]) / scale[:, None]
    pred = x[split:] @ theta
    mae = np.mean(np.abs(pred - y[split:]), axis=0)
    original_w = circuit.W
    circuit.W = np.zeros_like(original_w)
    ablated = features(circuit, u[split:]) @ theta
    circuit.W = original_w
    ablated_mae = np.mean(np.abs(ablated - y[split:]), axis=0)
    info = {"training_examples": split, "test_examples": n-split,
            "holdout_speed_mae_mps": float(mae[0]),
            "holdout_turn_mae_rps": float(mae[1]),
            "no_connectivity_speed_mae_mps": float(ablated_mae[0]),
            "no_connectivity_turn_mae_rps": float(ablated_mae[1]),
            "readout_features": int(x.shape[1]-1),
            "method": "ridge imitation of a local obstacle/goal expert; MaleCNS edges frozen"}
    out = Path(a.out)
    np.savez_compressed(out, theta=theta.astype(np.float32))
    out.with_suffix(".json").write_text(json.dumps(info, indent=2))
    print(json.dumps(info, indent=2))


if __name__ == "__main__":
    main()
