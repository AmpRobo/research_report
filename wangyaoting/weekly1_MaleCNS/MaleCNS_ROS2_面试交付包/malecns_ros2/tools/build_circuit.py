#!/usr/bin/env python3
"""Extract a small, auditable MaleCNS v1.0 sensor-to-motor circuit.

Inputs are the official FlyEM Feather tables. The output stores original body IDs,
synapse counts and predicted transmitter labels; no edges are fabricated.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import pyarrow.compute as pc
import pyarrow.feather as feather


def top_by_side(candidates, sides, scores, n):
    out = []
    for side in ("L", "R"):
        ids = candidates[sides[candidates] == side]
        out.extend(ids[np.argsort(scores[ids])[::-1][:n]].tolist())
    return np.asarray(out, dtype=np.int64)


def accum(ids, weights, index):
    np.add.at(index, ids, weights)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--annotations", required=True)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--neurotransmitters", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    ann = feather.read_table(args.annotations, columns=["bodyId", "superclass", "somaSide", "type", "instance"])
    ids = ann["bodyId"].to_numpy().astype(np.int64)
    role_names = ann["superclass"].to_pylist()
    sides = np.array([s or "" for s in ann["somaSide"].to_pylist()])
    roles = np.zeros(len(ids), dtype=np.uint8)
    names = ["visual_projection", "cb_intrinsic", "descending_neuron", "vnc_intrinsic", "vnc_motor"]
    for i, name in enumerate(names, 1):
        roles[np.array([r == name for r in role_names])] = i
    order = np.argsort(ids)
    sorted_ids = ids[order]

    conn = feather.read_table(args.weights, memory_map=True)
    conn = conn.filter(pc.greater_equal(conn["weight"], 10))
    pre_ids = conn["body_pre"].to_numpy().astype(np.int64)
    post_ids = conn["body_post"].to_numpy().astype(np.int64)
    weights = conn["weight"].to_numpy().astype(np.int32)
    pre_idx = np.searchsorted(sorted_ids, pre_ids)
    post_idx = np.searchsorted(sorted_ids, post_ids)
    valid = (pre_idx < len(ids)) & (post_idx < len(ids))
    valid &= (sorted_ids[np.minimum(pre_idx, len(ids)-1)] == pre_ids)
    valid &= (sorted_ids[np.minimum(post_idx, len(ids)-1)] == post_ids)
    pre_idx, post_idx, weights = order[pre_idx[valid]], order[post_idx[valid]], weights[valid]
    source, target = roles[pre_idx], roles[post_idx]

    # Select paired visual projection cells with strong connections into the brain.
    visual_score = np.zeros(len(ids), dtype=np.float64)
    mask = (source == 1) & np.isin(target, [2, 3])
    accum(pre_idx[mask], weights[mask], visual_score)
    visual = top_by_side(np.flatnonzero(roles == 1), sides, visual_score, 36)

    # Descending cells must receive visual/brain input and transmit toward VNC.
    dn_in = np.zeros(len(ids), dtype=np.float64)
    dn_out = np.zeros(len(ids), dtype=np.float64)
    mask = (target == 3) & np.isin(source, [1, 2])
    accum(post_idx[mask], weights[mask], dn_in)
    mask = (source == 3) & np.isin(target, [4, 5])
    accum(pre_idx[mask], weights[mask], dn_out)
    dn_score = np.sqrt(dn_in * dn_out)
    descending = top_by_side(np.flatnonzero(roles == 3), sides, dn_score, 16)

    # Brain interneurons are selected by a true two-hop visual -> CB -> DN path.
    cb_in = np.zeros(len(ids), dtype=np.float64)
    cb_out = np.zeros(len(ids), dtype=np.float64)
    mask = np.isin(pre_idx, visual) & (target == 2)
    accum(post_idx[mask], weights[mask], cb_in)
    mask = (source == 2) & np.isin(post_idx, descending)
    accum(pre_idx[mask], weights[mask], cb_out)
    cb_score = np.sqrt(cb_in * cb_out)
    central = np.flatnonzero(roles == 2)
    central = central[np.argsort(cb_score[central])[::-1][:128]]

    # Include VNC interneurons and actual motor cells downstream of the DNs.
    motor_in = np.zeros(len(ids), dtype=np.float64)
    mask = (target == 5) & np.isin(source, [3, 4])
    accum(post_idx[mask], weights[mask], motor_in)
    motor = top_by_side(np.flatnonzero(roles == 5), sides, motor_in, 12)
    vnc_in = np.zeros(len(ids), dtype=np.float64)
    vnc_out = np.zeros(len(ids), dtype=np.float64)
    mask = np.isin(pre_idx, descending) & (target == 4)
    accum(post_idx[mask], weights[mask], vnc_in)
    mask = (source == 4) & np.isin(post_idx, motor)
    accum(pre_idx[mask], weights[mask], vnc_out)
    vnc_score = np.sqrt(vnc_in * vnc_out)
    vnc = np.flatnonzero(roles == 4)
    vnc = vnc[np.argsort(vnc_score[vnc])[::-1][:48]]

    chosen_idx = np.concatenate([visual, central, descending, vnc, motor])
    chosen_ids = ids[chosen_idx]
    edge_mask = np.isin(pre_idx, chosen_idx) & np.isin(post_idx, chosen_idx)
    src_id = ids[pre_idx[edge_mask]]
    dst_id = ids[post_idx[edge_mask]]
    edge_weights = weights[edge_mask]
    nt = feather.read_table(args.neurotransmitters, columns=["body", "consensus_nt"])
    nt = nt.filter(pc.is_in(nt["body"], value_set=__import__("pyarrow").array(chosen_ids)))
    nt_map = {int(x["body"]): x["consensus_nt"] for x in nt.to_pylist() if x["consensus_nt"]}
    role_list = roles[chosen_idx]
    side_list = sides[chosen_idx]
    transmitter = [nt_map.get(int(i), "unknown") for i in chosen_ids]
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, ids=chosen_ids, roles=role_list, sides=side_list,
                        source=src_id, target=dst_id, synapses=edge_weights,
                        neurotransmitters=np.asarray(transmitter),
                        instances=np.asarray([ann["instance"][int(i)].as_py() or "" for i in chosen_idx]))
    manifest = {
        "dataset": "MaleCNS v1.0", "license": "CC-BY 4.0", "min_synapses_for_selection": 10,
        "neurons": len(chosen_ids), "connections": len(src_id), "synapses": int(edge_weights.sum()),
        "roles": {name: int(np.sum(role_list == i)) for i, name in enumerate(names, 1)},
        "source": "https://male-cns.janelia.org/download/",
        "selection": "visual projection -> central brain interneuron -> descending neuron -> VNC interneuron -> motor neuron"
    }
    out.with_suffix(".json").write_text(json.dumps(manifest, indent=2))
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
