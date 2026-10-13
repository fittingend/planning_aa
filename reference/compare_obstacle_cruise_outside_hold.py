"""Compare a disappearing outside stop object across consecutive source callbacks."""

import argparse
import bisect
import csv
import json
from pathlib import Path
import subprocess

import matplotlib.pyplot as plt

from offline_rosbag_cdr import decode_predicted_objects, messages


ROOT = Path(__file__).resolve().parent
BAG = ROOT.parents[1] / "251231_KCity_ori/ipc/rosbag_pedestrian/rosbag_20251231_132433_0.db3"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("--bag", type=Path, default=BAG)
    args = parser.parse_args()
    source = json.loads((ROOT / "outside_stop_synthetic_source_hold.json").read_text())
    object_records = list(messages(args.bag,
        "/perception/object_recognition/objects"))
    receipts = [receipt for receipt, _ in object_records]
    rows = [str(len(source))]
    stamps = []
    for index, frame in enumerate(source):
        receipt_ns = frame["record_stamp_ns"]
        object_index = bisect.bisect_right(receipts, receipt_ns) - 1
        object_stamp_ns = decode_predicted_objects(
            object_records[object_index][1])["stamp_ns"]
        stamps.append(object_stamp_ns)
        rows.append(f"{(receipt_ns + 5_000_000) * 1e-9:.12f} "
                    f"{object_stamp_ns * 1e-9:.12f} 0 0 0 0 0 0 1 5 101")
        rows.extend(f"{x} 0 0 0 0 0 1 5" for x in range(101))
        rows.append("1")
        y, speed = (3.0, -2.0) if index == 0 else (20.0, 0.0)
        rows.append(f"object 7 0 15 {y} 0 0.6 0.6 0 {speed} 0 1")
        rows.append("1 1 3")
        rows.extend(f"15 {value} 0" for value in (
            (3, 1, -1) if index == 0 else (20, 20, 20)))
    output = subprocess.run(
        [str(args.runner), "--enable-outside-stop-classes",
         "--show-stop-distance"], input="\n".join(rows) + "\n",
        text=True, capture_output=True, check=True).stdout.splitlines()
    if len(output) != len(source):
        raise ValueError("원본·독립 프레임 수가 다릅니다")
    compared = []
    for index, (frame, line) in enumerate(zip(source, output)):
        _, _, _, count, distance = line.split()
        source_count = frame["markers"]["counts"].get("obstacles_to_stop", 0)
        compared.append({
            "frame": frame["frame"],
            "elapsed_object_stamp_s": (stamps[index] - stamps[0]) * 1e-9,
            "source_stop_count": source_count,
            "planning_aa_stop_count": int(count),
            "source_collision_distance_m": (
                frame["stop_debug"][3] if source_count else 0.0),
            "planning_aa_collision_distance_m": (
                float(distance) if int(count) else 0.0),
        })
    csv_path = ROOT / "obstacle_cruise_outside_stop_hold.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=compared[0].keys())
        writer.writeheader()
        writer.writerows(compared)
    fig, ax = plt.subplots(figsize=(8, 3.6))
    times = [row["elapsed_object_stamp_s"] for row in compared]
    for key, label, marker in (
            ("source_collision_distance_m", "Autoware source node", "o"),
            ("planning_aa_collision_distance_m", "planning-aa candidate", "x")):
        ax.plot(times, [row[key] for row in compared], marker=marker,
                label=label)
    ax.axvline(1.0, linestyle="--", color="gray", alpha=0.7,
               label="1 s hold threshold")
    ax.set_xlabel("Time since first object stamp (s)")
    ax.set_ylabel("Selected collision distance (m)")
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    png_path = ROOT / "obstacle_cruise_outside_stop_hold.png"
    fig.savefig(png_path, dpi=170)
    matches = sum(row["source_stop_count"] == row["planning_aa_stop_count"]
                  for row in compared)
    print(f"activation matches: {matches}/{len(compared)}")
    for row in compared:
        print(row)
    print(csv_path)
    print(png_path)


if __name__ == "__main__":
    main()
