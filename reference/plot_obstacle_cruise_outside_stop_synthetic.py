"""Compare source node and ROS-free outside-stop candidate on matched synthetic input."""

import argparse
import bisect
import csv
import json
import math
import subprocess
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from offline_rosbag_cdr import decode_predicted_objects, messages


ROOT = Path(__file__).resolve().parent
BAG = ROOT.parents[1] / "251231_KCity_ori/ipc/rosbag_pedestrian/rosbag_20251231_132433_0.db3"
CASES = (
    ("launch disabled", "outside_stop_synthetic_source_disabled.json", 15, False, 1, False, 7, False, False),
    ("pedestrian", "outside_stop_synthetic_source_enabled.json", 15, False, 1, True, 7, False, False),
    ("late crossing", "outside_stop_synthetic_source_far.json", 35, False, 1, True, 7, False, False),
    ("top path misses", "outside_stop_synthetic_source_rank1.json", 15, True, 1, True, 7, False, False),
    ("two paths", "outside_stop_synthetic_source_rank2.json", 15, True, 2, True, 7, False, False),
    ("bicycle", "outside_stop_synthetic_source_bicycle.json", 15, False, 1, True, 6, False, False),
    ("car", "outside_stop_synthetic_source_car.json", 15, False, 1, True, 1, False, False),
    ("curved road", "outside_stop_synthetic_source_curve.json", 15, False, 1, True, 7, True, False),
    ("polygon", "outside_stop_synthetic_source_polygon.json", 15, False, 1, True, 7, False, True),
    ("curved polygon", "outside_stop_synthetic_source_curve_polygon.json", 15, False, 1, True, 7, True, True),
)


def runner_input(now_ns, object_stamp_ns, x, extra_path, classification,
                 curved, polygon):
    rows = ["1", f"{now_ns * 1e-9:.12f} {object_stamp_ns * 1e-9:.12f} "
            "0 0 0 0 0 0 1 5 101"]
    for i in range(101):
        if curved:
            yaw = math.atan(0.01 * i)
            rows.append(f"{i} {0.005 * i * i} 0 0 0 "
                        f"{math.sin(yaw / 2)} {math.cos(yaw / 2)} 5")
        else:
            rows.append(f"{i} 0 0 0 0 0 1 5")
    rows.append("1")
    length, width = (4.0, 2.0) if classification == 1 else (0.6, 0.6)
    center_y = 0.005 * x * x if curved else 0.0
    if polygon:
        length = 0.4
    rows.append(f"object {classification} {2 if polygon else 0} "
                f"{x} {center_y + 3} 0 {length} {width} 0 -2 "
                f"{3 if polygon else 0} "
                f"{2 if extra_path else 1}")
    if polygon:
        rows.extend(("0 -0.3", "0.4 0", "0 0.3"))
    if extra_path:
        rows.append("1 2 3")
        rows.extend(f"{x} {center_y + 3} 0" for _ in range(3))
    rows.append("1 1 3")
    rows.extend(f"{x} {center_y + y} 0" for y in (3, 1, -1))
    return "\n".join(rows) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("--bag", type=Path, default=BAG)
    args = parser.parse_args()
    object_records = list(messages(args.bag,
        "/perception/object_recognition/objects"))
    object_receipts = [receipt for receipt, _ in object_records]
    result = []
    for name, source_file, x, extra_path, path_count, enabled, classification, curved, polygon in CASES:
        source = json.loads((ROOT / source_file).read_text())[0]
        receipt_ns = source["record_stamp_ns"]
        object_index = bisect.bisect_right(object_receipts, receipt_ns) - 1
        object_stamp_ns = decode_predicted_objects(
            object_records[object_index][1])["stamp_ns"]
        command = [str(args.runner), "--show-stop-distance",
                   "--outside-stop-predicted-path-count", str(path_count)]
        if enabled:
            command.append("--enable-outside-stop-classes")
        output = subprocess.run(command, input=runner_input(
            receipt_ns + 5_000_000, object_stamp_ns, x, extra_path,
            classification, curved, polygon),
            text=True, capture_output=True, check=True).stdout.split()
        source_count = source["markers"]["counts"].get("obstacles_to_stop", 0)
        independent_count = int(output[3])
        result.append({
            "case": name,
            "source_stop_count": source_count,
            "planning_aa_stop_count": independent_count,
            "source_collision_distance_m": source["stop_debug"][3] if source_count else 0.0,
            "planning_aa_collision_distance_m": float(output[4]) if independent_count else 0.0,
        })
    csv_path = ROOT / "obstacle_cruise_outside_stop_synthetic.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=result[0].keys())
        writer.writeheader()
        writer.writerows(result)
    x = np.arange(len(result))
    fig, ax = plt.subplots(figsize=(14, 4))
    ax.bar(x - 0.18, [row["source_collision_distance_m"] for row in result],
           width=0.36, label="Autoware source node")
    ax.bar(x + 0.18, [row["planning_aa_collision_distance_m"] for row in result],
           width=0.36, label="planning-aa candidate")
    ax.set_xticks(x, [row["case"] for row in result], rotation=15)
    ax.set_ylabel("Selected collision distance from path start (m)")
    ax.set_ylim(0, 13)
    ax.grid(axis="y", alpha=0.25)
    ax.legend()
    fig.tight_layout()
    png_path = ROOT / "obstacle_cruise_outside_stop_synthetic.png"
    fig.savefig(png_path, dpi=170)
    print(csv_path)
    print(png_path)
    for row in result:
        print(row)


if __name__ == "__main__":
    main()
