"""Audit observable motion-velocity input receipts without inventing callback IDs.

The output trajectory copies the optimizer trajectory header in the source
node. Bag receipt times bound a callback interval but do not identify the
messages returned by PollingSubscriber::takeData inside that interval.
"""

import argparse
import bisect
import csv
import statistics
from pathlib import Path

from offline_rosbag_cdr import Reader, decode_marker_array, messages


PREFIX = "/planning/scenario_planning/lane_driving/motion_planning/"
OPTIMIZER = PREFIX + "path_optimizer/trajectory"
MOTION = PREFIX + "motion_velocity_planner/trajectory"
MARKERS = PREFIX + "motion_velocity_planner/out_of_lane/debug_markers"
WALLS = PREFIX + "motion_velocity_planner/out_of_lane/virtual_walls"
INPUTS = {
    "odometry": "/localization/kinematic_state",
    "acceleration": "/localization/acceleration",
    "objects": "/perception/object_recognition/objects",
    "signals": "/perception/traffic_light_recognition/traffic_signals",
}


def quantiles(values):
    if not values:
        return (None, None)
    ordered = sorted(values)
    return (statistics.median(ordered), ordered[int(.95 * (len(ordered) - 1))])


def audit(bag):
    receipts = {name: [time for time, _ in messages(bag, topic)]
                for name, topic in INPUTS.items()}
    outputs = {}
    for receipt, payload in messages(bag, MOTION):
        stamp, _ = Reader(payload).header()
        outputs.setdefault(stamp, []).append(receipt)
    inputs = []
    for receipt, payload in messages(bag, OPTIMIZER):
        stamp, _ = Reader(payload).header()
        candidates = outputs.get(stamp, [])
        matching_outputs = [time for time in candidates if time >= receipt]
        output_receipt = min(matching_outputs) if matching_outputs else None
        inputs.append((receipt, stamp, output_receipt))
    marker_count = nonempty_area_count = 0
    for _, payload in messages(bag, MARKERS):
        marker_count += 1
        if any(marker["namespace"] == "out_of_lane_areas" and marker["points"]
               for marker in decode_marker_array(payload)):
            nonempty_area_count += 1
    wall_count = sum(1 for _ in messages(bag, WALLS))
    rows = []
    for frame, (receipt, stamp, output_receipt) in enumerate(inputs):
        row = {"frame": frame, "optimizer_receipt_ns": receipt,
               "trajectory_stamp_ns": stamp,
               "motion_output_receipt_ns": output_receipt or ""}
        for name, times in receipts.items():
            before = bisect.bisect_right(times, receipt)
            latest = times[before - 1] if before else None
            end = (bisect.bisect_right(times, output_receipt)
                   if output_receipt is not None else before)
            row[name + "_latest_before_optimizer_ns"] = latest or ""
            row[name + "_new_receipts_before_output"] = end - before
            row[name + "_age_before_optimizer_ms"] = (
                (receipt - latest) / 1e6 if latest is not None else "")
        rows.append(row)
    print(f"optimizer={len(inputs)} matched_output="
          f"{sum(row['motion_output_receipt_ns'] != '' for row in rows)} "
          f"out_of_lane_markers={marker_count} "
          f"nonempty_areas={nonempty_area_count} virtual_walls={wall_count}")
    for name in INPUTS:
        ages = [row[name + "_age_before_optimizer_ms"] for row in rows
                if row[name + "_age_before_optimizer_ms"] != ""]
        ambiguous = sum(row[name + "_new_receipts_before_output"] > 0
                        for row in rows)
        median, p95 = quantiles(ages)
        print(f"{name}: latest-before-optimizer age median/p95="
              f"{median:.3f}/{p95:.3f}ms, "
              f"new receipts before output={ambiguous}/{len(rows)} frames")
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--csv", type=Path)
    args = parser.parse_args()
    rows = audit(args.bag)
    if args.csv and rows:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with args.csv.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)


if __name__ == "__main__":
    main()
