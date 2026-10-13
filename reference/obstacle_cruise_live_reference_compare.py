"""Compare a live original-image probe with the recorded stop/slowdown debug."""

import argparse
import bisect
from collections import Counter
import json
from pathlib import Path

from offline_rosbag_cdr import decode_float32_multi_array_stamped, messages


PREFIX = "/planning/scenario_planning/lane_driving/motion_planning/"


def debug_rows(bag, suffix):
    return [(stamp, decode_float32_multi_array_stamped(payload)["data"])
            for stamp, payload in messages(
                bag, PREFIX + "obstacle_cruise_planner/debug/" + suffix)]


def match(rows, stamps, stamp):
    index = bisect.bisect_left(stamps, stamp)
    if index >= len(rows) or rows[index][0] - stamp > 50_000_000:
        return None
    return rows[index][1]


def report(name, pairs):
    counts = Counter()
    frame_groups = {"bag_only": [], "live_only": []}
    for frame, bag, live in pairs:
        key = "both" if bag and live else "bag_only" if bag else \
              "live_only" if live else "neither"
        counts[key] += 1
        if key in frame_groups:
            frame_groups[key].append(frame)
    print(name, dict(counts))
    for key, frames in frame_groups.items():
        if frames:
            print(f"  {key}: {min(frames)}–{max(frames)} ({len(frames)} frames)")
    return counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("live", type=Path)
    args = parser.parse_args()
    live = json.loads(args.live.read_text())
    stop = debug_rows(args.bag, "stop_planning_info")
    slow = debug_rows(args.bag, "slow_down_planning_info")
    stop_times = [row[0] for row in stop]
    slow_times = [row[0] for row in slow]
    stop_pairs = []
    slow_pairs = []
    for row in live:
        frame = row["frame"]
        stamp = row["record_stamp_ns"]
        bag_stop = match(stop, stop_times, stamp)
        bag_slow = match(slow, slow_times, stamp)
        if bag_stop is None or bag_slow is None or row["stop_debug"] is None:
            raise RuntimeError(f"debug callback not paired for frame {frame}")
        live_markers = row["markers"]["counts"] if row["markers"] else {}
        stop_pairs.append((frame, len(bag_stop) >= 4 and bag_stop[3] != 0.0,
                           len(row["stop_debug"]) >= 4 and
                           row["stop_debug"][3] != 0.0))
        slow_pairs.append((frame, len(bag_slow) >= 7,
                           bool(live_markers.get("obstacles_to_slow_down"))))
    print(f"paired frames: {len(live)}")
    report("stop", stop_pairs)
    report("slowdown", slow_pairs)


if __name__ == "__main__":
    main()
