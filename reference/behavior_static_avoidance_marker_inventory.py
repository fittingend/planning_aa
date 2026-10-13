"""정적 회피 후보가 활성일 때 원본 객체 분류 표식을 집계합니다."""

import argparse
import bisect
import collections
from pathlib import Path

from offline_rosbag_cdr import decode_marker_array, decode_path, messages


CANDIDATE = "/planning/path_candidate/static_obstacle_avoidance"
DEBUG = ("/planning/scenario_planning/lane_driving/behavior_planning/"
         "behavior_path_planner/debug/static_obstacle_avoidance")


def inspect(bag):
    debug = list(messages(bag, DEBUG))
    times = [time for time, _ in debug]
    counts = collections.Counter()
    frames = 0
    for time, payload in messages(bag, CANDIDATE):
        if not decode_path(payload)["points"]:
            continue
        index = bisect.bisect_left(times, time)
        choices = [item for item in (index - 1, index)
                   if 0 <= item < len(debug) and
                   abs(times[item] - time) <= 50_000_000]
        if not choices:
            counts["대응 표식 없음"] += 1
            continue
        nearest = min(choices, key=lambda item:
                      abs(times[item] - time))
        markers = decode_marker_array(debug[nearest][1])
        reasons = {marker["text"].strip() for marker in markers
                   if marker["namespace"].endswith("_info_reason")}
        frames += 1
        if not reasons:
            counts["분류 없음"] += 1
        for reason in reasons:
            counts[reason] += 1
        if not any(marker["namespace"].startswith("target_")
                   for marker in markers):
            counts["대상 객체 표식 없음"] += 1
    return frames, counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    args = parser.parse_args()
    bags = ([args.bag] if args.bag.is_file() else
            sorted(path for path in args.bag.glob("*/*/rosbag_*.db3")
                   if path.is_file()))
    total_frames = 0
    total_counts = collections.Counter()
    for bag in bags:
        frames, counts = inspect(bag)
        if frames:
            print(f"{bag}: 활성 후보·디버그 대응 {frames}개, "
                  f"분류 {dict(counts)}")
        total_frames += frames
        total_counts.update(counts)
    print(f"전체 {len(bags)}개 기록: 활성 후보·디버그 대응 "
          f"{total_frames}개, 분류 {dict(total_counts)}")


if __name__ == "__main__":
    main()
