"""K-City 출발 계획의 원본 시작 자세 후보 표식을 집계합니다."""

import argparse
import collections
import math
import statistics
from pathlib import Path

from offline_rosbag_cdr import decode_marker_array, messages


TOPIC = ("/planning/scenario_planning/lane_driving/behavior_planning/"
         "behavior_path_planner/debug/start_planner")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    args = parser.parse_args()
    counts = collections.Counter()
    first_steps = []
    second_steps = []
    first_poses = []
    timestamps = []
    for stamp, payload in messages(args.bag, TOPIC):
        markers = decode_marker_array(payload)
        indexed = [item for item in markers
                   if item["namespace"] == "start_pose_candidates_idx"]
        if not indexed:
            continue
        counts[len(indexed)] += 1
        timestamps.append(stamp)
        first_poses.append(indexed[0]["pose"][:2])
        if len(indexed) >= 2:
            first_steps.append(math.dist(indexed[0]["pose"][:2],
                                         indexed[1]["pose"][:2]))
        if len(indexed) >= 3:
            second_steps.append(math.dist(indexed[1]["pose"][:2],
                                          indexed[2]["pose"][:2]))
    print(f"후보가 있는 프레임 {sum(counts.values())}개, "
          f"프레임별 후보 수 {dict(sorted(counts.items()))}")
    if first_steps:
        print(f"0→1번 후보 간격 중앙값 "
              f"{statistics.median(first_steps):.6f}m")
    if second_steps:
        print(f"1→2번 후보 간격 중앙값 "
              f"{statistics.median(second_steps):.6f}m")
    if first_poses:
        print(f"첫 후보 자세의 최대 위치 변화 "
              f"{max(math.dist(first_poses[0], pose) for pose in first_poses):.6f}m")
        print(f"후보 표식 구간 {(timestamps[-1] - timestamps[0]) * 1e-9:.2f}초")


if __name__ == "__main__":
    main()
