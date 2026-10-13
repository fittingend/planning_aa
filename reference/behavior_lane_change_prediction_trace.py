"""차로 변경 원본 예측 자세가 기록된 후보 경로 위에 있는지 검사합니다."""

import argparse
import bisect
import statistics
from pathlib import Path

from behavior_path_candidate_geometry import distance_to_path
from offline_rosbag_cdr import decode_marker_array, decode_path, messages


CANDIDATE = "/planning/path_candidate/lane_change_left"
DEBUG = ("/planning/scenario_planning/lane_driving/behavior_planning/"
         "behavior_path_planner/debug/lane_change_left")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    args = parser.parse_args()
    candidates = [(time, decode_path(payload)["points"])
                  for time, payload in messages(args.bag, CANDIDATE)]
    candidates = [(time, points) for time, points in candidates
                  if len(points) > 1]
    debug = list(messages(args.bag, DEBUG))
    times = [time for time, _ in debug]
    distances = []
    frame_maximums = []
    for time, candidate in candidates:
        index = bisect.bisect_left(times, time)
        choices = [item for item in (index - 1, index)
                   if 0 <= item < len(debug) and
                   abs(times[item] - time) <= 50_000_000]
        if not choices:
            continue
        matched = min(choices, key=lambda item:
                      abs(times[item] - time))
        markers = decode_marker_array(debug[matched][1])
        predicted = [item["pose"] for item in markers
                     if item["namespace"] == "ego_predicted_path" and
                     item["color"][0] < 0.1 and
                     item["color"][1] > 0.9]
        if not predicted:
            continue
        local = [distance_to_path(pose, candidate)
                 for pose in predicted]
        distances.extend(local)
        frame_maximums.append(max(local))
    print(f"대응 프레임 {len(frame_maximums)}, 예측 자세 {len(distances)}개, "
          f"점별 횡거리 중앙값 {statistics.median(distances):.6f}m, "
          f"프레임별 최대 중앙값 "
          f"{statistics.median(frame_maximums):.6f}m, "
          f"전체 최대 {max(frame_maximums):.6f}m")


if __name__ == "__main__":
    main()
