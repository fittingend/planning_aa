"""차로 변경 후보와 원본 충돌 검사 표식을 수신 시각으로 짝지어 집계합니다."""

import argparse
import bisect
import collections
from pathlib import Path

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
    candidates = [(time, points) for time, points in candidates if points]
    markers = list(messages(args.bag, DEBUG))
    times = [time for time, _ in markers]
    results = collections.Counter()
    reasons = collections.Counter()
    for time, _ in candidates:
        index = bisect.bisect_left(times, time)
        choices = [i for i in (index - 1, index)
                   if 0 <= i < len(markers) and
                   abs(times[i] - time) <= 50_000_000]
        if not choices:
            results["표식 없음"] += 1
            continue
        nearest = min(choices, key=lambda i: abs(times[i] - time))
        decoded = decode_marker_array(markers[nearest][1])
        statuses = [marker["text"] for marker in decoded
                    if marker["namespace"] == "collision_check_object_info"]
        if not statuses:
            results["충돌 정보 없음"] += 1
        for status in statuses:
            if "Safe: No" in status:
                results["안전하지 않음"] += 1
            elif "Safe: Yes" in status:
                results["안전함"] += 1
            else:
                results["판독 불가"] += 1
            for line in status.splitlines():
                if line.startswith("Unsafe reason:"):
                    reasons[line.split(":", 1)[1].strip()] += 1
    print(f"비어 있지 않은 후보 {len(candidates)}, 상태 {dict(results)}, "
          f"이유 {dict(reasons)}")


if __name__ == "__main__":
    main()
