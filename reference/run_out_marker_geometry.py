"""돌발 진입 디버그 표식의 차량·객체 외곽 교차를 기록 경로별로 확인합니다."""

import argparse
import bisect
from pathlib import Path

from offline_rosbag_cdr import (
    decode_float32_multi_array_stamped, decode_marker_array, messages,
)


PREFIX = ("/planning/scenario_planning/lane_driving/behavior_planning/"
          "behavior_velocity_planner/debug/run_out")
PATH = ("/planning/scenario_planning/lane_driving/behavior_planning/"
        "path_with_lane_id")


def orientation(a, b, c):
    return ((b[0] - a[0]) * (c[1] - a[1]) -
            (b[1] - a[1]) * (c[0] - a[0]))


def boundaries_cross(first, second):
    for a, b in zip(first, first[1:]):
        for c, d in zip(second, second[1:]):
            ab_c = orientation(a, b, c)
            ab_d = orientation(a, b, d)
            cd_a = orientation(c, d, a)
            cd_b = orientation(c, d, b)
            if ab_c * ab_d < 0 and cd_a * cd_b < 0:
                return True
    return False


def marker_intersections(payload):
    markers = decode_marker_array(payload)
    vehicles = []
    predicted = 0
    collisions = []
    for marker in markers:
        if marker["action"] != 0 or marker["type"] != 4:
            continue
        points = marker["points"]
        if marker["namespace"] == "predicted_vehicle_polygons":
            vehicles.extend(points[i:i + 5]
                            for i in range(0, len(points) - 4, 5))
        elif marker["namespace"] == "predicted_obstacle_polygons":
            predicted += 1
        elif marker["namespace"] == "collision_obstacle_polygons" and len(points) >= 4:
            collisions.append(points)
    # 원본은 검사한 모든 차체 외곽을 누적하지만, 충돌 뒤의 객체 외곽은
    # 마지막 차체 외곽에서 발견한 결과입니다.
    hits = sum(boundaries_cross(vehicles[-1], obstacle)
               for obstacle in collisions) if vehicles else 0
    return len(vehicles), predicted, len(collisions), hits


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--first", type=int, default=248)
    parser.add_argument("--last", type=int, default=252)
    args = parser.parse_args()
    paths = list(messages(args.bag, PATH))
    debug = list(messages(args.bag, PREFIX + "/debug_values"))
    marker = list(messages(args.bag, PREFIX))
    debug_times = [row[0] for row in debug]
    marker_times = [row[0] for row in marker]
    print("원시 경로 인덱스 | 디버그 지연 ms | 원본 충돌 거리 m | 표식 지연 ms | "
          "차량 외곽 | 비충돌 객체 표식 | 충돌 객체 표식 | 마지막 차체와 경계 교차")
    for index in range(args.first, min(args.last + 1, len(paths))):
        time = paths[index][0]
        di = bisect.bisect_left(debug_times, time)
        mi = bisect.bisect_left(marker_times, time)
        if di >= len(debug) or mi >= len(marker):
            continue
        debug_delay = (debug_times[di] - time) / 1e6
        marker_delay = (marker_times[mi] - time) / 1e6
        if debug_delay > 50 or marker_delay > 50:
            continue
        values = decode_float32_multi_array_stamped(debug[di][1])["data"]
        collision = values[7] if len(values) > 7 else 50.0
        vehicle_count, predicted_count, collision_count, hits = marker_intersections(
            marker[mi][1])
        print(f"{index} | {debug_delay:.2f} | {collision:.3f} | "
              f"{marker_delay:.2f} | {vehicle_count} | {predicted_count} | "
              f"{collision_count} | {hits}")


if __name__ == "__main__":
    main()
