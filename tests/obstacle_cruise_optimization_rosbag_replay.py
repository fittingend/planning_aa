"""K-City의 순항 활성 프레임으로 선택적 최적화 QP의 수렴을 점검합니다."""

import argparse
import bisect
import math
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import (decode_float32_multi_array_stamped,
                                decode_odometry, decode_trajectory, messages)

PREFIX = "/planning/scenario_planning/lane_driving/motion_planning/"
TRAJECTORY = PREFIX + "motion_velocity_planner/trajectory"
DEBUG = PREFIX + "obstacle_cruise_planner/debug/cruise_planning_info"
ODOMETRY = "/localization/kinematic_state"


def forward_length(points, x, y):
    arcs = [0.0]
    for i in range(1, len(points)):
        arcs.append(arcs[-1] + math.hypot(
            points[i]["x"] - points[i - 1]["x"],
            points[i]["y"] - points[i - 1]["y"]))
    nearest = min(range(len(points)), key=lambda i:
                  (points[i]["x"] - x) ** 2 +
                  (points[i]["y"] - y) ** 2)
    segment = min(nearest, len(points) - 2)
    if 0 < nearest < len(points) - 1:
        dx = points[nearest + 1]["x"] - points[nearest]["x"]
        dy = points[nearest + 1]["y"] - points[nearest]["y"]
        if ((x - points[nearest]["x"]) * dx +
                (y - points[nearest]["y"]) * dy <= 0.0):
            segment = nearest - 1
    dx = points[segment + 1]["x"] - points[segment]["x"]
    dy = points[segment + 1]["y"] - points[segment]["y"]
    length = math.hypot(dx, dy)
    ego_arc = arcs[segment] + ((x - points[segment]["x"]) * dx +
                               (y - points[segment]["y"]) * dy) / max(length, 1e-9)
    stop_arc = next((arcs[i] for i in range(nearest, len(points))
                     if abs(points[i]["velocity"]) < 1e-4), arcs[-1])
    return max(0.0, min(arcs[-1], stop_arc) - ego_arc)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag_root", type=Path)
    parser.add_argument("--per-bag", type=int, default=10)
    args = parser.parse_args()
    rows = []
    by_bag = {}
    for bag in sorted(args.bag_root.glob("*/*/*.db3")):
        if not bag.is_file():
            continue
        odometry = list(messages(bag, ODOMETRY))
        odometry_times = [item[0] for item in odometry]
        trajectories = []
        for time_ns, payload in messages(bag, TRAJECTORY):
            points = decode_trajectory(payload)["points"]
            if len(points) < 2:
                continue
            maximum_velocity = max(point["velocity"] for point in points)
            trajectories.append((time_ns, points, maximum_velocity))
        times = [item[0] for item in trajectories]
        count = 0
        for time_ns, payload in messages(bag, DEBUG):
            values = decode_float32_multi_array_stamped(payload)["data"]
            if len(values) < 6 or values[3] <= 0.0:
                continue
            index = bisect.bisect_right(times, time_ns) - 1
            if index < 0:
                continue
            odometry_index = bisect.bisect_right(odometry_times, time_ns) - 1
            if odometry_index < 0:
                continue
            ego = decode_odometry(odometry[odometry_index][1])
            points, maximum_velocity = trajectories[index][1:]
            length = forward_length(points, ego["x"], ego["y"])
            if length <= 0.0:
                continue
            rows.append((bag.parent.relative_to(args.bag_root),
                         values[0], values[3], values[5], length,
                         maximum_velocity))
            count += 1
            if count >= args.per_bag:
                break
        by_bag[str(bag.parent.relative_to(args.bag_root))] = count
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory) / "입력.txt"
        target = Path(directory) / "출력.txt"
        with source.open("w") as stream:
            stream.write(f"{len(rows)}\n")
            for _, speed, distance, object_speed, length, maximum in rows:
                stream.write(f"{speed} {distance} {object_speed} "
                             f"{length} {maximum}\n")
        subprocess.run([str(args.runner), str(source), str(target)], check=True)
        results = [line.split() for line in target.read_text().splitlines()]
    assert len(results) == len(rows)
    if any(item[0] == "1" and not all(math.isfinite(float(value))
                                      for value in item[1:])
           for item in results):
        raise AssertionError("수렴한 순항 최적화 결과에 유한하지 않은 값이 있습니다")
    for bag in by_bag:
        subset = [results[i] for i, row in enumerate(rows)
                  if str(row[0]) == bag]
        print(f"{bag}: 실측 활성 입력 {len(subset)}, "
              f"QP 수렴 {sum(item[0] == '1' for item in subset)}")
    print("이 결과는 실제 입력의 수렴 검사이며 원본 PID 출력과의 "
          "속도 일치 비교가 아닙니다")


if __name__ == "__main__":
    main()
