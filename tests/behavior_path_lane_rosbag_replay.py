"""K-City 행동 경로 후보를 이진 지도 차로 이탈 검사에 재생합니다."""

import argparse
import bisect
import math
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import (
    decode_lanelet_map, decode_marker_array, decode_path, messages,
)


MODULES = ("lane_change_left", "start_planner",
           "static_obstacle_avoidance")


def crop_start_shift(bag, active):
    topic = ("/planning/scenario_planning/lane_driving/behavior_planning/"
             "behavior_path_planner/info/start_planner")
    markers = list(messages(bag, topic))
    times = [row[0] for row in markers]
    cropped = []
    for time, points in active:
        index = bisect.bisect_left(times, time)
        choices = [i for i in (index - 1, index)
                   if 0 <= i < len(markers) and
                   abs(times[i] - time) <= 50_000_000]
        if not choices:
            continue
        matched = min(choices, key=lambda i: abs(times[i] - time))
        poses = {marker["namespace"]: marker["pose"]
                 for marker in decode_marker_array(markers[matched][1])}
        if "start_pose" not in poses or "end_pose" not in poses:
            continue
        start = poses["start_pose"]
        end = poses["end_pose"]
        first = min(range(len(points)), key=lambda i:
                    math.hypot(points[i][0] - start[0],
                               points[i][1] - start[1]))
        last = min(range(len(points)), key=lambda i:
                   math.hypot(points[i][0] - end[0],
                              points[i][1] - end[1]))
        if first <= last:
            cropped.append((time, points[first:last + 1]))
    return cropped


def inspect(runner, bag, module, shift_only=False):
    topic = "/planning/path_candidate/" + module
    active = [(time, decode_path(payload)["points"])
              for time, payload in messages(bag, topic)]
    active = [(time, points) for time, points in active if points]
    if shift_only:
        active = crop_start_shift(bag, active)
    if not active:
        return None
    map_record = next(messages(bag, "/map/vector_map"), None)
    if map_record is None:
        raise RuntimeError(f"이진 지도가 없는 기록: {bag}")
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        map_path = root / "지도.bin"
        input_path = root / "입력.txt"
        output_path = root / "출력.txt"
        map_path.write_bytes(decode_lanelet_map(map_record[1]))
        with input_path.open("w") as stream:
            stream.write(f"{len(active)}\n")
            for _, points in active:
                stream.write(f"{len(points)}\n")
                for point in points:
                    stream.write(" ".join(str(value) for value in
                                          (point[:2] + point[3:7])) + "\n")
        subprocess.run([str(runner), str(map_path), str(input_path),
                        str(output_path)], check=True)
        rows = [tuple(map(float, line.split()))
                for line in output_path.read_text().splitlines()]
    if len(rows) != len(active):
        raise RuntimeError("독립 차로 검사 결과 수가 후보 수와 다릅니다")
    departures = [(index, int(row[1]), row[2])
                  for index, row in enumerate(rows) if row[1] >= 0]
    return len(active), len(departures), departures[:10], (
        min(int(row[0]) for row in rows), max(int(row[0]) for row in rows))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag_root", type=Path)
    args = parser.parse_args()
    print("기록 | 모듈 | 비어 있지 않은 원본 후보 | 독립 차로 이탈 | "
          "후보 차로 수 범위 | 첫 이탈 사례")
    for bag in sorted(args.bag_root.glob("*/*/*.db3")):
        if not bag.is_file():
            continue
        for module in MODULES:
            result = inspect(args.runner, bag, module)
            if result is None:
                continue
            count, departure_count, examples, lane_count = result
            label = "/".join(bag.relative_to(args.bag_root).parts[:2])
            print(f"{label} | {module} | {count} | {departure_count} | "
                  f"{lane_count[0]}~{lane_count[1]} | {examples}")
            if module == "start_planner":
                shifted = inspect(args.runner, bag, module, shift_only=True)
                if shifted:
                    count, departure_count, examples, lane_count = shifted
                    print(f"{label} | start_planner(표식 이동구간) | "
                          f"{count} | {departure_count} | "
                          f"{lane_count[0]}~{lane_count[1]} | {examples}")


if __name__ == "__main__":
    main()
