"""원본 감속 구간·속도에서 독립 경로 삽입 결과를 대조합니다."""

import argparse
import bisect
import math
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import (
    decode_float32_multi_array_stamped, decode_trajectory, messages)

PREFIX = "/planning/scenario_planning/lane_driving/"
INPUT = PREFIX + "motion_planning/motion_velocity_planner/trajectory"
OUTPUT = PREFIX + "trajectory"
DEBUG = (PREFIX + "motion_planning/obstacle_cruise_planner/"
         "debug/slow_down_planning_info")
STOP_DEBUG = (PREFIX + "motion_planning/obstacle_cruise_planner/"
              "debug/stop_planning_info")


def following_debug(records, times, timestamp):
    index = bisect.bisect_right(times, timestamp)
    if index >= len(records) or records[index][0] - timestamp > 20_000_000:
        return None
    return decode_float32_multi_array_stamped(records[index][1])["data"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--assert-hpc-scurve-reference", action="store_true",
                        help="HPC S자 감속 구간 삽입 기준을 검사합니다")
    args = parser.parse_args()
    inputs = {item["stamp_ns"]: item["points"]
              for _, payload in messages(args.bag, INPUT)
              if (item := decode_trajectory(payload))}
    debug = list(messages(args.bag, DEBUG))
    stop_debug = list(messages(args.bag, STOP_DEBUG))
    debug_times = [time for time, _ in debug]
    stop_times = [time for time, _ in stop_debug]
    frames = []
    skipped_stop = 0
    for time, payload in messages(args.bag, OUTPUT):
        expected = decode_trajectory(payload)
        source = inputs.get(expected["stamp_ns"])
        if source is None:
            continue
        values = following_debug(debug, debug_times, time)
        if values is None or len(values) != 7:
            continue
        stop_values = following_debug(stop_debug, stop_times, time)
        if stop_values and len(stop_values) > 3 and stop_values[3] > 0.0:
            skipped_stop += 1
            continue
        frames.append((source, expected["points"], values))
    with tempfile.TemporaryDirectory() as directory:
        source_file = Path(directory) / "입력.txt"
        output_file = Path(directory) / "출력.txt"
        with source_file.open("w") as stream:
            stream.write(f"{len(frames)}\n")
            for points, _, values in frames:
                stream.write(f"{len(points)} {values[1]} {values[2]} {values[4]}\n")
                for point in points:
                    stream.write(" ".join(str(point[name]) for name in (
                        "x", "y", "z", "orientation_x", "orientation_y",
                        "orientation_z", "orientation_w", "velocity")) + "\n")
        subprocess.run((str(args.runner), str(source_file), str(output_file)),
                       check=True)
        computed = []
        with output_file.open() as stream:
            for _ in frames:
                count = int(stream.readline())
                computed.append([tuple(float(value) for value in
                                       stream.readline().split())
                                 for _ in range(count)])
    count_differences = 0
    maximum_position = maximum_velocity = 0.0
    for actual, (_, expected, _) in zip(computed, frames):
        count_differences += len(actual) != len(expected)
        for (x, y, velocity), point in zip(actual, expected):
            maximum_position = max(maximum_position,
                                   math.hypot(x - point["x"], y - point["y"]))
            maximum_velocity = max(maximum_velocity,
                                   abs(velocity - point["velocity"]))
    print(f"감속 활성 {len(frames)}프레임, 동시 정지로 제외 {skipped_stop}프레임, "
          f"점 수 차이 {count_differences}프레임, 위치 최대 차이 "
          f"{maximum_position:.6f}m, 속도 최대 차이 {maximum_velocity:.6f}m/s")
    if args.assert_hpc_scurve_reference and (
            len(frames) != 35 or skipped_stop != 5 or
            count_differences != 0 or maximum_position >= 1e-5 or
            maximum_velocity >= 1e-6):
        raise AssertionError("HPC S자 감속 구간 삽입 기준과 다릅니다")


if __name__ == "__main__":
    main()
