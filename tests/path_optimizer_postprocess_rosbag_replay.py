"""기록된 경로 최적화 내부 궤적에서 최종 출력 후처리를 비교합니다."""

import argparse
import bisect
import csv
import math
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import decode_odometry, decode_trajectory, messages

PREFIX = "/planning/scenario_planning/lane_driving/motion_planning/"
INPUT = PREFIX + "elastic_band_smoother/output/traj"
INTERNAL = PREFIX + "path_optimizer/debug/mpt_traj"
FINAL = PREFIX + "path_optimizer/trajectory"
ODOMETRY = "/localization/kinematic_state"


def write_trajectory(path, points):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        for point in points:
            writer.writerow(tuple(point[name] for name in (
                "x", "y", "z", "orientation_x", "orientation_y",
                "orientation_z", "orientation_w", "velocity")))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--indices", default="30,50,100,200")
    parser.add_argument("--start", type=int, default=30)
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()
    inputs = list(messages(args.bag, INPUT))
    internals = list(messages(args.bag, INTERNAL))
    finals = list(messages(args.bag, FINAL))
    odometry = list(messages(args.bag, ODOMETRY))
    internal_times = [time for time, _ in internals]
    final_times = [time for time, _ in finals]
    odometry_times = [time for time, _ in odometry]
    internal_by_stamp = {
        decode_trajectory(payload)["stamp_ns"]: index
        for index, (_, payload) in enumerate(internals)}
    final_by_stamp = {
        decode_trajectory(payload)["stamp_ns"]: index
        for index, (_, payload) in enumerate(finals)}
    indices = (range(args.start, min(args.start + args.frames, len(inputs)))
               if args.frames else (int(value) for value in args.indices.split(",")))
    maximum_position = 0.0
    maximum_velocity = 0.0
    count_differences = 0
    reused_internal = 0
    checked = 0
    for index in indices:
        input_time, input_payload = inputs[index]
        input_message = decode_trajectory(input_payload)
        stamp = input_message["stamp_ns"]
        final_index = final_by_stamp.get(stamp)
        odometry_index = bisect.bisect_right(odometry_times, input_time) - 1
        if final_index is None or odometry_index < 0:
            raise ValueError(f"{index}번 입력과 대응 출력이 없습니다")
        internal_index = internal_by_stamp.get(stamp)
        if internal_index is None:
            internal_index = bisect.bisect_right(
                internal_times, final_times[final_index]) - 1
        if internal_index < 0:
            raise ValueError(f"{index}번 출력 이전 내부 경로가 없습니다")
        reused_internal += stamp not in internal_by_stamp
        input_points = input_message["points"]
        optimized = decode_trajectory(internals[internal_index][1])["points"]
        reference = decode_trajectory(finals[final_index][1])["points"]
        ego = decode_odometry(odometry[odometry_index][1])
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            input_file = directory / "입력.csv"
            optimized_file = directory / "내부.csv"
            output_file = directory / "출력.csv"
            write_trajectory(input_file, input_points)
            write_trajectory(optimized_file, optimized)
            subprocess.run((str(args.runner), str(input_file), str(optimized_file),
                            str(output_file), str(ego["x"]), str(ego["y"])),
                           check=True, capture_output=True)
            with output_file.open(newline="") as stream:
                actual = [tuple(float(value) for value in row)
                          for row in csv.reader(stream)]
        shared = min(len(actual), len(reference))
        position = max(math.hypot(point[0] - saved["x"], point[1] - saved["y"])
                       for point, saved in zip(actual[:shared], reference[:shared]))
        velocity = max(abs(point[3] - saved["velocity"])
                       for point, saved in zip(actual[:shared], reference[:shared]))
        maximum_position = max(maximum_position, position)
        maximum_velocity = max(maximum_velocity, velocity)
        count_differences += len(actual) != len(reference)
        checked += 1
        if not args.frames:
            print(f"{index}번: 점 수 {len(actual)}/{len(reference)}, "
                  f"위치 {position:.6f}m, 속도 {velocity:.6f}m/s")
            if velocity > 0.01:
                differing = next(at for at, (point, saved) in enumerate(
                    zip(actual, reference))
                    if abs(point[3] - saved["velocity"]) > 0.01)
                stop = next((at for at, point in enumerate(input_points)
                             if abs(point["velocity"]) < 1e-4), None)
                print(f"  첫 속도 차이 {differing}번 점: "
                      f"{actual[differing][3]:.6f}/"
                      f"{reference[differing]['velocity']:.6f}m/s, "
                      f"입력 첫 정지점 {stop}")
        if args.strict and (len(actual) != len(reference) or position > 0.05
                            or velocity > 0.01):
            raise AssertionError(f"{index}번 후처리 출력이 허용 범위를 벗어났습니다: "
                                 f"{len(actual)}/{len(reference)}점, "
                                 f"{position:.6f}m, {velocity:.6f}m/s")
    print(f"후처리 {checked}프레임: 내부 경로 재사용 {reused_internal}, "
          f"점 수 차이 {count_differences}, "
          f"최대 위치 {maximum_position:.6f}m, "
          f"최대 속도 {maximum_velocity:.6f}m/s")


if __name__ == "__main__":
    main()
