"""K-City 기록의 Elastic Band 내부 경로를 독립 QP 계산과 비교합니다."""

import argparse
import bisect
import csv
import math
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import decode_odometry, decode_path, decode_trajectory, messages

INPUT = "/planning/scenario_planning/lane_driving/behavior_planning/path"
INTERNAL = ("/planning/scenario_planning/lane_driving/motion_planning/"
            "elastic_band_smoother/debug/eb_traj")
FINAL = ("/planning/scenario_planning/lane_driving/motion_planning/"
         "elastic_band_smoother/output/traj")


def write_path(path, points, recorded=False):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        for point in points:
            if recorded:
                writer.writerow(tuple(point[name] for name in (
                    "x", "y", "z", "orientation_x", "orientation_y",
                    "orientation_z", "orientation_w", "velocity")))
            else:
                writer.writerow(point[:8])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--indices", default="10,30,100,300")
    parser.add_argument("--maximum-position-error", type=float, default=0.05)
    parser.add_argument("--compare-final", action="store_true")
    args = parser.parse_args()

    inputs = list(messages(args.bag, INPUT))
    outputs = list(messages(args.bag, INTERNAL))
    odometry = list(messages(args.bag, "/localization/kinematic_state"))
    final_outputs = list(messages(args.bag, FINAL)) if args.compare_final else []
    output_times = [timestamp for timestamp, _ in outputs]
    odometry_times = [timestamp for timestamp, _ in odometry]
    final_times = [timestamp for timestamp, _ in final_outputs]
    maximum = 0.0
    checked = 0
    for index in (int(value) for value in args.indices.split(",")):
        if index < 0 or index >= len(inputs):
            raise ValueError(f"입력 인덱스 {index}가 기록 범위를 벗어납니다")
        timestamp, payload = inputs[index]
        output_index = bisect.bisect_left(output_times, timestamp)
        ego_index = bisect.bisect_right(odometry_times, timestamp) - 1
        if output_index == 0 or output_index >= len(outputs) or ego_index < 0:
            raise ValueError(f"{index}번 입력의 이전 경로 또는 차량 상태가 없습니다")
        delay_ns = output_times[output_index] - timestamp
        if delay_ns >= 100_000_000:
            raise ValueError(f"{index}번 입력과 내부 출력의 시각 차이가 큽니다")
        points = decode_path(payload)["points"]
        previous = decode_trajectory(outputs[output_index - 1][1])["points"]
        reference = decode_trajectory(outputs[output_index][1])["points"]
        ego = decode_odometry(odometry[ego_index][1])
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            input_file = directory / "입력.csv"
            previous_file = directory / "이전.csv"
            output_file = directory / "출력.csv"
            write_path(input_file, points)
            write_path(previous_file, previous, recorded=True)
            subprocess.run((str(args.runner), str(input_file), str(output_file),
                            str(ego["x"]), str(ego["y"]), str(previous_file)),
                           check=True, capture_output=True)
            with output_file.open(newline="") as stream:
                rows = list(csv.reader(stream))
                optimized = [(float(row[1]), float(row[2]))
                             for row in rows if row[0] == "optimized"]
                finalized = [(float(row[1]), float(row[2]), float(row[3]))
                             for row in rows if row[0] == "finalized"]
        if len(optimized) != len(reference):
            raise AssertionError(f"{index}번 출력 점 수가 다릅니다: "
                                 f"{len(optimized)}/{len(reference)}")
        difference = max(math.hypot(x - point["x"], y - point["y"])
                         for (x, y), point in zip(optimized, reference))
        maximum = max(maximum, difference)
        checked += 1
        print(f"{index}번 입력: {len(optimized)}점, 최대 위치 차이 {difference:.6f}m")
        if args.compare_final:
            final_index = bisect.bisect_left(final_times, timestamp)
            if final_index >= len(final_outputs):
                raise ValueError("최종 출력 기록이 없습니다")
            final_reference = decode_trajectory(final_outputs[final_index][1])["points"]
            if len(finalized) != len(final_reference):
                raise AssertionError(
                    f"{index}번 최종 출력 점 수가 다릅니다: "
                    f"{len(finalized)}/{len(final_reference)}")
            final_position_error = max(
                math.hypot(x - point["x"], y - point["y"])
                for (x, y, _), point in zip(finalized, final_reference))
            final_velocity_error = max(
                abs(velocity - point["velocity"])
                for (_, _, velocity), point in zip(finalized, final_reference))
            final_velocity_index = max(range(len(finalized)), key=lambda at:
                abs(finalized[at][2] - final_reference[at]["velocity"]))
            print(f"{index}번 최종 출력: 최대 위치 차이 "
                  f"{final_position_error:.6f}m, 속도 차이 "
                  f"{final_velocity_error:.6f}m/s "
                  f"({final_velocity_index}번 점)")
            if final_velocity_error > 0.01:
                print("속도 차이 지점 주변: " + ", ".join(
                    f"{at}:{finalized[at][2]:.3f}/"
                    f"{final_reference[at]['velocity']:.3f}"
                    for at in range(max(0, final_velocity_index - 3),
                                    min(len(finalized), final_velocity_index + 4))))
        if difference > args.maximum_position_error:
            raise AssertionError(f"허용 위치 차이 {args.maximum_position_error}m 초과")
    print(f"Elastic Band 내부 경로 {checked}프레임 비교 완료, "
          f"최대 위치 차이 {maximum:.6f}m")


if __name__ == "__main__":
    main()
