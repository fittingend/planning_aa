"""경로 최적화 기준 경로의 형상을 기록된 내부 기준점과 대조합니다."""

import argparse
import bisect
import csv
import math
import subprocess
import sys
import tempfile
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import decode_odometry, decode_trajectory, messages

PREFIX = "/planning/scenario_planning/lane_driving/motion_planning/"
INPUT = PREFIX + "elastic_band_smoother/output/traj"
REFERENCE = PREFIX + "path_optimizer/debug/mpt_ref_traj"
ODOMETRY = "/localization/kinematic_state"


def write_input(path, points):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        for point in points:
            writer.writerow(tuple(point[name] for name in (
                "x", "y", "z", "orientation_x", "orientation_y",
                "orientation_z", "orientation_w", "velocity")))


def alignment_error(actual, recorded, shift):
    size = min(len(actual), len(recorded) - shift)
    if size <= 2:
        return math.inf
    return max(math.hypot(actual[i][0] - recorded[i + shift]["x"],
                          actual[i][1] - recorded[i + shift]["y"])
               for i in range(size))


def yaw_error(actual, recorded, shift):
    size = min(len(actual), len(recorded) - shift)
    errors = []
    for i in range(2, size):
        ours = math.atan2(2 * actual[i][7] * actual[i][6],
                          1 - 2 * actual[i][6] ** 2)
        saved = recorded[i + shift]
        theirs = math.atan2(
            2 * (saved["orientation_w"] * saved["orientation_z"] +
                 saved["orientation_x"] * saved["orientation_y"]),
            1 - 2 * (saved["orientation_y"] ** 2 +
                     saved["orientation_z"] ** 2))
        errors.append(abs(math.remainder(ours - theirs, 2 * math.pi)))
    return max(errors, default=0.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--start", type=int, default=30)
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--recorded-previous", action="store_true")
    args = parser.parse_args()
    inputs = list(messages(args.bag, INPUT))
    reference_messages = [decode_trajectory(payload)
                          for _, payload in messages(args.bag, REFERENCE)]
    reference_indices = {message["stamp_ns"]: index
                         for index, message in enumerate(reference_messages)}
    odometry = list(messages(args.bag, ODOMETRY))
    odometry_times = [time for time, _ in odometry]
    shifts = Counter()
    maximum = 0.0
    maximum_yaw = 0.0
    maximum_index = None
    checked = 0
    for index in range(args.start, min(args.start + args.frames, len(inputs))):
        input_time, payload = inputs[index]
        input_message = decode_trajectory(payload)
        reference_index = reference_indices.get(input_message["stamp_ns"])
        ego_index = bisect.bisect_right(odometry_times, input_time) - 1
        if reference_index is None or ego_index < 0 or (
                args.recorded_previous and reference_index == 0):
            continue
        recorded = reference_messages[reference_index]["points"]
        ego = decode_odometry(odometry[ego_index][1])
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            input_file = directory / "입력.csv"
            output_file = directory / "기준점.csv"
            write_input(input_file, input_message["points"])
            command = [str(args.runner), str(input_file), str(output_file),
                       str(ego["x"]), str(ego["y"])]
            if args.recorded_previous:
                previous_file = directory / "이전기준점.csv"
                write_input(previous_file,
                            reference_messages[reference_index - 1]["points"])
                command.append(str(previous_file))
            subprocess.run(command,
                           check=True, capture_output=True)
            with output_file.open(newline="") as stream:
                actual = [tuple(float(value) for value in row)
                          for row in csv.reader(stream)]
        choices = [(alignment_error(actual, recorded, shift), shift)
                   for shift in (range(1) if args.recorded_previous else range(3))]
        error, shift = min(choices)
        maximum_yaw = max(maximum_yaw, yaw_error(actual, recorded, shift))
        shifts[shift] += 1
        checked += 1
        if error > maximum:
            maximum = error
            maximum_index = index
        if args.strict and error > 0.05:
            raise AssertionError(
                f"{index}번 기준 경로 차이가 큽니다: {error:.6f}m, "
                f"앞 고정점 {shift}개")
    print(f"기준 경로 {checked}프레임, 앞 고정점 {dict(shifts)}, "
          f"최대 정렬 위치 차이 {maximum:.6f}m ({maximum_index}번), "
          f"최대 방향 차이 {maximum_yaw:.6f}rad")


if __name__ == "__main__":
    main()
