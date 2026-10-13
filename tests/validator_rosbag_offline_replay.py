"""컨테이너 없이 실제 validator 상태와 독립 코어를 연속 비교합니다."""

import argparse
import bisect
import csv
import math
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import (
    decode_odometry, decode_planning_validator_status,
    decode_trajectory, messages)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frames", type=int, default=500)
    parser.add_argument("--wheelbase", type=float, default=2.79)
    parser.add_argument("--odometry-reference", choices=("input", "status"),
                        default="input")
    args = parser.parse_args()
    inputs = list(messages(
        args.bag, "/planning/scenario_planning/velocity_smoother/trajectory"))
    statuses = list(messages(
        args.bag, "/planning/planning_validator/validation_status"))
    odometry = list(messages(args.bag, "/localization/kinematic_state"))
    assert inputs and statuses and odometry
    input_stamps = [record[0] for record in inputs]
    odometry_stamps = [record[0] for record in odometry]
    pairs = []
    for timestamp, payload in statuses:
        status = decode_planning_validator_status(payload)
        input_index = bisect.bisect_right(input_stamps, timestamp) - 1
        if input_index < 0:
            continue
        candidates = range(max(0, input_index - 2),
                           min(len(inputs), input_index + 3))
        matching = [index for index in candidates
                    if abs(input_stamps[index] - timestamp) < 100_000_000 and
                    len(decode_trajectory(inputs[index][1])["points"]) ==
                    status["trajectory_size"]]
        if matching:
            input_index = min(matching,
                              key=lambda index: abs(input_stamps[index] - timestamp))
        input_timestamp = input_stamps[input_index]
        reference_time = (input_timestamp if args.odometry_reference == "input"
                          else timestamp)
        odometry_index = bisect.bisect_right(
            odometry_stamps, reference_time) - 1
        if odometry_index < 0 or abs(timestamp - input_timestamp) > 100_000_000:
            continue
        pairs.append((
            decode_trajectory(inputs[input_index][1])["points"],
            decode_odometry(odometry[odometry_index][1]),
            status))
        if len(pairs) == args.frames:
            break
    if not pairs:
        raise ValueError("비교할 validator 기록이 없습니다")
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory) / "trajectory.csv"
        with source.open("w", newline="") as stream:
            writer = csv.writer(stream)
            for index, (points, ego, _) in enumerate(pairs):
                for point in points:
                    writer.writerow((
                        index, point["x"], point["y"], point["z"],
                        point["orientation_x"], point["orientation_y"],
                        point["orientation_z"], point["orientation_w"],
                        point["velocity"], point["heading_rate"],
                        point["acceleration"], ego["x"], ego["y"],
                        ego["velocity_mps"]))
        initial = pairs[0][2]
        process = subprocess.run(
            (str(args.runner), str(source), str(args.wheelbase),
             str(initial["longitudinal_distance_deviation"]),
             str(initial["forward_trajectory_length_required"]),
             str(initial["forward_trajectory_length_measured"])),
            capture_output=True,
            text=True, check=True)
    observed = [{} for _ in pairs]
    for index, name, value in csv.reader(process.stdout.splitlines()):
        observed[int(index)][name] = float(value)
    mismatch_counts = {}
    mismatch_maxima = {}
    largest = (0.0, None, None)
    flag_mismatches = 0
    for index, ((_, _, expected), actual) in enumerate(zip(pairs, observed)):
        assert actual.keys() == expected.keys() - {"stamp_ns"}
        for name, value in actual.items():
            reference = float(expected[name])
            if not math.isfinite(value) or not math.isfinite(reference):
                continue
            difference = abs(value - reference)
            if difference > largest[0]:
                largest = (difference, index, name)
            if difference >= 1e-4:
                mismatch_counts[name] = mismatch_counts.get(name, 0) + 1
                mismatch_maxima[name] = max(
                    mismatch_maxima.get(name, 0.0), difference)
                if name.startswith("is_valid_"):
                    flag_mismatches += 1
    print(f"validator 실측 {len(pairs)}프레임 "
          f"(차량 상태 기준: {args.odometry_reference}, "
          "초기 진단 수치: 첫 기록), 불일치 항목 "
          f"{mismatch_counts}, 최대 차이 {largest}, "
          f"판정 불일치 {flag_mismatches}건")
    print(f"항목별 최대 차이 {mismatch_maxima}")
    if flag_mismatches:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
