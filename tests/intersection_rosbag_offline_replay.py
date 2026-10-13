"""컨테이너 없이 K-City 기록의 교차로 안전 판정과 경로를 코어에 대조합니다."""

import argparse
import bisect
from contextlib import ExitStack
import math
import re
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import (
    decode_acceleration, decode_lanelet_map, decode_odometry,
    decode_path, decode_string, messages)


BASE = "/planning/scenario_planning/lane_driving/behavior_planning/"
DECISION = BASE + "behavior_velocity_planner/debug/intersection/decision_state"


def nearest(records, stamps, target):
    insertion = bisect.bisect_left(stamps, target)
    candidates = [index for index in (insertion - 1, insertion)
                  if 0 <= index < len(records)]
    return records[min(candidates,
                       key=lambda index: abs(stamps[index] - target))]


def first_zero(points):
    return next((index for index, point in enumerate(points)
                 if abs(point[7]) < 1e-5), None)


def point_segment_distance(point, start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length2 = dx * dx + dy * dy
    ratio = max(0.0, min(1.0,
        ((point[0] - start[0]) * dx +
         (point[1] - start[1]) * dy) / length2)) if length2 else 0.0
    return math.hypot(point[0] - start[0] - ratio * dx,
                      point[1] - start[1] - ratio * dy)


def run_case(runner, map_file, input_record, output_record,
             decision_record, odometry_record, acceleration_record):
    input_time, input_data = input_record
    output_time, output_data = output_record
    decision_time, decision_data = decision_record
    assert 0 < decision_time - input_time < 100_000_000
    assert 0 < output_time - input_time < 100_000_000
    decision = decode_string(decision_data)
    assert re.search(r"intersection\d+\s*:\s*Safe:", decision), decision
    assert "occlusion_report:NotOccluded" in decision, decision

    source = decode_path(input_data, True)
    original = decode_path(output_data)["points"]
    ego = decode_odometry(odometry_record[1])
    acceleration = decode_acceleration(acceleration_record[1])
    assert first_zero(original) == len(original) - 1
    payload = [ego["x"], ego["y"], ego["velocity_mps"],
               acceleration["acceleration_mps2"], len(source["points"])]
    for point, lane_ids in zip(source["points"], source["lane_ids"]):
        payload.extend(point[:10])
        payload.extend((len(lane_ids), *lane_ids))
    payload.extend((0, 0))
    process = subprocess.run(
        [str(runner), str(map_file), "intersection_collision"],
        input=" ".join(map(str, payload)), text=True,
        capture_output=True, check=True)
    lines = process.stdout.splitlines()
    assert lines[:3] == ["0", "0", "0"], lines[:3]
    actual = [tuple(map(float, line.split())) for line in lines[4:]]
    assert len(actual) == int(lines[3])
    assert first_zero(actual) == len(actual) - 1
    distances = [min(
        point_segment_distance(point, original[index],
                               original[index + 1])
        for index in range(len(original) - 1)) for point in actual]
    max_distance = max(distances)
    worst_index = distances.index(max_distance)
    max_speed_difference = max(
        abs(point[7] - min(original,
                           key=lambda source_point: math.dist(
                               point[:2], source_point[:2]))[7])
        for point in actual)
    assert max_distance < 0.02, (
        input_time, max_distance, worst_index, len(actual),
        len(original), actual[worst_index][:2])
    assert max_speed_difference < 1e-6, (input_time, max_speed_difference)
    return max_distance, max_speed_difference


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("runner", type=Path)
    parser.add_argument("map_file", help="지도 파일 경로 또는 rosbag 내부 지도를 뜻하는 bag")
    parser.add_argument("bag", type=Path)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--indices", type=int, nargs="+")
    selection.add_argument("--sample-step", type=int)
    args = parser.parse_args()
    inputs = list(messages(args.bag, BASE + "path_with_lane_id"))
    outputs = list(messages(args.bag, BASE + "path"))
    decisions = list(messages(args.bag, DECISION))
    odometry = list(messages(args.bag, "/localization/kinematic_state"))
    acceleration = list(messages(args.bag, "/localization/acceleration"))
    assert all((inputs, outputs, decisions, odometry, acceleration))
    output_stamps = [record[0] for record in outputs]
    decision_stamps = [record[0] for record in decisions]
    odometry_stamps = [record[0] for record in odometry]
    acceleration_stamps = [record[0] for record in acceleration]
    with ExitStack() as stack:
        map_file = Path(args.map_file)
        if args.map_file == "bag":
            record = next(messages(args.bag, "/map/vector_map"), None)
            if record is None:
                raise ValueError("rosbag에 Lanelet2 지도가 없습니다")
            temporary_directory = Path(stack.enter_context(
                tempfile.TemporaryDirectory()))
            map_file = temporary_directory / "lanelet_map_inner.bin"
            map_file.write_bytes(decode_lanelet_map(record[1]))
        if args.sample_step is not None and args.sample_step <= 0:
            parser.error("표본 간격은 양수여야 합니다")
        indices = (args.indices if args.indices is not None else
                   range(0, len(inputs), args.sample_step))
        maximum_distance = 0.0
        examined = 0
        excluded = 0
        failures = []
        for index in indices:
            input_record = inputs[index]
            stamp = input_record[0]
            output_record = nearest(outputs, output_stamps, stamp + 24_000_000)
            decision_record = nearest(decisions, decision_stamps,
                                      stamp + 12_000_000)
            if args.sample_step is not None:
                decision = decode_string(decision_record[1])
                original = decode_path(output_record[1])["points"]
                if not (0 < output_record[0] - stamp < 100_000_000 and
                        0 < decision_record[0] - stamp < 100_000_000 and
                        re.search(r"intersection\d+\s*:\s*Safe:", decision) and
                        "occlusion_report:NotOccluded" in decision and
                        original and first_zero(original) == len(original) - 1):
                    excluded += 1
                    continue
            examined += 1
            try:
                distance, _ = run_case(
                    args.runner, map_file, input_record, output_record,
                    decision_record,
                    nearest(odometry, odometry_stamps, stamp),
                    nearest(acceleration, acceleration_stamps, stamp))
            except AssertionError as error:
                if args.indices is not None:
                    raise
                failures.append((index, str(error)))
                continue
            maximum_distance = max(maximum_distance, distance)
    print(f"K-City 원본 rosbag 검사 {examined}프레임: 통과 "
          f"{examined - len(failures)}, 실패 {len(failures)}, "
          f"비교 제외 {excluded}, 통과 최대 경로 편차 "
          f"{maximum_distance:.4f}m")
    for index, error in failures:
        print(f"  {index}프레임 실패: {error}")
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
