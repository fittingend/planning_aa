"""Compare the original free-grid parking trajectory with the independent core."""

import argparse
import json
import math
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("reference", type=Path)
    parser.add_argument("--completion-reference", type=Path)
    parser.add_argument("--check-next-prefix", type=int, default=0)
    parser.add_argument("--maximum-full-path-position-error", type=float)
    parser.add_argument("--maximum-full-path-endpoint-error", type=float)
    args = parser.parse_args()
    case = json.loads(args.reference.read_text())
    inputs = case["input"]["ego"] + case["input"]["goal"]
    obstacle = case["input"].get("obstacle")
    if obstacle:
        inputs += [obstacle["forward_m"], obstacle["half_length_m"],
                   obstacle["half_width_m"]]
    process = subprocess.run([str(args.runner), *map(str, inputs)],
                             capture_output=True, text=True, check=True)
    independent = [[float(value) for value in line.split()]
                   for line in process.stdout.splitlines()]
    original = case["output"]["points"]
    if not independent:
        raise AssertionError("독립 자유공간 경로가 없습니다")
    start_error = math.hypot(independent[0][0] - original[0][0],
                             independent[0][1] - original[0][1])
    end_error = math.hypot(independent[-1][0] - original[-1][0],
                           independent[-1][1] - original[-1][1])
    speed_error = abs(independent[0][2] - original[0][4])
    point_errors = [math.hypot(a[0] - b[0], a[1] - b[1])
                    for a, b in zip(independent, original)]
    max_point_error = max(point_errors) if point_errors else math.inf
    max_speed_error = max((abs(a[2] - b[4])
                           for a, b in zip(independent, original)),
                          default=math.inf)
    print(f"원본/독립 점 수: {len(original)}/{len(independent)}, "
          f"시작 차이 {start_error:.6f}m, 종점 차이 {end_error:.6f}m, "
          f"점별 최대 차이 {max_point_error:.6f}m, "
          f"속도 최대 차이 {max_speed_error:.6f}m/s")
    if start_error > 1e-6 or end_error > 0.5 or speed_error > 1e-6 or \
            abs(independent[-1][2]) > 1e-6:
        raise AssertionError("원본의 시작·목표 근접·정지 기본 계약과 다릅니다")
    if len(independent) != len(original) or max_point_error > 0.001 or \
            max_speed_error > 1e-6:
        raise AssertionError("합성 주차 원본의 첫 발행 경로와 다릅니다")
    if args.maximum_full_path_position_error is not None:
        expected_full = case["output"]["full_path_poses"]
        actual_full = [[float(value) for value in line.split()[1:]]
                       for line in process.stderr.splitlines()
                       if line.startswith("FULLPOINT ")]
        if len(expected_full) != len(actual_full):
            raise AssertionError("원본/독립 전체 A* 경로의 점 수가 다릅니다")
        full_errors = [math.dist(a[:2], b[:2])
                       for a, b in zip(expected_full, actual_full)]
        full_endpoint_error = math.dist(expected_full[-1][:2],
                                        actual_full[-1][:2])
        print(f"전체 경로: 원본/독립 {len(expected_full)}/{len(actual_full)}점, "
              f"최대 위치 차이 {max(full_errors):.6f}m, "
              f"종점 차이 {full_endpoint_error:.6f}m")
        if max(full_errors) > args.maximum_full_path_position_error:
            raise AssertionError("전체 A* 경로의 위치 차이가 허용값을 넘었습니다")
        if args.maximum_full_path_endpoint_error is not None and \
                full_endpoint_error > args.maximum_full_path_endpoint_error:
            raise AssertionError("전체 A* 경로의 종점 차이가 허용값을 넘었습니다")
    if args.check_next_prefix:
        expected = case["segment_transition"]["points"]
        actual = [[float(value) for value in line.split()[1:]]
                  for line in process.stderr.splitlines()
                  if line.startswith("NEXTPOINT ")]
        count = args.check_next_prefix
        if len(expected) < count or len(actual) < count:
            raise AssertionError("원본/독립 다음 구간의 접두 경로가 부족합니다")
        next_position_error = max(math.hypot(a[0] - b[0], a[1] - b[1])
                                  for a, b in zip(actual[:count], expected[:count]))
        next_speed_error = max(abs(a[2] - b[2])
                               for a, b in zip(actual[:count], expected[:count]))
        print(f"다음 구간: 원본/독립 {len(expected)}/{len(actual)}점, "
              f"처음 {count}점 위치 최대 차이 {next_position_error:.6f}m, "
              f"속도 최대 차이 {next_speed_error:.6f}m/s")
        if next_position_error > 0.002 or next_speed_error > 1e-6:
            raise AssertionError("원본 다음 구간의 첫 점들이 다릅니다")
    if args.completion_reference:
        completion = json.loads(args.completion_reference.read_text())[
            "completion_transition"]
        if not completion["completed_history"][-1] or \
                not completion["all_final_velocities_zero"] or \
                "COMPLETION 1 1 1" not in process.stderr:
            raise AssertionError("원본/독립 완료·정지 상태 계약이 다릅니다")
        print("완료 true 발행·완료 뒤 전 경로 영속도: 원본/독립 일치")


if __name__ == "__main__":
    main()
