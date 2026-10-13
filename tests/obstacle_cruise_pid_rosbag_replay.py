"""K-City 기록의 원시 순항 거리·속도로 독립 PID 출력을 대조합니다."""

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import decode_float32_multi_array_stamped, messages

TOPIC = ("/planning/scenario_planning/lane_driving/motion_planning/"
         "obstacle_cruise_planner/debug/cruise_planning_info")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--warmup-active", type=int, default=0,
                        help="연속 활성 구간의 앞 프레임 수를 추가 진단에서 제외합니다")
    args = parser.parse_args()
    recorded = []
    for _, payload in messages(args.bag, TOPIC):
        values = decode_float32_multi_array_stamped(payload)["data"]
        if len(values) < 15:
            raise ValueError("순항 디버그 배열 길이가 부족합니다")
        recorded.append(values)
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory) / "입력.txt"
        target = Path(directory) / "출력.txt"
        with source.open("w") as stream:
            stream.write(f"{len(recorded)}\n")
            for values in recorded:
                stream.write(f"{int(values[3] != 0.0)} {values[0]} "
                             f"{values[3]} {values[5]}\n")
        subprocess.run([str(args.runner), str(source), str(target)], check=True)
        independent = [tuple(float(item) for item in line.split())
                       for line in target.read_text().splitlines()]
    assert len(recorded) == len(independent)
    active = 0
    max_velocity = 0.0
    max_target_distance = 0.0
    max_filtered_distance = 0.0
    max_filtered_error = 0.0
    warmup_max_velocity = 0.0
    warmup_compared = 0
    active_run = 0
    first_large = None
    for index, (expected, actual) in enumerate(zip(recorded, independent)):
        if expected[3] == 0.0:
            active_run = 0
            continue
        active += 1
        active_run += 1
        velocity_error = abs(expected[12] - actual[1])
        max_velocity = max(max_velocity, velocity_error)
        if active_run > args.warmup_active:
            warmup_compared += 1
            warmup_max_velocity = max(warmup_max_velocity, velocity_error)
        max_target_distance = max(max_target_distance,
                                  abs(expected[7] - actual[3]))
        max_filtered_distance = max(max_filtered_distance,
                                    abs(expected[4] - actual[4]))
        max_filtered_error = max(max_filtered_error,
                                 abs(expected[9] - actual[5]))
        if velocity_error > 0.01 and first_large is None:
            first_large = index
            print(f"첫 차이 원본 속도 {expected[12]:.6f}m/s, "
                  f"독립 속도 {actual[1]:.6f}m/s, "
                  f"원본 오차거리 {expected[8]:.6f}m, "
                  f"원본 필터 오차거리 {expected[9]:.6f}m")
    print(f"순항 디버그 {len(recorded)}프레임, 활성 {active}프레임, "
          f"목표 속도 최대 차이 {max_velocity:.6f}m/s, "
          f"RSS 거리 최대 차이 {max_target_distance:.6f}m, "
          f"필터 거리 최대 차이 {max_filtered_distance:.6f}m, "
          f"필터 오차 최대 차이 {max_filtered_error:.6f}m, "
          f"첫 0.01m/s 초과 {first_large}, "
          f"활성 앞 {args.warmup_active}프레임 제외 {warmup_compared}프레임 "
          f"최대 {warmup_max_velocity:.6f}m/s")


if __name__ == "__main__":
    main()
