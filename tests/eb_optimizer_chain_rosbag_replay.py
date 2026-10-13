"""기록된 행동 속도 경로에서 탄성 평활화와 최적화를 이어 실행합니다."""

import argparse
import bisect
import math
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import decode_odometry, decode_path, decode_trajectory, messages


PREFIX = "/planning/scenario_planning/lane_driving/"
INPUT = PREFIX + "behavior_planning/path"
REFERENCE = PREFIX + "motion_planning/path_optimizer/trajectory"
SMOOTH_REFERENCE = PREFIX + "motion_planning/path_smoother/path"
ODOMETRY = "/localization/kinematic_state"


def write_point(stream, point):
    stream.write(" ".join(str(value) for value in point) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--compare-after", type=int, default=30)
    parser.add_argument("--assert-ipc-stopline-chain", action="store_true",
                        help="IPC 정지선 두 단계 연속 계산 기준을 검사합니다")
    parser.add_argument("--assert-ipc-stopline-full-chain", action="store_true",
                        help="IPC 정지선 전체 571프레임의 계산 성공을 검사합니다")
    parser.add_argument("--assert-ipc-stopline-state-at-smooth", action="store_true",
                        help="평활화 경로 수신 시각의 차량 상태로 첫 100프레임을 검사합니다")
    parser.add_argument("--assert-ipc-stopline-state-at-smooth-full", action="store_true",
                        help="두 단계별 차량 상태 시각으로 전체 571프레임을 검사합니다")
    parser.add_argument("--optimizer-recorded-smooth", action="store_true",
                        help="진단용으로 최적화기에 기록된 평활화 경로를 넣습니다")
    parser.add_argument("--state-at-smooth", action="store_true",
                        help="최적화에만 평활화 경로 수신 시각의 차량 상태를 사용합니다")
    parser.add_argument("--goal-at-smooth", action="store_true",
                        help="진단용으로 평활화 경로의 종점 포함 값을 사용합니다")
    parser.add_argument("--trace-frames", type=str, default="",
                        help="쉼표로 구분한 프레임의 평활화 경로 차이를 표시합니다")
    args = parser.parse_args()
    trace_frames = {int(value) for value in args.trace_frames.split(",")
                    if value.strip()}
    odometry = list(messages(args.bag, ODOMETRY))
    odometry_times = [time for time, _ in odometry]
    references = {item["stamp_ns"]: item["points"]
                  for _, payload in messages(args.bag, REFERENCE)
                  if (item := decode_trajectory(payload))}
    smooth_references = {item["stamp_ns"]: item["points"]
                         for _, payload in messages(args.bag, SMOOTH_REFERENCE)
                         if (item := decode_path(payload))}
    smooth_times = {item["stamp_ns"]: time
                    for time, payload in messages(args.bag, SMOOTH_REFERENCE)
                    if (item := decode_path(payload))}
    frames = []
    missing_smooth_receipts = []
    for time, payload in messages(args.bag, INPUT):
        path = decode_path(payload)
        behavior_index = bisect.bisect_right(odometry_times, time) - 1
        optimizer_time = (smooth_times.get(path["stamp_ns"], time)
                          if args.state_at_smooth else time)
        optimizer_index = bisect.bisect_right(odometry_times, optimizer_time) - 1
        if min(behavior_index, optimizer_index) < 0 or \
                path["stamp_ns"] not in references:
            continue
        if args.state_at_smooth and path["stamp_ns"] not in smooth_times:
            missing_smooth_receipts.append(len(frames))
        frames.append((time, path,
                       decode_odometry(odometry[behavior_index][1]),
                       decode_odometry(odometry[optimizer_index][1])))
        if len(frames) >= args.frames:
            break
    if not frames:
        raise RuntimeError("연속 계산에 사용할 경로와 차량 상태가 없습니다")
    with tempfile.TemporaryDirectory() as directory:
        source_file = Path(directory) / "입력.txt"
        output_file = Path(directory) / "출력.txt"
        with source_file.open("w") as stream:
            stream.write(f"{len(frames)}\n")
            for time, path, behavior_ego, optimizer_ego in frames:
                points = path["points"]
                goal_flag = (smooth_references[path["stamp_ns"]][-1][10]
                             if args.goal_at_smooth and
                             path["stamp_ns"] in smooth_references else
                             points[-1][10])
                stream.write(f"{time / 1e9} {int(goal_flag)} "
                             f"{len(points)} {len(path['left_bound'])} "
                             f"{len(path['right_bound'])}\n")
                for ego in (behavior_ego, optimizer_ego):
                    write_point(stream, (ego["x"], ego["y"], ego["z"],
                                         ego["orientation_x"], ego["orientation_y"],
                                         ego["orientation_z"], ego["orientation_w"],
                                         ego["velocity_mps"]))
                for point in points:
                    write_point(stream, point[:8])
                for bound in path["left_bound"]:
                    write_point(stream, bound[:2])
                for bound in path["right_bound"]:
                    write_point(stream, bound[:2])
                if args.optimizer_recorded_smooth:
                    recorded_smooth = smooth_references.get(
                        path["stamp_ns"], [])
                    stream.write(f"{len(recorded_smooth)}\n")
                    for point in recorded_smooth:
                        write_point(stream, point[:8])
        command = [str(args.runner), str(source_file), str(output_file)]
        if args.optimizer_recorded_smooth:
            command.append("--optimizer-recorded-smooth")
        subprocess.run(command, check=True)
        solved = count_mismatches = 0
        smooth_count_mismatches = 0
        maximum_position = maximum_velocity = 0.0
        smooth_maximum_position = smooth_maximum_velocity = 0.0
        worst_position_frame = worst_velocity_frame = None
        count_difference_frames = []
        with output_file.open() as stream:
            for index, (_, path, _, _) in enumerate(frames):
                smooth_count, success = map(int, stream.readline().split())
                smooth_output = [tuple(map(float, stream.readline().split()))
                                 for _ in range(smooth_count)]
                if success:
                    solved += 1
                    count = int(stream.readline())
                    output = [tuple(map(float, stream.readline().split()))
                              for _ in range(count)]
                else:
                    output = []
                if index < args.compare_after:
                    continue
                smooth_expected = smooth_references.get(path["stamp_ns"])
                if smooth_expected:
                    smooth_count_mismatches += len(smooth_output) != len(smooth_expected)
                    smooth_maximum_position = max(
                        smooth_maximum_position,
                        max((math.hypot(point[0] - target[0],
                                        point[1] - target[1])
                             for point, target in
                             zip(smooth_output, smooth_expected)), default=0.0))
                    smooth_maximum_velocity = max(
                        smooth_maximum_velocity,
                        max((abs(point[2] - target[7])
                             for point, target in
                             zip(smooth_output, smooth_expected)), default=0.0))
                    if index in trace_frames:
                        point_errors = [math.hypot(point[0] - target[0],
                                                   point[1] - target[1])
                                        for point, target in
                                        zip(smooth_output, smooth_expected)]
                        print(f"{index}번 평활화: 점 수 "
                              f"{len(smooth_output)}/{len(smooth_expected)}, "
                              f"위치 최대 {max(point_errors, default=0.0):.6f}m, "
                              f"최대 점 {point_errors.index(max(point_errors)) if point_errors else '-'}, "
                              f"종점 차이 {point_errors[-1] if point_errors else 0.0:.6f}m")
                if not output:
                    continue
                expected = references[path["stamp_ns"]]
                if len(output) != len(expected):
                    count_mismatches += 1
                    count_difference_frames.append(index)
                position_error = max(
                    (math.hypot(point[0] - target["x"],
                                point[1] - target["y"])
                     for point, target in zip(output, expected)), default=0.0)
                velocity_error = max(
                    (abs(point[2] - target["velocity"])
                     for point, target in zip(output, expected)), default=0.0)
                if position_error > maximum_position:
                    maximum_position = position_error
                    worst_position_frame = index
                if velocity_error > maximum_velocity:
                    maximum_velocity = velocity_error
                    worst_velocity_frame = index
        print(f"입력 {len(frames)}프레임, 최적화 성공 {solved}, "
              f"비교 시작 {args.compare_after}, "
              f"평활화 점 수 차이 {smooth_count_mismatches}, "
              f"평활화 위치 차이 {smooth_maximum_position:.6f}m, "
              f"평활화 속도 차이 {smooth_maximum_velocity:.6f}m/s, "
              f"최적화 점 수 차이 {count_mismatches}, "
              f"최대 공통 점 위치 차이 {maximum_position:.6f}m "
              f"({worst_position_frame}번), 속도 차이 "
              f"{maximum_velocity:.6f}m/s ({worst_velocity_frame}번)")
        if args.state_at_smooth:
            print(f"평활화 수신 시각 누락으로 행동 경로 시각을 사용한 프레임: "
                  f"{missing_smooth_receipts}")
        if len(count_difference_frames) <= 25:
            print(f"점 수 차이 프레임: {count_difference_frames}")
        else:
            print("점 수 차이 프레임 앞 25개: "
                  f"{count_difference_frames[:25]} "
                  f"(전체 {len(count_difference_frames)}개)")
        if args.assert_ipc_stopline_chain and (
                args.optimizer_recorded_smooth or args.state_at_smooth or
                args.goal_at_smooth or
                len(frames) != 100 or args.compare_after != 30 or
                solved != 100 or smooth_count_mismatches != 0 or
                smooth_maximum_position >= 0.002 or
                smooth_maximum_velocity >= 1e-6 or
                count_mismatches > 1):
            raise AssertionError("IPC 정지선 두 단계 연속 계산 기준과 다릅니다")
        if args.assert_ipc_stopline_full_chain and (
                args.optimizer_recorded_smooth or args.state_at_smooth or
                args.goal_at_smooth or len(frames) != 571 or
                args.compare_after != 30 or solved != 571 or
                smooth_count_mismatches != 0 or
                smooth_maximum_position >= 0.002 or
                count_mismatches > 4):
            raise AssertionError("IPC 정지선 전체 두 단계 연속 계산 기준과 다릅니다")
        if args.assert_ipc_stopline_state_at_smooth and (
                not args.state_at_smooth or args.optimizer_recorded_smooth or
                args.goal_at_smooth or len(frames) != 100 or
                args.compare_after != 30 or solved != 100 or
                any(index >= args.compare_after for index in missing_smooth_receipts) or
                smooth_count_mismatches != 0 or count_mismatches != 0 or
                smooth_maximum_position >= 0.002 or
                smooth_maximum_velocity >= 1e-6 or
                maximum_position >= 0.06 or maximum_velocity >= 1e-6):
            raise AssertionError("평활화 수신 시각 기준 두 단계 연속 계산과 다릅니다")
        if args.assert_ipc_stopline_state_at_smooth_full and (
                not args.state_at_smooth or args.optimizer_recorded_smooth or
                args.goal_at_smooth or len(frames) != 571 or
                args.compare_after != 30 or solved != 571 or
                any(index >= args.compare_after for index in missing_smooth_receipts) or
                smooth_count_mismatches != 0 or count_mismatches != 0 or
                smooth_maximum_position >= 0.002 or
                maximum_position >= 0.06 or maximum_velocity >= 1e-6):
            raise AssertionError("전체 평활화·최적화 시각 경계 재생과 다릅니다")


if __name__ == "__main__":
    main()
