"""실측 odometry·경로·속도 제한으로 남은 거리·시간을 원본 출력과 비교합니다."""

import argparse
import bisect
import math
import statistics
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import (
    decode_lanelet_map, decode_lanelet_route, decode_odometry,
    decode_remaining_distance_time,
    decode_scenario, decode_velocity_limit, messages)


def latest(rows, times, timestamp):
    index = bisect.bisect_right(times, timestamp) - 1
    return rows[index] if index >= 0 else None


def yaw(pose):
    x, y, z, w = pose[3:7]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("map", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frames", type=int, default=0,
                        help="0이면 기록된 전체 출력을 비교합니다")
    parser.add_argument("--maximum-distance-error-m", type=float,
                        default=None)
    parser.add_argument("--maximum-frames-above-1cm", type=int,
                        default=None)
    args = parser.parse_args()
    route_rows = [(t, decode_lanelet_route(raw))
                  for t, raw in messages(args.bag,
                                         "/planning/mission_planning/route")]
    odom_rows = [(t, decode_odometry(raw))
                 for t, raw in messages(args.bag,
                                        "/localization/kinematic_state")]
    scenario_rows = [(t, decode_scenario(raw))
                     for t, raw in messages(args.bag,
                         "/planning/scenario_planning/scenario")]
    limits = [(t, decode_velocity_limit(raw))
              for t, raw in messages(args.bag,
                  "/planning/scenario_planning/current_max_velocity")]
    outputs = [(t, decode_remaining_distance_time(raw))
               for t, raw in messages(args.bag,
                         "/planning/mission_remaining_distance_time")]
    if not route_rows or not odom_rows or not scenario_rows or not outputs:
        raise ValueError("경로·차량·시나리오·원본 출력이 모두 필요합니다")
    route_times = [t for t, _ in route_rows]
    odom_times = [t for t, _ in odom_rows]
    scenario_times = [t for t, _ in scenario_rows]
    limit_times = [t for t, _ in limits]
    selected = []
    for stamp, expected in outputs:
        route = latest(route_rows, route_times, stamp)
        odom = latest(odom_rows, odom_times, stamp)
        scenario = latest(scenario_rows, scenario_times, stamp)
        limit = latest(limits, limit_times, stamp)
        if not route or not odom or not scenario or \
                scenario[1]["current"] != "LaneDriving":
            continue
        selected.append((stamp, route[1], odom[1],
                         limit[1]["max_velocity_mps"] if limit else 99.99,
                         expected))
        if args.frames and len(selected) >= args.frames:
            break
    if not selected:
        raise ValueError("비교 가능한 차로 주행 출력이 없습니다")
    if len({row[1]["uuid"] for row in selected}) != 1:
        raise ValueError("경로가 바뀌는 기록은 아직 지원하지 않습니다")
    goal = selected[0][1]["goal"]
    with tempfile.TemporaryDirectory() as directory:
        map_source = args.map
        if str(map_source) == "-":
            map_message = next(messages(args.bag, "/map/vector_map"), None)
            if not map_message:
                raise ValueError("지도 메시지가 없습니다")
            map_source = Path(directory) / "map.bin"
            map_source.write_bytes(decode_lanelet_map(map_message[1]))
        source = Path(directory) / "input.txt"
        output_file = Path(directory) / "output.txt"
        with source.open("w") as stream:
            stream.write(f"{goal[0]} {goal[1]} {yaw(goal)} {goal[2]}\n")
            stream.write(f"{len(selected)}\n")
            for _, _, ego, velocity, _ in selected:
                pose = (ego["x"], ego["y"], ego["z"],
                        ego["orientation_x"], ego["orientation_y"],
                        ego["orientation_z"], ego["orientation_w"])
                stream.write(f"{ego['x']} {ego['y']} {yaw(pose)} "
                             f"{ego['z']} {velocity}\n")
        process = subprocess.run(
            [str(args.runner), str(map_source), str(source), str(output_file)],
            capture_output=True, text=True)
        if process.returncode:
            raise RuntimeError(f"계산기 실행 실패: {process.stderr}")
        actual = [tuple(map(float, line.split()))
                  for line in output_file.read_text().splitlines()]
    if len(actual) != len(selected):
        raise AssertionError("독립 출력 수가 원본 비교 수와 다릅니다")
    distance_errors = [abs(value[0] - row[4]["distance_m"])
                       for value, row in zip(actual, selected)]
    time_errors = [abs(value[1] - row[4]["time_s"])
                   for value, row in zip(actual, selected)]
    worst = max(range(len(selected)), key=distance_errors.__getitem__)
    within_mm = sum(error <= 0.001 for error in distance_errors)
    within_cm = sum(error <= 0.01 for error in distance_errors)
    print(f"차로 주행 {len(selected)}프레임, 거리 차이 중앙값 "
          f"{statistics.median(distance_errors):.6f}m/최대 "
          f"{distance_errors[worst]:.6f}m ({worst}번), "
          f"1mm 이내 {within_mm}프레임·1cm 이내 {within_cm}프레임, "
          f"시간 차이 중앙값 {statistics.median(time_errors):.6f}s/최대 "
          f"{max(time_errors):.6f}s")
    print("거리 최대 차이 프레임 원본/독립/자차/목표:",
          selected[worst][4]["distance_m"], actual[worst][0],
          (selected[worst][2]["x"], selected[worst][2]["y"]),
          (goal[0], goal[1]))
    worst_stamp = selected[worst][0]
    next_odom_index = bisect.bisect_right(odom_times, worst_stamp)
    alternate = [index for index in (next_odom_index - 2,
                                     next_odom_index)
                 if 0 <= index < len(odom_rows)]
    if alternate:
        with tempfile.TemporaryDirectory() as directory:
            map_source = args.map
            if str(map_source) == "-":
                map_message = next(messages(args.bag, "/map/vector_map"))
                map_source = Path(directory) / "map.bin"
                map_source.write_bytes(decode_lanelet_map(map_message[1]))
            source = Path(directory) / "alternate_input.txt"
            output_file = Path(directory) / "alternate_output.txt"
            with source.open("w") as stream:
                stream.write(f"{goal[0]} {goal[1]} {yaw(goal)} {goal[2]}\n")
                stream.write(f"{len(alternate)}\n")
                for index in alternate:
                    ego = odom_rows[index][1]
                    pose = (ego["x"], ego["y"], ego["z"],
                            ego["orientation_x"], ego["orientation_y"],
                            ego["orientation_z"], ego["orientation_w"])
                    stream.write(f"{ego['x']} {ego['y']} {yaw(pose)} "
                                 f"{ego['z']} {selected[worst][3]}\n")
            subprocess.run([str(args.runner), str(map_source), str(source),
                            str(output_file)], check=True)
            alternate_distances = [float(line.split()[0]) for line in
                                   output_file.read_text().splitlines()]
        print("최대 차이 프레임의 전후 odometry로 재계산한 거리:",
              [(round((odom_rows[index][0] - worst_stamp) / 1e6, 3),
                round(value, 6))
               for index, value in zip(alternate, alternate_distances)])
    print("첫 5개 거리 원본/독립:",
          [(round(row[4]["distance_m"], 3), round(value[0], 3))
           for row, value in zip(selected[:5], actual[:5])])
    if (args.maximum_distance_error_m is not None and
            max(distance_errors) > args.maximum_distance_error_m):
        raise AssertionError("남은 거리 최대 차이가 기준을 넘었습니다")
    if (args.maximum_frames_above_1cm is not None and
            len(selected) - within_cm > args.maximum_frames_above_1cm):
        raise AssertionError("남은 거리 1cm 초과 프레임 수가 기준을 넘었습니다")


if __name__ == "__main__":
    main()
