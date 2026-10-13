"""K-City 실측 연속 궤적을 컨테이너 없이 스무더 코어에 재생합니다."""

import argparse
import bisect
import csv
import math
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import (
    decode_acceleration, decode_float32_stamped, decode_odometry, decode_trajectory,
    decode_operation_mode, decode_velocity_limit, messages, Reader)

INPUT_TOPIC = "/planning/scenario_planning/scenario_selector/trajectory"
OUTPUT_TOPIC = "/planning/scenario_planning/velocity_smoother/trajectory"


def nearest(records, stamps, target):
    insertion = bisect.bisect_left(stamps, target)
    candidates = [index for index in (insertion - 1, insertion)
                  if 0 <= index < len(records)]
    return records[min(candidates,
                       key=lambda index: abs(stamps[index] - target))]


def latest_before(records, stamps, target):
    index = bisect.bisect_right(stamps, target) - 1
    if index < 0:
        raise ValueError("프레임 이전의 차량 상태 기록이 없습니다")
    return records[index]


def write_points(path, points):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        for point in points:
            writer.writerow((
                point["x"], point["y"], point["velocity"],
                point["acceleration"], point["z"],
                point["orientation_x"], point["orientation_y"],
                point["orientation_z"], point["orientation_w"]))


def project_motion(points, ego):
    if len(points) == 1:
        return points[0]["velocity"], points[0]["acceleration"]
    best = (float("inf"), 0, 0.0)
    for index, (first, second) in enumerate(zip(points, points[1:])):
        dx = second["x"] - first["x"]
        dy = second["y"] - first["y"]
        length2 = dx * dx + dy * dy
        if length2 <= 1e-12:
            continue
        ratio = max(0.0, min(1.0,
            ((ego["x"] - first["x"]) * dx +
             (ego["y"] - first["y"]) * dy) / length2))
        distance2 = ((ego["x"] - first["x"] - ratio * dx) ** 2 +
                     (ego["y"] - first["y"] - ratio * dy) ** 2)
        if distance2 < best[0]:
            best = distance2, index, ratio
    _, index, ratio = best
    first, second = points[index:index + 2]
    return tuple(first[key] + ratio * (second[key] - first[key])
                 for key in ("velocity", "acceleration"))


def stop_distance(points, closest):
    distance = 0.0
    for index in range(closest, len(points)):
        if abs(float(points[index][2])) < 1e-3:
            return distance
        if index + 1 < len(points):
            distance += math.hypot(
                float(points[index + 1][0]) - float(points[index][0]),
                float(points[index + 1][1]) - float(points[index][1]))
    return float("inf")


def extract_path_around_ego(points, ego, ahead_m=200.0, behind_m=5.0):
    closest = min(range(len(points)), key=lambda index: math.hypot(
        points[index]["x"] - ego["x"],
        points[index]["y"] - ego["y"]))
    first = 0
    distance = 0.0
    for index in range(closest, 0, -1):
        distance += math.hypot(
            points[index]["x"] - points[index - 1]["x"],
            points[index]["y"] - points[index - 1]["y"])
        if distance > behind_m:
            first = index - 1
            break
    last = len(points) - 1
    distance = 0.0
    for index in range(closest, len(points) - 1):
        distance += math.hypot(
            points[index + 1]["x"] - points[index]["x"],
            points[index + 1]["y"] - points[index]["y"])
        if distance > ahead_m:
            last = index + 1
            break
    return points[first:last + 1]


def distance_to_path(point, path):
    best = float("inf")
    for first, second in zip(path, path[1:]):
        dx = second["x"] - first["x"]
        dy = second["y"] - first["y"]
        length2 = dx * dx + dy * dy
        ratio = max(0.0, min(1.0,
            ((point["x"] - first["x"]) * dx +
             (point["y"] - first["y"]) * dy) / length2)) if length2 else 0.0
        best = min(best, math.hypot(
            point["x"] - first["x"] - ratio * dx,
            point["y"] - first["y"] - ratio * dy))
    return best


def decode_kcity_limit_constraints(data):
    """보유 K-City 기록의 3개 제약 값과 사용 여부를 읽습니다."""
    reader = Reader(data)
    reader.value("i", 4)
    reader.value("I", 4)
    velocity = reader.value("f", 4)
    enabled = bool(reader.value("B", 1))
    minimum_acceleration = reader.value("f", 4)
    maximum_jerk = reader.value("f", 4)
    minimum_jerk = reader.value("f", 4)
    reader.text()
    if reader.offset < len(data) and any(data[reader.offset:]):
        raise ValueError("외부 속도 제한 메시지에 해석되지 않은 값이 있습니다")
    return velocity, enabled, minimum_acceleration, maximum_jerk, minimum_jerk


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("preprocess_runner", type=Path)
    parser.add_argument("smoother_runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--start", type=int)
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--maximum-velocity", type=float)
    parser.add_argument("--stateful-external-limit", action="store_true")
    parser.add_argument("--limit-plan-runner", type=Path)
    parser.add_argument("--trace-frame", type=int)
    args = parser.parse_args()
    inputs = list(messages(args.bag, INPUT_TOPIC))
    outputs = list(messages(args.bag, OUTPUT_TOPIC))
    odometry = list(messages(args.bag, "/localization/kinematic_state"))
    acceleration = list(messages(args.bag, "/localization/acceleration"))
    operation_modes = list(messages(
        args.bag, "/system/operation_mode/state"))
    velocity_limits = list(messages(
        args.bag, "/planning/scenario_planning/current_max_velocity"))
    debug_velocity = list(messages(
        args.bag, "/planning/scenario_planning/velocity_smoother/closest_velocity"))
    debug_acceleration = list(messages(
        args.bag, "/planning/scenario_planning/velocity_smoother/closest_acceleration"))
    assert args.frames > 0
    output_stamps = [record[0] for record in outputs]
    odometry_stamps = [record[0] for record in odometry]
    acceleration_stamps = [record[0] for record in acceleration]
    operation_mode_stamps = [record[0] for record in operation_modes]
    velocity_limit_stamps = [record[0] for record in velocity_limits]
    plan_runner = args.limit_plan_runner or (
        args.preprocess_runner.parent / "jerk_external_limit_plan_runner")
    if args.stateful_external_limit and not plan_runner.is_file():
        raise ValueError("외부 제한 거리 계산 실행기를 찾을 수 없습니다")
    debug_velocity_stamps = [record[0] for record in debug_velocity]
    debug_acceleration_stamps = [record[0] for record in debug_acceleration]
    if args.start is None:
        first_state_time = max(odometry_stamps[0], acceleration_stamps[0])
        args.start = bisect.bisect_left(
            [record[0] for record in inputs], first_state_time) + 2
    assert 0 <= args.start and args.start + args.frames <= len(inputs)
    elapsed_ms = []
    maximum_position_difference = 0.0
    maximum_endpoint_difference = 0.0
    worst_endpoint_frame = None
    maximum_speed_difference = 0.0
    worst_speed_frame = None
    worst_position_frame = None
    paired_frames = 0
    debug_paired_frames = 0
    maximum_internal_velocity_difference = 0.0
    maximum_internal_acceleration_difference = 0.0
    previous_internal = None
    previous_ego = None
    external_plan = None
    previous_limit_velocity = None
    if args.start > 0:
        previous_time = inputs[args.start - 1][0]
        prior_output_time, prior_output_payload = nearest(
            outputs, output_stamps, previous_time + 7_000_000)
        if 0 < prior_output_time - previous_time < 100_000_000:
            previous_internal = decode_trajectory(prior_output_payload)["points"]
    with tempfile.TemporaryDirectory() as directory:
        directory = Path(directory)
        input_file = directory / "input.csv"
        preprocess_file = directory / "preprocess.csv"
        resampled_file = directory / "resampled.csv"
        output_file = directory / "output.csv"
        previous_file = directory / "previous.csv"
        for frame in range(args.start, args.start + args.frames):
            timestamp, payload = inputs[frame]
            recorded_time, recorded_payload = nearest(
                outputs, output_stamps, timestamp + 7_000_000)
            next_input_time = (inputs[frame + 1][0]
                               if frame + 1 < len(inputs)
                               else timestamp + 100_000_000)
            limit_reference_time = (
                recorded_time if 0 < recorded_time - timestamp < 100_000_000
                and recorded_time < next_input_time else timestamp)
            raw = decode_trajectory(payload)["points"]
            ego = decode_odometry(latest_before(
                odometry, odometry_stamps, timestamp)[1])
            accel = decode_acceleration(latest_before(
                acceleration, acceleration_stamps, timestamp)[1])
            mode_index = bisect.bisect_right(operation_mode_stamps, timestamp) - 1
            operation_mode = (decode_operation_mode(
                operation_modes[mode_index][1]) if mode_index >= 0 else None)
            raw = extract_path_around_ego(raw, ego)
            limit_index = None
            if velocity_limits:
                limit_index = max(0, bisect.bisect_right(
                    velocity_limit_stamps, limit_reference_time) - 1)
            if args.maximum_velocity is not None:
                maximum_velocity = args.maximum_velocity
            elif limit_index is not None:
                maximum_velocity = decode_velocity_limit(
                    velocity_limits[limit_index][1])["max_velocity_mps"]
            else:
                maximum_velocity = 20.0
            if args.stateful_external_limit:
                if limit_index is not None:
                    _, enabled, minimum_acceleration, maximum_jerk, minimum_jerk = (
                        decode_kcity_limit_constraints(
                            velocity_limits[limit_index][1]))
                else:
                    enabled = False
                if not enabled:
                    minimum_acceleration = -1.0
                    maximum_jerk = 1.0
                    minimum_jerk = -1.0
                travel = (math.hypot(
                    ego["x"] - previous_ego["x"],
                    ego["y"] - previous_ego["y"])
                    if previous_ego else 0.0)
                if external_plan is None:
                    external_plan = [maximum_velocity, maximum_velocity, 0.0]
                elif (previous_limit_velocity is not None and
                      abs(maximum_velocity - previous_limit_velocity) > 1e-4 and
                      previous_internal):
                    old_v, old_a = project_motion(previous_internal, ego)
                    if abs(ego["velocity_mps"]) < 0.125 and old_v >= 0.25:
                        external_plan = [maximum_velocity, maximum_velocity, 0.0]
                    else:
                        plan_process = subprocess.run((
                            str(plan_runner), str(old_v), str(old_a),
                            str(maximum_velocity), str(minimum_acceleration),
                            str(maximum_jerk), str(minimum_jerk), "0.3",
                            str(travel)), capture_output=True, text=True, check=True)
                        external_plan = [float(value) for value in
                                         plan_process.stdout.strip().split(",")[:3]]
                else:
                    external_plan[2] = max(0.0, external_plan[2] - travel)
                previous_limit_velocity = maximum_velocity
                previous_ego = ego
            write_points(input_file, raw)
            pose = [str(ego[field]) for field in (
                "x", "y", "z", "orientation_x", "orientation_y",
                "orientation_z", "orientation_w")]
            preprocess_args = (
                ("--external-state", *map(str, external_plan))
                if args.stateful_external_limit else
                ("--extended", str(maximum_velocity)))
            subprocess.run((
                str(args.preprocess_runner), str(input_file),
                str(preprocess_file), str(ego["velocity_mps"]),
                str(accel["acceleration_mps2"]), *pose,
                *preprocess_args), check=True,
                capture_output=True)
            with preprocess_file.open(newline="") as stream:
                rows = [row[1:] for row in csv.reader(stream)
                        if row[0] == "resampled_full"]
            assert len(rows) >= 2
            with resampled_file.open("w", newline="") as stream:
                csv.writer(stream).writerows(rows)
            closest = min(range(len(rows)), key=lambda index: math.hypot(
                float(rows[index][0]) - ego["x"],
                float(rows[index][1]) - ego["y"]))
            initial_velocity = abs(ego["velocity_mps"])
            initial_acceleration = 0.0
            keep_closest = False
            large_deviation = False
            engaging = False
            if previous_internal:
                initial_velocity, initial_acceleration = project_motion(
                    previous_internal, ego)
                large_deviation = abs(abs(ego["velocity_mps"]) -
                                      abs(initial_velocity)) > 5.53
                keep_closest = large_deviation
                if large_deviation:
                    initial_velocity = abs(ego["velocity_mps"])
                elif (abs(ego["velocity_mps"]) < 0.125 and
                      float(rows[closest][2]) >= 0.25 and
                      stop_distance(rows, closest) > 0.5):
                    initial_velocity = 0.25
                    initial_acceleration = 0.5
                    keep_closest = True
                    engaging = True
                write_points(previous_file, previous_internal)
            if (previous_internal and not large_deviation and not engaging and
                    operation_mode and not (
                    operation_mode["is_autoware_control_enabled"] and
                    operation_mode["mode"] in (1, 2))):
                initial_velocity = min(
                    abs(float(rows[closest][2])), abs(ego["velocity_mps"]))
                initial_acceleration = max(-1.0, min(
                    1.0, accel["acceleration_mps2"]))
                keep_closest = True
            start = time.perf_counter()
            result = subprocess.run((
                str(args.smoother_runner), str(resampled_file),
                str(output_file), str(ego["velocity_mps"]),
                str(initial_acceleration), str(closest), *pose,
                str(initial_velocity), str(initial_acceleration),
                str(previous_file) if previous_internal else "",
                "1" if keep_closest else "0",
                str(external_plan[0] if args.stateful_external_limit
                    else maximum_velocity), "1"), capture_output=True)
            elapsed_ms.append((time.perf_counter() - start) * 1000.0)
            if result.returncode:
                raise RuntimeError(
                    f"{frame}프레임 스무더 실패: {result.stderr.decode()}")
            merged = []
            optimized = []
            internal = []
            actual = []
            with output_file.open(newline="") as stream:
                for row in csv.reader(stream):
                    if row[0] not in ("merged", "optimized", "internal", "output"):
                        continue
                    point = {"x": float(row[1]), "y": float(row[2]),
                             "velocity": float(row[3]),
                             "acceleration": float(row[4]),
                             "z": 0.0, "orientation_x": 0.0,
                             "orientation_y": 0.0,
                             "orientation_z": 0.0,
                             "orientation_w": 1.0}
                    {"merged": merged, "optimized": optimized, "internal": internal,
                     "output": actual}[row[0]].append(point)
            assert internal and actual
            assert all(math.isfinite(value)
                       for point in actual for value in (
                           point["x"], point["y"], point["velocity"],
                           point["acceleration"]))
            previous_internal = internal
            if frame == args.trace_frame:
                raw_zero = next((index for index, point in enumerate(raw)
                                 if point["velocity"] == 0.0), None)
                resampled_zero = next((index for index, point in enumerate(rows)
                                       if float(point[2]) == 0.0), None)
                internal_zero = next((index for index, point in enumerate(internal)
                                      if point["velocity"] == 0.0), None)
                optimized_zero = next((index for index, point in enumerate(optimized)
                                       if point["velocity"] == 0.0), None)
                output_zero = next((index for index, point in enumerate(actual)
                                    if point["velocity"] == 0.0), None)
                print(f"추적 {frame}프레임: 입력 시각 {timestamp}, "
                      f"출력 시각 {recorded_time}, 입력 점 {len(raw)}, "
                      f"재표본 점 {len(rows)}, 최근접 점 {closest}, "
                      f"최적화 점 {len(optimized)}, 내부 점 {len(internal)}, "
                      f"출력 점 {len(actual)}, "
                      f"차량 속도 {ego['velocity_mps']:.3f}m/s, "
                      f"초기 속도 {initial_velocity:.3f}m/s, "
                      f"초기 가속도 {initial_acceleration:.3f}m/s², "
                      f"첫 영속도 점 입력/재표본/최적화/내부 "
                      f"{raw_zero}/{resampled_zero}/{optimized_zero}/{internal_zero}, "
                      f"최종 첫 영속도 점 {output_zero}, "
                      f"제어 모드 {operation_mode}, "
                      f"외부 제한 {external_plan if external_plan else maximum_velocity}")
                for label, path in (("병합", merged), ("최적화", optimized)):
                    print(f"추적 {frame}프레임 {label} 앞점: " + ", ".join(
                        f"{point['velocity']:.3f}/{point['acceleration']:.3f}"
                        for point in path[:8]))
                print(f"추적 {frame}프레임 끝점: 재표본 "
                      f"{rows[-1][0]}/{rows[-1][1]}, 최적화 "
                      f"{optimized[-1]['x']:.3f}/{optimized[-1]['y']:.3f}; "
                      f"앞 5점 위치 " + ", ".join(
                          f"{point['x']:.3f}/{point['y']:.3f}"
                          for point in optimized[:5]))
                print(f"추적 {frame}프레임 최근접 후보: " + ", ".join(
                    f"{index}:{math.hypot(float(rows[index][0])-ego['x'], float(rows[index][1])-ego['y']):.3f}m/"
                    f"{float(rows[index][2]):.3f}m/s"
                    for index in range(max(0, closest-2), min(len(rows), closest+3))))
            if debug_velocity and debug_acceleration:
                debug_v_time, debug_v_data = nearest(
                    debug_velocity, debug_velocity_stamps, timestamp + 8_000_000)
                debug_a_time, debug_a_data = nearest(
                    debug_acceleration, debug_acceleration_stamps,
                    timestamp + 8_000_000)
                if (0 < debug_v_time - timestamp < 100_000_000 and
                        0 < debug_a_time - timestamp < 100_000_000 and
                        debug_v_time < next_input_time and
                        debug_a_time < next_input_time and
                        abs(debug_v_time - debug_a_time) < 20_000_000):
                    projected_v, projected_a = project_motion(internal, ego)
                    recorded_v = decode_float32_stamped(debug_v_data)["value"]
                    recorded_a = decode_float32_stamped(debug_a_data)["value"]
                    maximum_internal_velocity_difference = max(
                        maximum_internal_velocity_difference,
                        abs(projected_v - recorded_v))
                    maximum_internal_acceleration_difference = max(
                        maximum_internal_acceleration_difference,
                        abs(projected_a - recorded_a))
                    debug_paired_frames += 1
            if 0 < recorded_time - timestamp < 100_000_000:
                paired_frames += 1
                original = decode_trajectory(recorded_payload)["points"]
                if frame == args.trace_frame:
                    projected_v, projected_a = project_motion(internal, ego)
                    print(f"추적 {frame}프레임: 기록 출력 점 {len(original)}, "
                          f"독립 내부 최근접 속도 {projected_v:.3f}m/s, "
                          f"가속도 {projected_a:.3f}m/s², "
                          f"독립 출력 시작·끝 속도 "
                          f"{actual[0]['velocity']:.3f}/"
                          f"{actual[-1]['velocity']:.3f}m/s, "
                          f"기록 출력 시작·끝 속도 "
                          f"{original[0]['velocity']:.3f}/"
                          f"{original[-1]['velocity']:.3f}m/s")
                    print(f"추적 {frame}프레임 45~55번 속도: " + ", ".join(
                        f"{index}:{actual[index]['velocity']:.3f}/"
                        f"{original[index]['velocity']:.3f}"
                        for index in range(45, min(56, len(actual), len(original)))))
                endpoint_difference = math.hypot(
                    actual[-1]["x"] - original[-1]["x"],
                    actual[-1]["y"] - original[-1]["y"])
                if endpoint_difference > maximum_endpoint_difference:
                    maximum_endpoint_difference = endpoint_difference
                    worst_endpoint_frame = frame
                common_start = min(range(len(actual)), key=lambda index: math.hypot(
                    actual[index]["x"] - original[0]["x"],
                    actual[index]["y"] - original[0]["y"]))
                common_end = min(range(len(actual)), key=lambda index: math.hypot(
                    actual[index]["x"] - original[-1]["x"],
                    actual[index]["y"] - original[-1]["y"]))
                assert common_start <= common_end
                comparable = actual[common_start:common_end + 1]
                worst_point = max(
                    comparable, key=lambda point: distance_to_path(point, original))
                frame_position_difference = distance_to_path(worst_point, original)
                if frame_position_difference > maximum_position_difference:
                    maximum_position_difference = frame_position_difference
                    worst_position_frame = frame
                remaining_arc = [0.0] * len(actual)
                for index in range(len(actual) - 2, -1, -1):
                    remaining_arc[index] = remaining_arc[index + 1] + math.hypot(
                        actual[index + 1]["x"] - actual[index]["x"],
                        actual[index + 1]["y"] - actual[index]["y"])
                speed_points = [actual[index] for index in range(
                    common_start, common_end + 1)
                    if remaining_arc[index] > 5.0 and math.hypot(
                        actual[index]["x"] - original[-1]["x"],
                        actual[index]["y"] - original[-1]["y"]) > 5.0]
                if not speed_points:
                    continue
                frame_speed_difference = max(
                    abs(point["velocity"] - min(
                        original, key=lambda ref: math.hypot(
                            point["x"] - ref["x"],
                            point["y"] - ref["y"]))["velocity"])
                    for point in speed_points)
                if frame == args.trace_frame:
                    worst = max(speed_points, key=lambda point: abs(
                        point["velocity"] - min(
                            original, key=lambda ref: math.hypot(
                                point["x"] - ref["x"],
                                point["y"] - ref["y"]))["velocity"]))
                    matching = min(original, key=lambda ref: math.hypot(
                        worst["x"] - ref["x"], worst["y"] - ref["y"]))
                    actual_index = actual.index(worst)
                    matching_index = original.index(matching)
                    same_index = original[min(actual_index, len(original) - 1)]
                    print(f"추적 {frame}프레임: 최대 차이 지점 "
                          f"({worst['x']:.3f}, {worst['y']:.3f}), "
                          f"독립 속도 {worst['velocity']:.3f}m/s, "
                          f"기록 속도 {matching['velocity']:.3f}m/s, "
                          f"대응 위치 차이 {math.hypot(worst['x'] - matching['x'], worst['y'] - matching['y']):.3f}m, "
                          f"점 번호 {actual_index}/{matching_index}, "
                          f"같은 번호의 기록 속도 {same_index['velocity']:.3f}m/s")
                    for index in range(max(0, actual_index - 3),
                                       min(len(actual), actual_index + 4)):
                        source = original[min(index, len(original) - 1)]
                        print(f"추적 {frame}프레임 {index}번 점: 독립 "
                              f"{actual[index]['velocity']:.3f}m/s, 기록 "
                              f"{source['velocity']:.3f}m/s")
                if frame_speed_difference > maximum_speed_difference:
                    maximum_speed_difference = frame_speed_difference
                    worst_speed_frame = frame
    elapsed_ms.sort()
    p95 = elapsed_ms[int(0.95 * (len(elapsed_ms) - 1))]
    print(f"실측 {args.start}프레임부터 연속 {args.frames}프레임 수렴, "
          f"실행 호출 중앙값 "
          f"{statistics.median(elapsed_ms):.2f}ms, 95백분위 {p95:.2f}ms, "
          f"최대 {elapsed_ms[-1]:.2f}ms")
    print(f"기록 출력 공통 경로 {paired_frames}프레임의 진단용 비교: "
          f"경로 편차 최대 {maximum_position_difference:.3f}m "
          f"({worst_position_frame}프레임), 끝점 차이 최대 "
          f"{maximum_endpoint_difference:.3f}m "
          f"({worst_endpoint_frame}프레임), "
          f"종단 5m 제외 속도 차이 최대 {maximum_speed_difference:.3f}m/s "
          f"({worst_speed_frame}프레임)")
    print(f"원본 내부 궤적의 최근접 운동 상태 {debug_paired_frames}프레임 비교: "
          f"속도 차이 최대 {maximum_internal_velocity_difference:.3f}m/s, "
          f"가속도 차이 최대 "
          f"{maximum_internal_acceleration_difference:.3f}m/s²")


if __name__ == "__main__":
    main()
