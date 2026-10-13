"""K-City 돌발 진입 기록의 실제 경로와 예측 객체로 충돌 앞단을 재생합니다."""

import argparse
import bisect
import math
import statistics
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import (decode_acceleration, decode_float32_multi_array_stamped,
                                decode_lanelet_map, decode_marker_array, decode_odometry,
                                decode_path, decode_predicted_objects, messages)

PATH = "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id"
OUTPUT_PATH = "/planning/scenario_planning/lane_driving/behavior_planning/path"
OBJECTS = "/perception/object_recognition/objects"
ODOMETRY = "/localization/kinematic_state"
ACCELERATION = "/localization/acceleration"
DEBUG = ("/planning/scenario_planning/lane_driving/behavior_planning/"
         "behavior_velocity_planner/debug/run_out/debug_values")
MARKER = DEBUG.rsplit("/", 1)[0]


def yaw(pose):
    qx, qy, qz, qw = pose[3:7]
    return math.atan2(2 * (qw * qz + qx * qy),
                      1 - 2 * (qy * qy + qz * qz))


def range_from_covariance(item):
    vx, vy = map(abs, item["twist"][:2])
    var_x, var_y = item["twist_variance_xy"]
    if var_x < 0 or var_y < 0:
        return None
    dx, dy = 1.96 * math.sqrt(var_x), 1.96 * math.sqrt(var_y)
    return (math.hypot(max(0, vx - dx), max(0, vy - dy)),
            math.hypot(vx + dx, vy + dy))


def ego_arc(path, x, y):
    distance = math.inf
    selected = 0.0
    traveled = 0.0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]
        squared = dx * dx + dy * dy
        if squared < 1e-12:
            continue
        ratio = ((x - path[i - 1][0]) * dx +
                 (y - path[i - 1][1]) * dy) / squared
        if i != 1:
            ratio = max(0.0, min(1.0, ratio))
        error = math.hypot(x - path[i - 1][0] - ratio * dx,
                           y - path[i - 1][1] - ratio * dy)
        if error < distance:
            distance = error
            selected = traveled + ratio * math.sqrt(squared)
        traveled += math.sqrt(squared)
    return selected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frames", type=int, default=10000)
    parser.add_argument("--diagnose", action="store_true")
    parser.add_argument("--prepare-path", action="store_true",
                        help="원본의 경로 끝 연장과 차량 앞 45m 자르기를 적용합니다")
    parser.add_argument("--smooth-path", action="store_true",
                        help="원본의 경로 속도 스무더를 함께 적용합니다")
    parser.add_argument("--constant-ego-speed", action="store_true",
                        help="진단용으로 경로 전 구간을 현재 차량 속도로 제한합니다")
    parser.add_argument("--analytical-forward", action="store_true",
                        help="실차 설정의 Analytical 스무더 전방 분기를 적용합니다")
    parser.add_argument("--analytical-full", action="store_true",
                        help="Analytical의 전방·역방향 감속을 적용합니다")
    parser.add_argument("--ego-speed-scale", type=float, default=1.0,
                        help="진단용 일정 속도 배율을 지정합니다")
    parser.add_argument("--trace-frames", type=str, default="",
                        help="재생 프레임 인덱스를 쉼표로 구분해 예측 시간을 출력합니다")
    parser.add_argument("--compare-travel-time-markers", action="store_true",
                        help="원본 충돌 표식의 마지막 검사점 예측 시간을 대조합니다")
    parser.add_argument("--use-recorded-output-velocity", action="store_true",
                        help="진단용으로 기록된 행동 속도 계획 출력의 속도를 대응시킵니다")
    parser.add_argument("--state-offset-ms", type=float, default=0.0,
                        help="차량 자세·속도·가속도 조회 시각을 경로 시각에서 이동합니다")
    parser.add_argument("--object-offset-ms", type=float, default=0.0,
                        help="진단용으로 예측 객체 조회 시각을 경로 시각에서 이동합니다")
    parser.add_argument("--assert-hpc-runout-reference", action="store_true",
                        help="HPC 돌발진입 기록의 알려진 충돌 후보 집계를 검사합니다")
    args = parser.parse_args()
    if args.ego_speed_scale <= 0.0:
        parser.error("차량 속도 배율은 양수여야 합니다")
    if args.smooth_path:
        args.prepare_path = True
    if args.analytical_forward:
        args.prepare_path = True
    if args.analytical_full:
        args.prepare_path = True
    objects = list(messages(args.bag, OBJECTS))
    odometry = list(messages(args.bag, ODOMETRY))
    accelerations = list(messages(args.bag, ACCELERATION)) if args.smooth_path else []
    object_times = [time for time, _ in objects]
    odometry_times = [time for time, _ in odometry]
    acceleration_times = [time for time, _ in accelerations]
    output_paths = list(messages(args.bag, OUTPUT_PATH)) if (
        args.use_recorded_output_velocity) else []
    output_times = [time for time, _ in output_paths]
    velocity_replaced = 0
    velocity_matched_frames = 0
    frames = []
    for time, payload in messages(args.bag, PATH):
        if len(frames) >= args.frames:
            break
        path = decode_path(payload, with_lane_ids=True)["points"]
        if output_paths and path:
            output_index = bisect.bisect_left(output_times, time)
            if (output_index < len(output_paths) and
                    output_times[output_index] - time <= 50_000_000):
                output_points = decode_path(output_paths[output_index][1])["points"]
                if output_points:
                    velocity_matched_frames += 1
                    replaced_path = []
                    for point in path:
                        nearest = min(output_points, key=lambda candidate:
                                      math.hypot(candidate[0] - point[0],
                                                 candidate[1] - point[1]))
                        separation = math.hypot(nearest[0] - point[0],
                                                nearest[1] - point[1])
                        if separation <= 0.2:
                            velocity_replaced += abs(nearest[7] - point[7]) > 1.0e-5
                            replaced_path.append(
                                point[:7] + (nearest[7],) + point[8:])
                        else:
                            replaced_path.append(point)
                    path = replaced_path
        object_time = time + int(args.object_offset_ms * 1_000_000)
        oi = bisect.bisect_right(object_times, object_time) - 1
        state_time = time + int(args.state_offset_ms * 1_000_000)
        vi = bisect.bisect_right(odometry_times, state_time) - 1
        if oi < 0 or vi < 0 or len(path) < 2:
            continue
        ego = decode_odometry(odometry[vi][1])
        ai = bisect.bisect_right(acceleration_times, state_time) - 1
        ego_acceleration = (decode_acceleration(accelerations[ai][1])[
            "acceleration_mps2"] if ai >= 0 else 0.0)
        nearest = min(range(len(path)), key=lambda i:
                      (path[i][0] - ego["x"]) ** 2 +
                      (path[i][1] - ego["y"]) ** 2)
        if nearest + 1 >= len(path):
            continue
        observed = decode_predicted_objects(objects[oi][1])["objects"]
        candidates = []
        for item in observed:
            label = (max(item["classification"], key=lambda entry: entry[1])[0]
                     if item["classification"] else 0)
            if label not in (5, 6, 7):
                continue
            velocities = range_from_covariance(item)
            if velocities is None:
                continue
            candidates.append((item, label, velocities))
        selected_path = path if args.prepare_path else path[nearest:]
        frames.append((time * 1e-9, ego, selected_path, candidates,
                       ego_acceleration))
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory) / "입력.txt"
        target = Path(directory) / "출력.txt"
        map_record = next(messages(args.bag, "/map/vector_map"), None)
        if map_record is None:
            raise RuntimeError("기록에 이진 차로 지도가 없습니다")
        map_path = Path(directory) / "지도.bin"
        map_path.write_bytes(decode_lanelet_map(map_record[1]))
        with source.open("w") as stream:
            stream.write(f"{len(frames)}\n")
            for now, ego, path, candidates, ego_acceleration in frames:
                ego_pose = (0, 0, 0, ego["orientation_x"],
                            ego["orientation_y"], ego["orientation_z"],
                            ego["orientation_w"])
                stream.write(f"{len(path)} {len(candidates)} "
                             f"{now:.9f} {ego['x']} {ego['y']} "
                             f"{yaw(ego_pose)} "
                             f"{ego['velocity_mps'] * args.ego_speed_scale} "
                             f"{ego_acceleration}\n")
                for point in path:
                    stream.write(f"{point[0]} {point[1]} {yaw(point[:7])} "
                                 f"{point[7]}\n")
                for item, label, velocities in candidates:
                    pose = item["pose"]
                    paths = item["predicted_paths"]
                    stream.write(f"{item['uuid']} {label} {item['shape_type']} "
                                 f"{pose[0]} {pose[1]} {yaw(pose)} "
                                 f"{item['dimensions'][0]} "
                                 f"{item['dimensions'][1]} "
                                 f"{velocities[0]} {velocities[1]} "
                                 f"{len(paths)}\n")
                    for prediction in paths:
                        stream.write(f"{prediction['confidence']} "
                                     f"{prediction['time_step_s']} "
                                     f"{len(prediction['poses'])}\n")
                        for point in prediction["poses"]:
                            stream.write(f"{point[0]} {point[1]} "
                                         f"{yaw(point)}\n")
        command = [str(args.runner), str(source), str(target), str(map_path)]
        if args.prepare_path:
            command.append("--prepare-path")
        if args.smooth_path:
            command.append("--smooth-path")
        if args.constant_ego_speed:
            command.append("--constant-ego-speed")
        if args.analytical_forward:
            command.append("--analytical-forward")
        if args.analytical_full:
            command.append("--analytical-full")
        subprocess.run(command, check=True)
        rows = [line.split() for line in target.read_text().splitlines()]
    active = [row for row in rows if row[1] != "-"]
    debug = [(time, decode_float32_multi_array_stamped(payload)["data"])
             for time, payload in messages(args.bag, DEBUG)]
    debug_times = [time for time, _ in debug]
    matched = reference_active = joint = 0
    independent_on_matched = 0
    accepted_on_matched = accepted_joint = 0
    distance_errors = []
    differences = []
    for frame, row in zip(frames, rows):
        index = bisect.bisect_left(debug_times, int(frame[0] * 1e9))
        if index >= len(debug) or debug_times[index] - int(frame[0] * 1e9) > 50_000_000:
            continue
        matched += 1
        original = len(debug[index][1]) > 7 and debug[index][1][7] < 49.0
        independent_on_matched += row[1] != "-"
        accepted = len(row) > 6 and row[6] == "1"
        accepted_on_matched += accepted
        accepted_joint += original and accepted
        reference_active += original
        joint += original and row[1] != "-"
        if args.diagnose and original != (row[1] != "-"):
            differences.append((int(row[0]), original, row[1][:8],
                                round(float(row[2]), 3),
                                round(debug[index][1][7], 3),
                                int(row[5]), row[7][:8]))
        if original and row[1] != "-":
            _, ego, path, _, _ = frame
            if args.prepare_path:
                nearest = min(range(len(path)), key=lambda i:
                              (path[i][0] - ego["x"]) ** 2 +
                              (path[i][1] - ego["y"]) ** 2)
                path = path[nearest:]
            distance_errors.append(abs(
                float(row[2]) - ego_arc(path, ego["x"], ego["y"]) -
                3.79 - debug[index][1][7]))
    print(f"{args.bag}: 재생 경로 {len(frames)}, 대상 객체 "
          f"{sum(len(frame[3]) for frame in frames)}, "
          f"필터 후 객체 전달 {sum(int(row[5]) for row in rows)}, "
          f"독립 충돌 후보 {len(active)}프레임, "
          f"원본 디버그 대응 {matched}프레임 중 충돌 {reference_active}, "
          f"함께 활성 {joint}, 독립만 활성 "
          f"{independent_on_matched - joint}, 원본만 활성 "
          f"{reference_active - joint}")
    if args.use_recorded_output_velocity:
        print(f"  출력 속도 대응 경로 {velocity_matched_frames}, "
              f"실제로 바뀐 입력점 {velocity_replaced}")
    print(f"  0.15초 순간 검출 필터 뒤: 독립 활성 {accepted_on_matched}, "
          f"동시 활성 {accepted_joint}, 독립만 활성 "
          f"{accepted_on_matched - accepted_joint}, 원본만 활성 "
          f"{reference_active - accepted_joint}")
    if distance_errors:
        print(f"  전면 기준 충돌 거리 차이 {len(distance_errors)}프레임: "
              f"중앙값 {statistics.median(distance_errors):.3f}m, "
              f"최대 {max(distance_errors):.3f}m")
    if args.assert_hpc_runout_reference:
        observed = (len(frames), matched, reference_active, len(active),
                    joint, independent_on_matched - joint,
                    reference_active - joint, len(distance_errors))
        expected = (517, 491, 17, 26, 17, 9, 0, 17)
        if not args.analytical_full or observed != expected or (
                max(distance_errors, default=math.inf) >= 0.65):
            raise AssertionError(
                f"HPC 돌발진입 후보 집계 불일치: {observed}, "
                f"기대 {expected}, 최대 거리 차이 "
                f"{max(distance_errors, default=math.inf):.3f}m")
    if differences:
        print("  활성 불일치 첫 사례", differences[:30])
    if args.compare_travel_time_markers:
        markers = list(messages(args.bag, MARKER))
        marker_times = [time for time, _ in markers]
        time_errors = []
        for frame, row in zip(frames, rows):
            if row[1] == "-":
                continue
            time = int(frame[0] * 1e9)
            di = bisect.bisect_left(debug_times, time)
            mi = bisect.bisect_left(marker_times, time)
            if di >= len(debug) or mi >= len(markers) or (
                    debug_times[di] - time > 50_000_000 or
                    marker_times[mi] - time > 50_000_000):
                continue
            if len(debug[di][1]) <= 7 or debug[di][1][7] >= 49.0:
                continue
            marker_array = decode_marker_array(markers[mi][1])
            texts = [marker["text"] for marker in marker_array
                     if marker["namespace"] == "travel_time_texts"]
            if texts and texts[-1].endswith("s"):
                time_errors.append(abs(float(row[3]) - float(texts[-1][:-1])))
        if time_errors:
            print(f"  원본 마지막 검사점 예측 시간 차이 {len(time_errors)}프레임: "
                  f"중앙값 {statistics.median(time_errors):.3f}초, "
                  f"최대 {max(time_errors):.3f}초")
    if active and args.diagnose:
        arcs = [float(row[2]) for row in active]
        print(f"  충돌점 경로 호길이 중앙값 "
              f"{statistics.median(arcs):.3f}m, 첫 사례 {active[:5]}")
        for row in active[:5]:
            now, _, path, candidates, _ = frames[int(row[0])]
            index = bisect.bisect_left(debug_times, int(now * 1e9))
            if index < len(debug):
                print(f"  원본 디버그 {row[0]}: "
                      f"{debug[index][1][5:12]}")
            selected = next((item for item, _, _ in candidates
                             if item["uuid"] == row[1]), None)
            if selected:
                pose = selected["pose"]
                nearest = min(path, key=lambda point:
                              (point[0] - pose[0]) ** 2 +
                              (point[1] - pose[1]) ** 2)
                angle = yaw(nearest[:7])
                lateral = (-math.sin(angle) * (pose[0] - nearest[0]) +
                           math.cos(angle) * (pose[1] - nearest[1]))
                print(f"  객체 {row[1][:8]}의 최초 횡거리 "
                      f"{lateral:.3f}m, 외형 "
                      f"{selected['dimensions'][:2]}")
    if args.trace_frames:
        markers = list(messages(args.bag, MARKER))
        marker_times = [time for time, _ in markers]
        for value in args.trace_frames.split(","):
            index = int(value)
            if index < 0 or index >= len(frames):
                continue
            time = int(frames[index][0] * 1e9)
            marker_index = bisect.bisect_left(marker_times, time)
            original_time = "-"
            if marker_index < len(markers) and marker_times[marker_index] - time < 50_000_000:
                marker_array = decode_marker_array(markers[marker_index][1])
                texts = [marker["text"] for marker in marker_array
                         if marker["namespace"] == "travel_time_texts"]
                if texts:
                    original_time = texts[-1]
            print(f"  예측 시간 프레임 {index}: 독립 {rows[index][3]}초, "
                  f"원본 마지막 검사점 {original_time}")


if __name__ == "__main__":
    main()
