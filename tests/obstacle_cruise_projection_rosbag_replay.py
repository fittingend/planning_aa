"""K-City 기록의 예측 객체를 독립 순항 후보 앞단에 재생합니다."""

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
    decode_acceleration, decode_float32_multi_array_stamped, decode_marker_array,
    decode_odometry,
    decode_predicted_objects, decode_trajectory, messages)
from obstacle_cruise_stop_rosbag_replay import inserted_index, stop_arc

PREFIX = "/planning/scenario_planning/lane_driving/motion_planning/"
TRAJECTORY = PREFIX + "motion_velocity_planner/trajectory"
OBJECTS = "/perception/object_recognition/objects"
ODOMETRY = "/localization/kinematic_state"
ACCELERATION = "/localization/acceleration"
CRUISE_DEBUG = PREFIX + "obstacle_cruise_planner/debug/cruise_planning_info"
STOP_DEBUG = PREFIX + "obstacle_cruise_planner/debug/stop_planning_info"
SLOWDOWN_DEBUG = PREFIX + "obstacle_cruise_planner/debug/slow_down_planning_info"
STOP_MARKER = PREFIX + "obstacle_cruise_planner/debug/marker"
FINAL_TRAJECTORY = "/planning/scenario_planning/lane_driving/trajectory"


def yaw(pose):
    qx, qy, qz, qw = pose[3:7]
    return math.atan2(2 * (qw * qz + qx * qy),
                      1 - 2 * (qy * qy + qz * qz))


def radius(item):
    kind = item["shape_type"]
    length, width, _ = item["dimensions"]
    if kind == 0:
        return math.hypot(length / 2, width / 2)
    if kind == 1:
        return length / 2
    return max((math.hypot(x, y) for x, y, _ in item["footprint"]),
               default=0.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frames", type=int, default=10000)
    parser.add_argument("--start-frame", type=int, default=0,
                        help="초기 입력이 준비된 뒤 이 수만큼 건너뛰고 상태를 새로 시작합니다")
    parser.add_argument("--stop-runner", type=Path,
                        help="같은 실제 객체로 정지 후보 다각형도 계산합니다")
    parser.add_argument("--stop-vehicle-shape", choices=("obigo", "sample"),
                        default="obigo",
                        help="정지·감속 후보의 차량 외형 설정을 선택합니다")
    parser.add_argument("--diagnose", action="store_true",
                        help="원본 디버그와 후보 활성 판단 차이의 처음 세 사례를 보입니다")
    parser.add_argument("--state-offset-ms", type=float, default=0.0,
                        help="차량 자세·속도·가속도 조회 시각을 옮깁니다")
    parser.add_argument("--object-offset-ms", type=float, default=0.0,
                        help="예측 객체 조회 시각을 옮겨 콜백 경계를 진단합니다")
    parser.add_argument("--marker-guided-object-pairing", action="store_true",
                        help="원본 정지 표식 위치로 객체 콜백을 역선택하는 진단입니다")
    args = parser.parse_args()
    objects = list(messages(args.bag, OBJECTS))
    odometry = list(messages(args.bag, ODOMETRY))
    accelerations = list(messages(args.bag, ACCELERATION))
    object_times = [time for time, _ in objects]
    markers = ([(time, [marker for marker in decode_marker_array(payload)
                       if marker["namespace"] == "obstacles_to_stop" and
                       marker["action"] == 0])
                for time, payload in messages(args.bag, STOP_MARKER)]
               if args.marker_guided_object_pairing else [])
    marker_times = [time for time, _ in markers]
    guided_total = guided_future = 0
    odometry_times = [time for time, _ in odometry]
    acceleration_times = [time for time, _ in accelerations]
    slowdown_debug = ([(time, decode_float32_multi_array_stamped(payload)["data"])
                       for time, payload in messages(args.bag, SLOWDOWN_DEBUG)]
                      if args.stop_runner else [])
    slowdown_times = [time for time, _ in slowdown_debug]
    frames = []
    valid_seen = 0
    for time, payload in messages(args.bag, TRAJECTORY):
        if len(frames) >= args.frames:
            break
        trajectory = decode_trajectory(payload)
        points = trajectory["points"]
        if len(points) < 2:
            continue
        object_time = time + int(args.object_offset_ms * 1_000_000)
        oi = bisect.bisect_right(object_times, object_time) - 1
        if markers:
            mi = bisect.bisect_left(marker_times, time)
            if (mi < len(markers) and
                    marker_times[mi] - time <= 50_000_000 and
                    markers[mi][1]):
                pose = markers[mi][1][0]["pose"]
                lower = bisect.bisect_left(object_times, time - 20_000_000)
                upper = bisect.bisect_right(object_times, time + 30_000_000)
                matches = []
                for candidate_index in range(lower, upper):
                    candidate = decode_predicted_objects(
                        objects[candidate_index][1])
                    for obj in candidate["objects"]:
                        distance = math.hypot(obj["pose"][0] - pose[0],
                                              obj["pose"][1] - pose[1])
                        matches.append((distance, candidate_index))
                if matches:
                    distance, selected = min(matches)
                    if distance < 0.1:
                        oi = selected
                        guided_total += 1
                        guided_future += object_times[oi] > time
        state_time = time + int(args.state_offset_ms * 1_000_000)
        vi = bisect.bisect_right(odometry_times, state_time) - 1
        ai = bisect.bisect_right(acceleration_times, state_time) - 1
        if oi < 0 or vi < 0:
            continue
        if valid_seen < args.start_frame:
            valid_seen += 1
            continue
        predicted = decode_predicted_objects(objects[oi][1])
        observed = predicted["objects"]
        ego = decode_odometry(odometry[vi][1])
        ego_acceleration = (decode_acceleration(accelerations[ai][1])[
            "acceleration_mps2"] if ai >= 0 else 0.0)
        nearest = min(range(len(points)), key=lambda i: (
            points[i]["x"] - ego["x"]) ** 2 +
            (points[i]["y"] - ego["y"]) ** 2)
        cruise_objects = observed
        elapsed = (time - predicted["stamp_ns"]) * 1e-9
        ego_yaw = yaw((0, 0, 0, ego["orientation_x"],
                       ego["orientation_y"], ego["orientation_z"],
                       ego["orientation_w"]))
        frames.append((points, nearest, observed, cruise_objects, elapsed,
                       ego["velocity_mps"], time, ego["x"], ego["y"],
                       ego_yaw, trajectory["stamp_ns"], ego_acceleration))
        valid_seen += 1
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory) / "입력.txt"
        target = Path(directory) / "출력.txt"
        with source.open("w") as stream:
            stream.write(f"{len(frames)}\n")
            for (points, nearest, _, cruise_objects, elapsed, ego_speed,
                 frame_time, ego_x, ego_y, ego_yaw, _, ego_acceleration) in frames:
                stream.write(f"{len(points)} {len(cruise_objects)} "
                             f"{nearest} {ego_speed} {ego_x} {ego_y} "
                             f"{ego_yaw} {frame_time * 1e-9:.9f} "
                             f"{ego_acceleration}\n")
                for point in points:
                    stream.write(" ".join(str(point[name]) for name in (
                        "x", "y", "orientation_x", "orientation_y",
                        "orientation_z", "orientation_w", "velocity")) + "\n")
                for item in cruise_objects:
                    pose = item["pose"]
                    twist = item["twist"]
                    stream.write(f"{item['uuid']} {pose[0]} {pose[1]} {yaw(pose)} "
                                 f"{twist[0]} {twist[1]} {radius(item)} "
                                 f"{elapsed} {len(item['predicted_paths'])} "
                                 f"{item['shape_type']} "
                                 f"{item['dimensions'][0]} "
                                 f"{item['dimensions'][1]} "
                                 f"{len(item['footprint'])} "
                                 f"{item['classification'][0][0] if item['classification'] else 0}\n")
                    for vertex in item["footprint"]:
                        stream.write(f"{vertex[0]} {vertex[1]}\n")
                    for path in item["predicted_paths"]:
                        stream.write(f"{path['time_step_s']} "
                                     f"{path['confidence']} {len(path['poses'])}\n")
                        for predicted_pose in path["poses"]:
                            stream.write(f"{predicted_pose[0]} "
                                         f"{predicted_pose[1]} "
                                         f"{yaw(predicted_pose)}\n")
        ids_target = Path(directory) / "순항객체.txt"
        runner_command = [str(args.runner), str(source), str(target)]
        if args.stop_runner:
            runner_command.extend(("--write-ids", str(ids_target)))
            if args.stop_vehicle_shape == "sample":
                runner_command.append("--sample-vehicle")
        subprocess.run(runner_command, check=True)
        id_lines = ids_target.read_text().splitlines() if args.stop_runner else []
        rows = [[int(number) for number in line.split()]
                for line in target.read_text().splitlines()]
        stop_rows = None
        if args.stop_runner:
            stop_target = Path(directory) / "정지출력.txt"
            slowdown_activity = Path(directory) / "원본감속활성.txt"
            with slowdown_activity.open("w") as stream:
                for frame in frames:
                    index = bisect.bisect_left(slowdown_times, frame[6])
                    active = (index < len(slowdown_debug) and
                              slowdown_times[index] - frame[6] <= 50_000_000 and
                              len(slowdown_debug[index][1]) >= 7)
                    stream.write(f"{int(active)}\n")
            stop_command = [str(args.stop_runner), str(source),
                            str(stop_target), str(ids_target)]
            if args.stop_vehicle_shape == "sample":
                stop_command.extend(("3.79", "1.10", "1.896"))
            stop_command.append(str(slowdown_activity))
            subprocess.run(stop_command, check=True)
            stop_rows = [[float(number) for number in line.split()]
                         for line in stop_target.read_text().splitlines()]
    assert len(rows) == len(frames)
    if args.marker_guided_object_pairing:
        print(f"원본 정지 표식 역선택: {guided_total}프레임, "
              f"궤적 기록 뒤 객체 메시지 {guided_future}프레임 "
              "(출력 정보를 쓴 진단이며 실제 입력 복원으로 간주하지 않음)")
    if stop_rows is not None:
        assert len(stop_rows) == len(frames)
    totals = [sum(row[i] for row in rows) for i in range(8)]
    active = [sum(row[i] > 0 for row in rows) for i in range(1, 8)]
    observed_total = sum(len(frame[2]) for frame in frames)
    debug = [(time, decode_float32_multi_array_stamped(payload)["data"])
             for time, payload in messages(args.bag, CRUISE_DEBUG)]
    debug_times = [time for time, _ in debug]
    matched = active_reference = accepted_reference = same_active = 0
    false_inside = false_outside = 0
    strict_reference = strict_same_active = 0
    strict_false_inside = strict_false_outside = 0
    yield_matched = yield_original_active = 0
    yield_examples = []
    examples = {"원본만 활성": [], "간이 후보만 활성": []}
    for frame_index, (frame, row) in enumerate(zip(frames, rows)):
        index = bisect.bisect_left(debug_times, frame[6])
        if index >= len(debug) or debug_times[index] - frame[6] > 50_000_000:
            continue
        matched += 1
        original = len(debug[index][1]) >= 4 and debug[index][1][3] != 0.0
        if row[11] > 0:
            yield_matched += 1
            yield_original_active += original
            if len(yield_examples) < 3:
                yield_examples.append((frame_index, original))
        independent = row[5] > 0
        strict = row[8] > 0
        active_reference += original
        accepted_reference += independent
        same_active += original and independent
        strict_reference += strict
        strict_same_active += original and strict
        if independent and not original:
            false_inside += row[6] > 0
            false_outside += row[7] > 0
        if strict and not original:
            strict_false_inside += row[9] > 0
            strict_false_outside += row[10] > 0
        if original != independent:
            key = "원본만 활성" if original else "간이 후보만 활성"
            if len(examples[key]) < 3:
                examples[key].append((frame_index, row, debug[index][1][3]))
    print(f"{args.bag}: 프레임 {len(frames)}, 전체 객체 {observed_total}, "
          f"순항 종류 {totals[0]}, "
          f"전방 {totals[1]}({active[0]}프레임), "
          f"넓은 횡거리 통과 {totals[2]}({active[1]}프레임), "
          f"비음수 종속도 {totals[3]}({active[2]}프레임), "
          f"예측 외형 충돌 {totals[4]}({active[3]}프레임), "
          f"간이 순항 후보 {totals[5]}({active[4]}프레임, "
          f"내부 {totals[6]}, 외부 {totals[7]}), "
          f"원본 디버그 대응 {matched}프레임 중 활성 {active_reference}, "
          f"간이 후보 활성 {accepted_reference}, 함께 활성 {same_active}, "
          f"간이만 활성 중 내부 {false_inside}, 외부 {false_outside}")
    print(f"  별도 배포 설정 후보 활성 {strict_reference}, "
          f"원본과 동시 활성 {strict_same_active}, "
          f"원본만 활성 {active_reference - strict_same_active}, "
          f"간이만 활성 {strict_reference - strict_same_active}, "
          f"간이만 활성 중 내부 {strict_false_inside}, "
          f"외부 {strict_false_outside}")
    print(f"  양보 순항 객체 쌍 {sum(row[11] for row in rows)}, "
          f"해당 프레임 {sum(row[11] > 0 for row in rows)}, "
          f"원본 디버그 대응 {yield_matched}프레임 중 활성 "
          f"{yield_original_active}")
    if args.diagnose:
        for key, values in examples.items():
            print(key, values)
        print("양보 후보 첫 사례", yield_examples)
    if stop_rows is not None:
        stop_debug = [(time, decode_float32_multi_array_stamped(payload)["data"])
                      for time, payload in messages(args.bag, STOP_DEBUG)]
        stop_times = [time for time, _ in stop_debug]
        stop_matched = stop_active = stop_candidate_active = stop_joint = 0
        distance_errors = []
        strict_distance_errors = []
        for frame_index, (frame, row) in enumerate(zip(frames, stop_rows)):
            index = bisect.bisect_left(stop_times, frame[6])
            if index >= len(stop_debug) or stop_times[index] - frame[6] > 50_000_000:
                continue
            stop_matched += 1
            original = (len(stop_debug[index][1]) >= 4 and
                        stop_debug[index][1][3] > 0.0)
            independent = row[3] > 0
            stop_active += original
            stop_candidate_active += independent
            stop_joint += original and independent
            if original and independent and row[5] >= 0:
                distance_errors.append(abs(stop_debug[index][1][3] - row[5]))
            if original and row[9] > 0 and row[10] >= 0:
                strict_distance_errors.append(
                    abs(stop_debug[index][1][3] - row[10]))
        print(f"  정지 앞단: 현재 외형 충돌 "
              f"{sum(row[2] for row in stop_rows):.0f}, "
              f"간이 후보 {sum(row[3] for row in stop_rows):.0f} "
              f"({sum(row[3] > 0 for row in stop_rows)}프레임, "
              f"보행자 {sum(row[4] for row in stop_rows):.0f}), "
              f"원본 디버그 대응 {stop_matched}프레임 중 활성 "
              f"{stop_active}, 간이 후보 활성 {stop_candidate_active}, "
              f"함께 활성 {stop_joint}")
        for name, column in (("샘플 설정", 8), ("별도 배포 설정", 9)):
            priority_active = priority_joint = 0
            for frame, row in zip(frames, stop_rows):
                index = bisect.bisect_left(stop_times, frame[6])
                if (index >= len(stop_debug) or
                        stop_times[index] - frame[6] > 50_000_000):
                    continue
                original = (len(stop_debug[index][1]) >= 4 and
                            stop_debug[index][1][3] > 0.0)
                priority_active += row[column] > 0
                priority_joint += original and row[column] > 0
            print(f"  순항 객체 우선 제외({name}): 정지 후보 "
                  f"{sum(row[column] for row in stop_rows):.0f}, "
                  f"대응 활성 {priority_active}, 동시 활성 {priority_joint}")
        held_active = held_joint = 0
        original_only_transient = 0
        margin_matched = margin_equal = 0
        margin_pairs = {}
        for frame, row in zip(frames, stop_rows):
            index = bisect.bisect_left(stop_times, frame[6])
            if (index >= len(stop_debug) or
                    stop_times[index] - frame[6] > 50_000_000):
                continue
            original = (len(stop_debug[index][1]) >= 4 and
                        stop_debug[index][1][3] > 0.0)
            held_active += row[13] > 0
            held_joint += original and row[13] > 0
            if original and row[13] == 0 and len(row) > 26:
                original_only_transient += row[26] > 0
            if original and row[13] > 0 and len(stop_debug[index][1]) > 5:
                margin_matched += 1
                expected_margin = stop_debug[index][1][5]
                margin_equal += abs(expected_margin - row[15]) < 1e-3
                key = (round(expected_margin, 2), round(row[15], 2))
                margin_pairs[key] = margin_pairs.get(key, 0) + 1
        print(f"  이전 정지 객체 유지 뒤: 대응 활성 {held_active}, "
              f"원본과 동시 활성 {held_joint}, "
              f"원본만 활성 {stop_active - held_joint}, "
              f"독립만 활성 {held_active - held_joint}")
        if len(stop_rows[0]) > 26:
            print(f"  원본만 정지 중 독립 미래 통과 판정 "
                  f"{original_only_transient}프레임")
        if margin_matched:
            print(f"  정지 여유 거리: 동시 활성 {margin_matched}프레임 중 "
                  f"일치 {margin_equal}, 원본/독립 분포 {margin_pairs}")
        final_paths = {message["stamp_ns"]: message["points"]
                       for _, payload in messages(args.bag, FINAL_TRAJECTORY)
                       if (message := decode_trajectory(payload))}
        inserted_errors = []
        planned_inserted_errors = []
        planned_disagreements = []
        for frame, row in zip(frames, stop_rows):
            debug_index = bisect.bisect_left(stop_times, frame[6])
            if (debug_index >= len(stop_debug) or
                    stop_times[debug_index] - frame[6] > 50_000_000 or
                    len(stop_debug[debug_index][1]) < 4 or
                    stop_debug[debug_index][1][3] <= 0.0):
                continue
            target = final_paths.get(frame[10])
            source = frame[0]
            if (row[13] <= 0 or target is None or
                    len(target) != len(source) + 1):
                continue
            index = inserted_index(source, target)
            if index >= len(target) or abs(target[index]["velocity"]) > 1e-4:
                continue
            offset_error, actual_arc = stop_arc(source, target[index])
            if offset_error > 0.05:
                continue
            inserted_errors.append(abs(actual_arc - row[16]))
            if len(row) > 21 and row[21] >= 0.0:
                planned_inserted_errors.append(abs(actual_arc - row[21]))
                if abs(actual_arc - row[21]) > 0.1:
                    planned_disagreements.append((round(actual_arc, 3),
                        round(row[16], 3), round(row[21], 3),
                        round(frame[5], 3)))
        if inserted_errors:
            print(f"  최종 경로 단일 정지점 삽입: 비교 "
                  f"{len(inserted_errors)}프레임, 정지 호길이 차이 "
                  f"중앙값 {statistics.median(inserted_errors):.3f}m, "
                  f"최대 {max(inserted_errors):.3f}m, "
                  f"0.1m 이내 {sum(error <= 0.1 for error in inserted_errors)}")
        if planned_inserted_errors:
            print(f"  정지 코어 삽입 호길이: 비교 "
                  f"{len(planned_inserted_errors)}프레임, 차이 "
                  f"중앙값 {statistics.median(planned_inserted_errors):.3f}m, "
                  f"최대 {max(planned_inserted_errors):.3f}m, "
                  f"0.1m 이내 "
                  f"{sum(error <= 0.1 for error in planned_inserted_errors)}")
            if args.diagnose:
                print("  정지 코어 차이 첫 사례", planned_disagreements[:8])
        if args.diagnose:
            missing = []
            for frame_index, (frame, row) in enumerate(zip(frames, stop_rows)):
                index = bisect.bisect_left(stop_times, frame[6])
                if (index >= len(stop_debug) or
                        stop_times[index] - frame[6] > 50_000_000):
                    continue
                original = (len(stop_debug[index][1]) >= 4 and
                            stop_debug[index][1][3] > 0.0)
                if original and row[13] == 0:
                    missing.append((frame_index, stop_debug[index][1][3],
                                    row[2], row[3], row[9],
                                    [(item['uuid'][:8],
                                      item['classification'][0][0]
                                      if item['classification'] else 0,
                                      round(item['twist'][0], 2))
                                     for item in frame[2][:3]]))
            print("  원본 정지만 활성인 첫 사례", missing[:8])
        if distance_errors:
            print(f"  정지 충돌 거리 차이: {len(distance_errors)}프레임, "
                  f"중앙값 {statistics.median(distance_errors):.3f}m, "
                  f"최대 {max(distance_errors):.3f}m")
        if strict_distance_errors:
            print(f"  우선순위 적용 충돌 거리 차이: "
                  f"{len(strict_distance_errors)}프레임, "
                  f"중앙값 {statistics.median(strict_distance_errors):.3f}m, "
                  f"최대 {max(strict_distance_errors):.3f}m")
        slow_matched = slow_original = slow_candidate = slow_joint = 0
        slow_priority = slow_priority_joint = 0
        slow_current_priority = slow_current_priority_joint = 0
        slow_planned = slow_planned_joint = 0
        slow_velocity_errors = []
        slow_stateless_errors = []
        slow_lateral_errors = []
        slow_start_errors = []
        slow_end_errors = []
        slow_oracle_errors = []
        slow_missing = []
        for frame_index, (frame, row) in enumerate(zip(frames, stop_rows)):
            index = bisect.bisect_left(slowdown_times, frame[6])
            if (index >= len(slowdown_debug) or
                    slowdown_times[index] - frame[6] > 50_000_000):
                continue
            slow_matched += 1
            original = len(slowdown_debug[index][1]) >= 7
            independent = row[7] > 0
            slow_original += original
            slow_candidate += independent
            slow_joint += original and independent
            slow_priority += row[11] > 0
            slow_priority_joint += original and row[11] > 0
            slow_current_priority += row[12] > 0
            slow_current_priority_joint += original and row[12] > 0
            slow_planned += row[17] > 0
            slow_planned_joint += original and row[17] > 0
            if args.diagnose and original and row[17] == 0 and len(slow_missing) < 5:
                ids = id_lines[frame_index].split()
                baseline_count = int(ids[0])
                strict_count_index = 1 + baseline_count
                strict_count = int(ids[strict_count_index])
                strict_ids = ids[strict_count_index + 1:
                                 strict_count_index + 1 + strict_count]
                cruise_index = bisect.bisect_left(debug_times, frame[6])
                original_cruise = (cruise_index < len(debug) and
                                   debug_times[cruise_index] - frame[6] <=
                                   50_000_000 and
                                   len(debug[cruise_index][1]) >= 4 and
                                   debug[cruise_index][1][3] != 0.0)
                first_id = strict_ids[0] if strict_ids else ""
                observed_object = next((item for item in frame[2]
                                        if item["uuid"] == first_id), None)
                object_speed = (math.hypot(*observed_object["twist"][:2])
                                if observed_object else None)
                slow_missing.append((frame_index,
                                     slowdown_debug[index][1][:5], row[6:20],
                                     strict_ids, original_cruise,
                                     object_speed))
            if original and row[17] == 1 and len(slowdown_debug[index][1]) >= 5:
                slow_velocity_errors.append(abs(
                    slowdown_debug[index][1][4] - row[18]))
                if row[19] >= 0:
                    slow_lateral_errors.append(abs(
                        slowdown_debug[index][1][0] - row[19]))
                if row[22] >= 0:
                    slow_start_errors.append(abs(
                        slowdown_debug[index][1][1] - row[22]))
                    slow_end_errors.append(abs(
                        slowdown_debug[index][1][2] - row[23]))
                if row[20] >= 0:
                    slow_stateless_errors.append(abs(
                        slowdown_debug[index][1][3] - row[20]))
                if row[24] == 1:
                    slow_oracle_errors.append(abs(
                        slowdown_debug[index][1][4] - row[25]))
        print(f"  감속 앞단: 현재 외형 교차 {sum(row[6] for row in stop_rows):.0f}, "
              f"5프레임 상태를 거친 후보 {sum(row[7] for row in stop_rows):.0f} "
              f"({sum(row[7] > 0 for row in stop_rows)}프레임), "
              f"원본 디버그 대응 {slow_matched}프레임 중 활성 "
              f"{slow_original}, 간이 후보 활성 {slow_candidate}, "
              f"함께 활성 {slow_joint}; 현재 프레임 우선 제외 뒤 "
              f"{sum(row[12] for row in stop_rows):.0f}건, "
              f"대응 활성 {slow_current_priority}, "
              f"함께 활성 {slow_current_priority_joint}; "
              f"연속 상태도 다시 계산하면 {sum(row[11] for row in stop_rows):.0f}건, "
              f"대응 활성 {slow_priority}, 함께 활성 {slow_priority_joint}")
        print(f"  감속 속도 계획: 대응 활성 {slow_planned}, "
              f"원본과 동시 활성 {slow_planned_joint}, "
              f"원본만 활성 {slow_original - slow_planned_joint}")
        if slow_velocity_errors:
            print(f"  단일 감속 출력 속도 차이: "
                  f"{len(slow_velocity_errors)}프레임, "
                  f"중앙값 {statistics.median(slow_velocity_errors):.3f}m/s, "
                  f"최대 {max(slow_velocity_errors):.3f}m/s")
        if slow_lateral_errors:
            print(f"  감속 객체 횡거리 차이: "
                  f"{len(slow_lateral_errors)}프레임, "
                  f"중앙값 {statistics.median(slow_lateral_errors):.3f}m, "
                  f"최대 {max(slow_lateral_errors):.3f}m")
        if slow_start_errors:
            print(f"  감속 구간 호길이 차이: {len(slow_start_errors)}프레임, "
                  f"시작 중앙값 {statistics.median(slow_start_errors):.3f}m·"
                  f"최대 {max(slow_start_errors):.3f}m, "
                  f"종료 중앙값 {statistics.median(slow_end_errors):.3f}m·"
                  f"최대 {max(slow_end_errors):.3f}m")
        if slow_stateless_errors:
            print(f"  상태 초기화 감속 가능 속도 차이: "
                  f"{len(slow_stateless_errors)}프레임, "
                  f"중앙값 {statistics.median(slow_stateless_errors):.3f}m/s, "
                  f"최대 {max(slow_stateless_errors):.3f}m/s")
        if slow_oracle_errors:
            print(f"  원본 활성 프레임만 상태 갱신한 안정화 속도 차이: "
                  f"{len(slow_oracle_errors)}프레임, "
                  f"중앙값 {statistics.median(slow_oracle_errors):.6f}m/s, "
                  f"최대 {max(slow_oracle_errors):.6f}m/s")
        if slow_missing:
            print("  원본만 활성인 감속 첫 사례", slow_missing)
        if args.diagnose:
            comparisons = []
            extra = []
            for frame_index, (frame, row) in enumerate(zip(frames, stop_rows)):
                index = bisect.bisect_left(slowdown_times, frame[6])
                if (index >= len(slowdown_debug) or
                        slowdown_times[index] - frame[6] > 50_000_000):
                    continue
                if row[17] > 0 and len(slowdown_debug[index][1]) < 7:
                    extra.append((frame_index, row[12], row[17:24],
                                  row[9], frame[5]))
                if len(slowdown_debug[index][1]) < 7 or row[17] != 1:
                    continue
                comparisons.append((frame_index,
                                    slowdown_debug[index][1][:5],
                                    row[18], row[19], row[20], row[12],
                                    frame[5], frame[11]))
            print("  감속 속도 차이 첫 사례", comparisons[:8])
            print("  독립 감속만 활성인 첫 사례", extra[:12],
                  "범위", (extra[0][0], extra[-1][0]) if extra else None)


if __name__ == "__main__":
    main()
