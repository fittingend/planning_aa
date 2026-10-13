"""원본 차로 변경 예측 경로 표식으로 독립 객체 충돌 코어를 근사 재생합니다."""

import argparse
import bisect
import collections
import math
import statistics
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import (Reader, decode_marker_array, decode_odometry,
                                decode_predicted_objects, messages)


DEBUG = ("/planning/scenario_planning/lane_driving/behavior_planning/"
         "behavior_path_planner/debug/lane_change_left")
OBJECTS = "/perception/object_recognition/objects"
ODOMETRY = "/localization/kinematic_state"
INTERNAL_STATE = ("/planning/scenario_planning/lane_driving/behavior_planning/"
                  "behavior_path_planner/debug/internal_state")


def yaw(pose):
    x, y, z, w = pose[3:7]
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def distance(first, second):
    return math.hypot(first[0] - second[0], first[1] - second[1])


def point_in_triangle(point, first, second, third):
    def cross(start, end, target):
        return ((end[0] - start[0]) * (target[1] - start[1]) -
                (end[1] - start[1]) * (target[0] - start[0]))

    values = (cross(first, second, point),
              cross(second, third, point),
              cross(third, first, point))
    return min(values) >= -1e-7 or max(values) <= 1e-7


def expected_reason(marker):
    for line in marker["text"].splitlines():
        if line.startswith("Unsafe reason:"):
            return line.split(":", 1)[1].strip()
    return "safe"


def decode_internal_state(data):
    reader = Reader(data)
    reader.value("i", 4)
    reader.value("I", 4)
    return reader.text()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--ego-speed-limit", type=float)
    parser.add_argument("--ego-speed-bias", type=float, default=0.0)
    parser.add_argument("--ego-time-offset-s", type=float, default=0.0)
    parser.add_argument("--assert-rejection", action="store_true")
    args = parser.parse_args()
    perception = list(messages(args.bag, OBJECTS))
    times = [time for time, _ in perception]
    odometry = [(time, decode_odometry(payload))
                for time, payload in messages(args.bag, ODOMETRY)]
    odometry_times = [time for time, _ in odometry]
    internal_states = [(time, decode_internal_state(payload))
                       for time, payload in messages(args.bag, INTERNAL_STATE)]
    internal_times = [time for time, _ in internal_states]
    encoded = []
    expected = []
    original_rss = []
    original_polygon_kinds = collections.Counter()
    original_filtered_groups = collections.Counter()
    original_group_reasons = collections.Counter()
    filtered_marker_counts = collections.Counter()
    current_object_in_target_marker = 0
    original_lateral_errors = []
    original_object_polygon_indices = collections.Counter()
    original_object_polygon_errors = []
    original_ego_polygon_errors = []
    inferred_ego_speeds = []
    inferred_vs_path_speed = []
    snapshots = collections.defaultdict(list)
    snapshot_object_indices = collections.defaultdict(list)
    snapshot_object_errors = collections.defaultdict(list)
    snapshot_ego_indices = collections.defaultdict(list)
    snapshot_ego_errors = collections.defaultdict(list)
    region_counts = []
    alignment_errors = []
    time_offsets_ms = []
    highest_confidence_matches = 0
    object_path_start_indices = collections.Counter()
    ego_speed_errors = []
    object_speed_component_errors = []
    object_speeds = []
    waiting_approval = []
    internal_state_lags_ms = []
    skipped = collections.Counter()
    for time, payload in messages(args.bag, DEBUG):
        markers = decode_marker_array(payload)
        statuses = [item for item in markers
                    if item["namespace"] == "collision_check_object_info"]
        if len(statuses) != 1:
            continue
        predicted = [item for item in markers
                     if item["namespace"] == "ego_predicted_path"]
        ego = [item["pose"] for item in predicted
               if item["color"][0] < 0.1 and item["color"][1] > 0.9]
        target = [item["pose"] for item in predicted
                  if item["color"][0] > 0.9 and
                  item["color"][1] > 0.9]
        index = bisect.bisect_right(times, time) - 1
        if index < 0 or not ego or not target:
            skipped["경로 또는 인식 없음"] += 1
            continue
        object_pose = statuses[0]["pose"]
        target_lane_markers = [item for item in markers
                               if item["namespace"] == "target_lanes"]
        if any(point_in_triangle(object_pose, *marker["points"][offset:offset + 3])
               for marker in target_lane_markers
               for offset in range(0, len(marker["points"]) - 2, 3)):
            current_object_in_target_marker += 1
        filtered_markers = [item for item in markers
                            if item["namespace"] == "object_filtered_cube"]
        filtered_marker_counts[len(filtered_markers)] += 1
        selected_filtered = [item for item in filtered_markers
                             if distance(item["pose"], object_pose) < 0.02]
        if len(selected_filtered) == 1:
            color = selected_filtered[0]["color"]
            filtered_group = tuple(round(component, 2)
                                   for component in color[:3])
            original_filtered_groups[filtered_group] += 1
        else:
            filtered_group = "대응 불명"
            original_filtered_groups[filtered_group] += 1
        nearby = [candidate for candidate in
                  range(max(0, index - 10),
                        min(len(perception), index + 11))
                  if 0 <= candidate < len(perception) and
                  abs(times[candidate] - time) < 1_000_000_000]
        objects = [(candidate, item) for candidate in nearby for item in
                   decode_predicted_objects(perception[candidate][1])["objects"]]
        if not objects:
            skipped["객체 없음"] += 1
            continue
        selected_index, selected = min(objects, key=lambda entry:
                                       (distance(entry[1]["pose"], object_pose),
                                        abs(times[entry[0]] - time)))
        object_error = distance(selected["pose"], object_pose)
        if object_error > 0.02 or selected["shape_type"] not in (0, 1):
            skipped["객체 연결 실패"] += 1
            continue
        object_speed = selected["twist"][0]
        object_speeds.append(object_speed)
        object_speed_component_errors.append(
            math.hypot(selected["twist"][0], selected["twist"][1]) -
            object_speed)
        time_offsets_ms.append(abs(times[selected_index] - time) / 1e6)
        matches = []
        for path in selected["predicted_paths"]:
            if not path["poses"]:
                continue
            nearest = min(range(len(path["poses"])), key=lambda i:
                          distance(path["poses"][i], target[0]))
            error = distance(path["poses"][nearest], target[0])
            matches.append((error, nearest, path))
        if not matches:
            skipped["예측 경로 없음"] += 1
            continue
        error, start, path = min(matches, key=lambda item: item[0])
        if error > 0.02 or start + len(target) > len(path["poses"]):
            skipped["예측 경로 연결 실패"] += 1
            continue
        alignment_errors.append(error)
        object_path_start_indices[start] += 1
        highest_confidence_matches += (
            path is max(selected["predicted_paths"],
                        key=lambda candidate: candidate["confidence"]))
        step = path["time_step_s"]
        if step <= 0:
            skipped["시각 간격 없음"] += 1
            continue
        odometry_index = bisect.bisect_right(odometry_times, time) - 1
        if odometry_index >= 0 and len(ego) > 1:
            first_speed = distance(ego[0], ego[1]) / step
            ego_speed_errors.append(abs(
                first_speed - odometry[odometry_index][1]["velocity_mps"]))
        regions = []
        for marker in markers:
            if marker["namespace"] not in ("current_lanes", "target_lanes"):
                continue
            points = marker["points"]
            for offset in range(0, len(points) - 2, 3):
                triangle = points[offset:offset + 3]
                cross = ((triangle[1][0] - triangle[0][0]) *
                         (triangle[2][1] - triangle[0][1]) -
                         (triangle[1][1] - triangle[0][1]) *
                         (triangle[2][0] - triangle[0][0]))
                if abs(cross) > 1e-7:
                    regions.append(triangle)
        region_counts.append(len(regions))
        lines = [f"{len(ego)} {len(target)} {len(regions)} "
                 f"{selected['shape_type']} ",
                 f"{selected['dimensions'][0]:.12g} ",
                 f"{selected['dimensions'][1]:.12g}\n"]
        for i, pose in enumerate(ego):
            if i + 1 < len(ego):
                speed = distance(pose, ego[i + 1]) / step
            else:
                speed = distance(ego[i - 1], pose) / step
            speed = max(0.0, speed + args.ego_speed_bias)
            lines.append(f"{i * step + args.ego_time_offset_s:.12g} {pose[0]:.12g} "
                         f"{pose[1]:.12g} {yaw(pose):.12g} "
                         f"{speed:.12g}\n")
        for i, pose in enumerate(target):
            lines.append(f"{(start + i) * step:.12g} {pose[0]:.12g} "
                         f"{pose[1]:.12g} {yaw(pose):.12g} "
                         f"{object_speed:.12g}\n")
        for triangle in regions:
            lines.append(" ".join(
                f"{point[0]:.12g} {point[1]:.12g}"
                for point in triangle) + "\n")
        encoded.append("".join(lines))
        reason = expected_reason(statuses[0])
        expected.append(reason)
        state_index = bisect.bisect_right(internal_times, time) - 1
        if state_index >= 0:
            state_time, state_text = internal_states[state_index]
            candidate_line = next(
                (line for line in state_text.splitlines()
                 if line.startswith("candidate modules")), "")
            waiting_approval.append(
                "[lane_change_left(WAITING_APPROVAL)]" in candidate_line)
            internal_state_lags_ms.append((time - state_time) / 1e6)
        else:
            waiting_approval.append(False)
        original_group_reasons[(filtered_group, reason)] += 1
        debug_fields = dict(
            (line.split(":", 1)[0], line.split(":", 1)[1].strip())
            for line in statuses[0]["text"].splitlines() if ":" in line)
        if reason == "overlap_extended_polygon":
            kind = debug_fields.get("Extended polygon", "")
            original_polygon_kinds[kind] += 1
            if kind == "ego":
                collision_polygons = [item["points"] for item in markers
                                      if item["namespace"] ==
                                      "ego_and_target_polygon_relation"]
                if len(collision_polygons) == 2 and len(collision_polygons[1]) >= 4:
                    original_object = collision_polygons[1][:-1]
                    center = (sum(point[0] for point in original_object) /
                              len(original_object),
                              sum(point[1] for point in original_object) /
                              len(original_object))
                    nearest = min(range(len(target)), key=lambda item:
                                  distance(center, target[item]))
                    original_object_polygon_indices[nearest] += 1
                    original_object_polygon_errors.append(
                        distance(center, target[nearest]))
                    rss_text = debug_fields.get("RSS dist")
                    ego_index = start + nearest
                    offsets = [debug_fields.get(name) for name in (
                        "Extended polygon forward longitudinal offset",
                        "Extended polygon backward longitudinal offset",
                        "Extended polygon lateral offset")]
                    if (all(value is not None for value in offsets) and
                            ego_index < len(ego) and
                            len(collision_polygons[0]) == 5):
                        front, back, lateral = map(float, offsets)
                        pose = ego[ego_index]
                        angle = yaw(pose)
                        cosine, sine = math.cos(angle), math.sin(angle)
                        expected_vertices = [
                            (pose[0] + longitudinal * cosine - side * sine,
                             pose[1] + longitudinal * sine + side * cosine)
                            for longitudinal, side in (
                                (front, lateral), (front, -lateral),
                                (back, -lateral), (back, lateral))]
                        original_vertices = collision_polygons[0][:-1]
                        original_ego_polygon_errors.append(min(
                            sum(distance(expected_vertices[index],
                                         original_vertices[(index + offset) % 4])
                                for index in range(4)) / 4.0
                            for offset in range(4)))
                    if rss_text is not None and 0 < ego_index < len(ego) - 1:
                        discriminant = (9.0 + 2.0 * float(rss_text) +
                                        object_speed * object_speed)
                        if discriminant >= 0.0:
                            inferred_speed = -3.0 + math.sqrt(discriminant)
                            path_speed = (distance(ego[ego_index - 1],
                                                   ego[ego_index + 1]) /
                                          (2.0 * step))
                            inferred_ego_speeds.append(inferred_speed)
                            inferred_vs_path_speed.append(
                                inferred_speed - path_speed)
            offset = debug_fields.get("Extended polygon lateral offset")
            if offset is not None and kind in ("ego", "object"):
                expected_offset = (0.948 if kind == "ego" else
                                   selected["dimensions"][1] * 0.5) + 2.0
                original_lateral_errors.append(
                    abs(float(offset) - expected_offset))
        rss_line = next((line.split(":", 1)[1].strip()
                         for line in statuses[0]["text"].splitlines()
                         if line.startswith("RSS dist:")), "")
        original_rss.append(float(rss_line) if rss_line else None)
        red = [item["pose"] for item in predicted
               if item["color"][0] > 0.9 and
               item["color"][1] < 0.1]
        if len(red) == 2 and all(abs(pose[0]) + abs(pose[1]) > 1.0
                                 for pose in red):
            expected_index = min(
                range(len(path["poses"])),
                key=lambda item: distance(path["poses"][item], red[1]))
            snapshot_object_indices[reason].append(expected_index)
            snapshot_object_errors[reason].append(
                distance(path["poses"][expected_index], red[1]))
            ego_expected_index = min(
                range(len(ego)),
                key=lambda item: distance(ego[item], red[0]))
            snapshot_ego_indices[reason].append(ego_expected_index)
            snapshot_ego_errors[reason].append(
                distance(ego[ego_expected_index], red[0]))
            snapshots[reason].append(
                f"2 1 0 {selected['shape_type']} "
                f"{selected['dimensions'][0]:.12g} "
                f"{selected['dimensions'][1]:.12g}\n"
                f"0 {red[0][0]:.12g} {red[0][1]:.12g} "
                f"{yaw(red[0]):.12g} 0\n"
                f"1 {red[0][0]:.12g} {red[0][1]:.12g} "
                f"{yaw(red[0]):.12g} 0\n"
                f"0 {red[1][0]:.12g} {red[1][1]:.12g} "
                f"{yaw(red[1]):.12g} "
                f"{object_speed:.12g}\n")
    command = [str(args.runner)]
    if args.ego_speed_limit is not None:
        command.append(str(args.ego_speed_limit))
    result = subprocess.run(command, input="".join(encoded),
                            text=True, capture_output=True, check=True)
    actual_rows = [line.split() for line in result.stdout.splitlines()]
    actual = [row[0] for row in actual_rows]
    region_actual = [row[2] for row in actual_rows]
    if len(actual) != len(expected):
        raise RuntimeError("독립 실행기의 결과 수가 입력 수와 다릅니다")
    if args.assert_rejection and (len(actual) != 77 or
                                  len(waiting_approval) != 77 or
                                  not all(waiting_approval) or
                                  any(value not in (
                                      "overlap_polygon",
                                      "overlap_extended_polygon")
                                      for value in expected + actual)):
        raise AssertionError("77개 안전 거부와 원본 승인 대기 상태를 확인하지 못했습니다")
    pairs = collections.Counter(zip(expected, actual))
    region_pairs = collections.Counter(zip(expected, region_actual))
    print(f"비교 {len(actual)}개, 건너뜀 {dict(skipped)}, "
          f"인식 메시지 최대 시간 간격 "
          f"{max(time_offsets_ms, default=0):.1f}ms, "
          f"객체 경로 첫 표본 최대 위치 오차 "
          f"{max(alignment_errors, default=0):.6f}m, "
          f"최고 신뢰도 경로 {highest_confidence_matches}/{len(actual)}")
    print(f"원본 차로 변경 승인 대기 상태 "
          f"{sum(waiting_approval)}/{len(waiting_approval)}, "
          f"상태 메시지 최대 시각 차이 "
          f"{max(internal_state_lags_ms, default=0):.1f}ms")
    print(f"객체 예측 경로 첫 원형 표본 인덱스 "
          f"{dict(sorted(object_path_start_indices.items()))}, "
          f"자차 첫 구간 차분 속도와 최근 차량 상태 속도 차이 중앙값 "
          f"{statistics.median(ego_speed_errors) if ego_speed_errors else 0:.3f}m/s")
    print(f"객체 Twist 벡터 크기와 사용한 x 성분 차이: 최대 "
          f"{max(object_speed_component_errors, default=0):.3f}m/s, "
          f"중앙값 {statistics.median(object_speed_component_errors) if object_speed_component_errors else 0:.3f}m/s")
    print(f"충돌 객체 초기 x 속도: {min(object_speeds, default=0):.3f}~"
          f"{max(object_speeds, default=0):.3f}m/s, "
          f"1m/s 이하 {sum(speed <= 1.0 for speed in object_speeds)}/"
          f"{len(object_speeds)}")
    print(f"기록 차로 표식의 유효 삼각형 수 "
          f"{min(region_counts, default=0)}~{max(region_counts, default=0)}")
    print(f"원본 객체 분류 표식 수 {dict(filtered_marker_counts)}, "
          f"충돌 객체와 일치한 색 {dict(original_filtered_groups)}, "
          f"분류·이유 {dict(original_group_reasons)}, "
          f"현재 객체 중심의 목표 차로 삼각형 포함 "
          f"{current_object_in_target_marker}/{len(actual)}")
    print(f"원본 확장 다각형의 주체 {dict(original_polygon_kinds)}, "
          f"기록 원본 횡확장량과 복원 외형 차이 최대 "
          f"{max(original_lateral_errors, default=0):.3f}m")
    print(f"원본 차량 확장 충돌의 객체 외형 중심이 일치한 예측 표본 "
          f"{dict(sorted(original_object_polygon_indices.items()))}, "
          f"최대 위치 오차 "
          f"{max(original_object_polygon_errors, default=0):.6f}m, "
          f"같은 표본의 확장 자차 외형 재구성 최대 평균 점 오차 "
          f"{max(original_ego_polygon_errors, default=0):.4f}m")
    if inferred_ego_speeds:
        print(f"원본 차량 확장 RSS에서 역산한 자차 속도 "
              f"{len(inferred_ego_speeds)}개: 중앙값 "
              f"{statistics.median(inferred_ego_speeds):.3f}m/s, "
              f"자차 표식 위치 차분과의 차이 중앙값 "
              f"{statistics.median(inferred_vs_path_speed):.3f}m/s")
    for pair, count in sorted(pairs.items()):
        print(f"원본 {pair[0]} / 독립 {pair[1]}: {count}")
    for pair, count in sorted(region_pairs.items()):
        print(f"차로 표식 적용 원본 {pair[0]} / 독립 {pair[1]}: {count}")
    for reason in sorted(set(expected)):
        rows = [(row, rss) for original, row, rss in
                zip(expected, actual_rows, original_rss)
                if original == reason and row[0] != "safe"]
        source_rss = [rss for _, rss in rows if rss is not None]
        if rows:
            fronts = collections.Counter(
                "객체 전방" if int(row[7]) else "객체 후방"
                for row, _ in rows)
            print(f"원본 {reason}의 독립 첫 겹침 방향 {dict(fronts)}, "
                  f"독립 RSS 중앙값 "
                  f"{statistics.median(float(row[4]) for row, _ in rows):.3f}m, "
                  f"원본 RSS 중앙값 "
                  f"{statistics.median(source_rss) if source_rss else 0:.3f}m")
    for reason in sorted(set(expected)):
        indices = [int(row[1]) for original, row in
                   zip(expected, actual_rows) if original == reason]
        if indices:
            print(f"원본 {reason}의 독립 첫 충돌 표본: "
                  f"{min(indices)}~{max(indices)}")
    for reason, inputs in sorted(snapshots.items()):
        result = subprocess.run([str(args.runner)], input="".join(inputs),
                                text=True, capture_output=True, check=True)
        outcomes = collections.Counter(
            line.split()[0] for line in result.stdout.splitlines())
        print(f"원본 {reason} 기대 자세 단일 시각 {len(inputs)}개: "
              f"독립 {dict(outcomes)}, 객체 원형 경로 최근접 인덱스 "
              f"{min(snapshot_object_indices[reason])}~"
              f"{max(snapshot_object_indices[reason])}, 최대 위치 오차 "
              f"{max(snapshot_object_errors[reason]):.3f}m, "
              f"자차 표식 최근접 인덱스 "
              f"{min(snapshot_ego_indices[reason])}~"
              f"{max(snapshot_ego_indices[reason])}, 최대 위치 오차 "
              f"{max(snapshot_ego_errors[reason]):.3f}m")


if __name__ == "__main__":
    main()
