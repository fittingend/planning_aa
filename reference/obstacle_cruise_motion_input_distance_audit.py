"""Audit cruise candidate distance on the recorded motion-velocity trajectory.

This is a conditional source-input comparison. It does not feed the recorded
trajectory into the independent route-to-final pipeline.
"""

import argparse
import bisect
import json
import math
import statistics
import subprocess
from pathlib import Path

from offline_rosbag_cdr import (
    decode_float32_multi_array_stamped, decode_odometry,
    decode_marker_array, decode_predicted_objects, decode_trajectory, messages,
)


PREFIX = "/planning/scenario_planning/lane_driving/motion_planning/"


def pose_yaw(pose):
    x, y, z, w = pose[3:7]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def prior(records, times, receipt):
    index = bisect.bisect_right(times, receipt) - 1
    return records[index] if index >= 0 else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--start", type=int, default=0,
                        help="필수 입력 준비 뒤의 첫 motion trajectory 번호")
    parser.add_argument("--frames", type=int, default=250)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--package-cruise-hysteresis", action="store_true")
    parser.add_argument("--enable-outside-stop-classes", action="store_true")
    parser.add_argument("--stop-collision-time-margin", type=float,
                        default=1.0)
    args = parser.parse_args()
    if args.start < 0 or args.frames < 1:
        parser.error("start와 frames가 유효해야 합니다")
    if (not math.isfinite(args.stop_collision_time_margin) or
            args.stop_collision_time_margin < 0):
        parser.error("정지 충돌 시간 여유가 유효해야 합니다")
    odometry = list(messages(args.bag, "/localization/kinematic_state"))
    perception = list(messages(args.bag,
                               "/perception/object_recognition/objects"))
    if not odometry or not perception:
        raise ValueError("차량 상태와 인식 객체가 필요합니다")
    first_input = max(odometry[0][0], perception[0][0])
    motion = [(receipt, decode_trajectory(raw)) for receipt, raw in messages(
        args.bag, PREFIX + "motion_velocity_planner/trajectory")
        if receipt >= first_input]
    motion = [item for item in motion if len(item[1]["points"]) >= 2]
    debug = [(receipt, decode_float32_multi_array_stamped(raw)["data"])
             for receipt, raw in messages(
                 args.bag,
                 PREFIX + "obstacle_cruise_planner/debug/cruise_planning_info")]
    markers = list(messages(
        args.bag, PREFIX + "obstacle_cruise_planner/debug/marker"))
    odometry_times = [item[0] for item in odometry]
    perception_times = [item[0] for item in perception]
    debug_times = [item[0] for item in debug]
    marker_times = [item[0] for item in markers]
    chosen = motion[args.start:args.start + args.frames]
    if not chosen:
        raise ValueError("선택한 motion trajectory가 없습니다")
    rows = [str(len(chosen))]
    inputs = []
    for valid_index, (receipt, trajectory) in enumerate(
            chosen, args.start):
        odom_record = prior(odometry, odometry_times, receipt)
        perception_record = prior(perception, perception_times, receipt)
        ego = decode_odometry(odom_record[1])
        objects = decode_predicted_objects(perception_record[1])
        points = trajectory["points"]
        rows.append(" ".join(map(str, (
            f"{receipt * 1e-9:.12f}",
            f"{objects['stamp_ns'] * 1e-9:.12f}",
            ego["x"], ego["y"], ego["z"],
            ego["orientation_x"], ego["orientation_y"],
            ego["orientation_z"], ego["orientation_w"],
            ego["velocity_mps"], len(points)))))
        for point in points:
            rows.append(" ".join(map(str, (
                point["x"], point["y"], point["z"],
                point["orientation_x"], point["orientation_y"],
                point["orientation_z"], point["orientation_w"],
                point["velocity"]))))
        rows.append(str(len(objects["objects"])))
        for obj in objects["objects"]:
            classification = (max(obj["classification"],
                                  key=lambda item: item[1])[0]
                              if obj["classification"] else 7)
            footprint = obj["footprint"]
            length, width = obj["dimensions"][:2]
            if obj["shape_type"] == 2 and footprint:
                length = max(p[0] for p in footprint) - min(
                    p[0] for p in footprint)
                width = max(p[1] for p in footprint) - min(
                    p[1] for p in footprint)
            predicted = obj["predicted_paths"]
            rows.append(" ".join(map(str, (
                obj["uuid"], classification, obj["shape_type"],
                obj["pose"][0], obj["pose"][1], pose_yaw(obj["pose"]),
                length, width, obj["twist"][0], obj["twist"][1],
                len(footprint), len(predicted)))))
            rows.extend(f"{p[0]} {p[1]}" for p in footprint)
            for prediction in predicted:
                rows.append(" ".join(map(str, (
                    prediction["time_step_s"], prediction["confidence"],
                    len(prediction["poses"])))))
                rows.extend(f"{p[0]} {p[1]} {pose_yaw(p)}"
                            for p in prediction["poses"])
        debug_index = bisect.bisect_left(debug_times, receipt)
        source = (debug[debug_index]
                  if debug_index < len(debug) and
                  debug[debug_index][0] - receipt <= 50_000_000 else None)
        marker_index = bisect.bisect_left(marker_times, receipt)
        marker_record = (markers[marker_index]
                         if marker_index < len(markers) and
                         markers[marker_index][0] - receipt <= 50_000_000
                         else None)
        source_markers = (decode_marker_array(marker_record[1])
                          if marker_record else [])
        inputs.append({
            "frame": valid_index, "motion_receipt_ns": receipt,
            "odometry_receipt_ns": odom_record[0],
            "objects_receipt_ns": perception_record[0],
            "source_debug_receipt_ns": source[0] if source else None,
            "source_raw_distance_m": source[1][3]
            if source and len(source[1]) > 3 else None,
            "source_stop_marker_count": sum(
                marker["namespace"] == "obstacles_to_stop" and
                marker["action"] == 0 for marker in source_markers),
            "source_cruise_marker_count": sum(
                marker["namespace"] == "obstacles_to_cruise" and
                marker["action"] == 0 for marker in source_markers),
        })
    command = [str(args.runner), "--stop-collision-time-margin",
               str(args.stop_collision_time_margin)]
    if args.package_cruise_hysteresis:
        command.append("--package-cruise-hysteresis")
    if args.enable_outside_stop_classes:
        command.append("--enable-outside-stop-classes")
    result = subprocess.run(
        command, input="\n".join(rows) + "\n", text=True,
        capture_output=True, check=True)
    parsed = result.stdout.splitlines()
    if len(parsed) != len(inputs):
        raise ValueError("후보 실행기 출력 프레임 수가 다릅니다")
    for local_index, (input_record, line) in enumerate(zip(inputs, parsed)):
        frame, count, distance, stop_count = line.split()
        if int(frame) != local_index:
            raise ValueError("후보 실행기 출력 순서가 다릅니다")
        input_record["independent_candidate_count"] = int(count)
        input_record["independent_stop_candidate_count"] = int(stop_count)
        input_record["independent_raw_distance_m"] = (
            float(distance) if int(count) == 1 else None)
    paired = [record for record in inputs
              if record["source_raw_distance_m"] is not None]
    source_active = [r for r in paired if r["source_raw_distance_m"] > 0]
    independent_active = [r for r in paired
                          if r["independent_candidate_count"] > 0]
    common = [r for r in source_active
              if r["independent_candidate_count"] == 1]
    errors = [abs(r["source_raw_distance_m"] -
                  r["independent_raw_distance_m"]) for r in common]
    print(f"{args.bag}: motion 입력 {len(inputs)}프레임, 원본 디버그 "
          f"{len(paired)}, 원본 활성 {len(source_active)}, "
          f"후보 활성 {len(independent_active)}, 단일 후보 공동 활성 {len(common)}")
    print("원본 stop/cruise 표식 활성 "
          f"{sum(r['source_stop_marker_count'] > 0 for r in inputs)}/"
          f"{sum(r['source_cruise_marker_count'] > 0 for r in inputs)}, "
          "독립 stop/cruise 후보 활성 "
          f"{sum(r['independent_stop_candidate_count'] > 0 for r in inputs)}/"
          f"{sum(r['independent_candidate_count'] > 0 for r in inputs)}")
    if source_active and independent_active:
        print(f"첫 활성: 원본 {source_active[0]['frame']}, "
              f"후보 {independent_active[0]['frame']}")
    if errors:
        print(f"공동 활성 원시 거리 절대 오차 중앙/최대: "
              f"{statistics.median(errors):.6f}/{max(errors):.6f}m")
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(inputs, indent=2) + "\n")
        print(f"상세 결과: {args.output_json}")


if __name__ == "__main__":
    main()
