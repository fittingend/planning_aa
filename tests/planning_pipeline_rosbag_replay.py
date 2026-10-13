"""기록된 경로와 최신 차량·인식·신호 입력으로 독립 종단 계산을 실행합니다."""

import argparse
import bisect
import collections
import csv
import math
import statistics
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import (
    decode_acceleration, decode_lanelet_map, decode_odometry,
    decode_float32_multi_array_stamped,
    decode_occupancy_grid, decode_path, decode_lanelet_route,
    decode_operation_mode, decode_velocity_limit,
    decode_predicted_objects,
    decode_traffic_light_groups, decode_trajectory, decode_marker_array,
    messages,
)


PREFIX = "/planning/scenario_planning/lane_driving/behavior_planning/"


def latest(records, stamps, time):
    index = bisect.bisect_right(stamps, time) - 1
    return records[index] if index >= 0 else None


def yaw(pose):
    x, y, z, w = pose[3:7]
    return math.atan2(2 * (w * z + x * y),
                      1 - 2 * (y * y + z * z))


def append_objects(rows, objects):
    rows.append(str(len(objects)))
    for obj in objects:
        classification = (max(obj["classification"],
                              key=lambda item: item[1])[0]
                          if obj["classification"] else 7)
        pose = obj["pose"]
        dimensions = obj["dimensions"]
        footprint = obj["footprint"]
        length = dimensions[0]
        width = dimensions[1]
        if obj["shape_type"] == 2 and footprint:
            length = max(point[0] for point in footprint) - min(
                point[0] for point in footprint)
            width = max(point[1] for point in footprint) - min(
                point[1] for point in footprint)
        predicted = obj["predicted_paths"]
        rows.append(" ".join(map(str, (
            obj["uuid"], classification, obj["shape_type"],
            pose[0], pose[1], yaw(pose), length, width,
            obj["twist"][0], obj["twist"][1], len(footprint),
            len(predicted)))))
        rows.extend(f"{vertex[0]} {vertex[1]}" for vertex in footprint)
        for predicted_path in predicted:
            rows.append(" ".join(map(str, (
                predicted_path["time_step_s"],
                predicted_path["confidence"],
                len(predicted_path["poses"])))))
            rows.extend(f"{pose[0]} {pose[1]} {yaw(pose)}"
                        for pose in predicted_path["poses"])


def planned_metrics_at_ego(points, ego_x, ego_y):
    """Project ego onto a planned path; report speed, along-path a, and stop arc."""
    xyz = [((p["x"], p["y"], p["velocity"])
            if isinstance(p, dict) else (p[0], p[1], p[2])) for p in points]
    if len(xyz) < 2:
        return (None, None, None, None)
    arcs = [0.0]
    for a, b in zip(xyz, xyz[1:]):
        arcs.append(arcs[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    best = (math.inf, 0, 0.0)
    for index, (a, b) in enumerate(zip(xyz, xyz[1:])):
        length = arcs[index + 1] - arcs[index]
        if length < 1e-9:
            continue
        ratio = max(0.0, min(1.0, ((ego_x - a[0]) * (b[0] - a[0]) +
                                      (ego_y - a[1]) * (b[1] - a[1])) /
                                     length**2))
        distance = math.hypot(ego_x - a[0] - ratio * (b[0] - a[0]),
                              ego_y - a[1] - ratio * (b[1] - a[1]))
        if distance < best[0]:
            best = distance, index, ratio
    distance, index, ratio = best
    if not math.isfinite(distance):
        return (None, None, None, None)
    a, b = xyz[index:index + 2]
    span = arcs[index + 1] - arcs[index]
    speed = a[2] + ratio * (b[2] - a[2])
    ego_arc = arcs[index] + ratio * span
    def speed_at_arc(query):
        segment = max(0, min(len(arcs) - 2,
                             bisect.bisect_right(arcs, query) - 1))
        distance = arcs[segment + 1] - arcs[segment]
        fraction = ((query - arcs[segment]) / distance
                    if distance > 1e-9 else 0.0)
        return (xyz[segment][2] + fraction *
                (xyz[segment + 1][2] - xyz[segment][2]))
    # A 4 m spatial window avoids division by tiny adjacent point spacing.
    before_arc = max(0.0, ego_arc - 2.0)
    after_arc = min(arcs[-1], ego_arc + 2.0)
    acceleration = None
    if after_arc - before_arc > 0.1:
        before_speed = speed_at_arc(before_arc)
        after_speed = speed_at_arc(after_arc)
        acceleration = ((after_speed**2 - before_speed**2) /
                        (2 * (after_arc - before_arc)))
    first_stop = next((arcs[i] - ego_arc for i in range(index + 1, len(xyz))
                       if xyz[i][2] <= 0.01 and arcs[i] >= ego_arc), None)
    return speed, acceleration, first_stop, distance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frames", type=int, default=5)
    parser.add_argument("--offset", type=int, default=0)
    parser.add_argument("--expected-complete-frames", type=int)
    parser.add_argument("--with-occupancy-grid", action="store_true",
                        help="기록된 점유 격자를 가림 계산에도 넣습니다")
    parser.add_argument("--without-dynamic-stop", action="store_true",
                        help="동적 장애물 정지 단계의 영향을 분리해 확인합니다")
    parser.add_argument("--disable-velocity-module", action="append", default=[],
                        choices=("crosswalk", "blind_spot", "intersection",
                                 "run_out", "traffic_light"),
                        help="행동 속도 정지 원인 진단을 위해 지정한 모듈을 제외합니다")
    parser.add_argument("--with-cruise-candidates", action="store_true",
                        help="기록된 객체에서 종단 순항 정지 후보를 생성합니다")
    parser.add_argument("--with-cruise-slowdown", action="store_true",
                        help="원본 순항 노드의 객체 감속 후보를 진단 재생합니다")
    parser.add_argument("--with-recorded-external-limit", action="store_true",
                        help="원본 Planning의 max_velocity 출력을 스무더 입력으로 재사용하는 별도 진단입니다")
    parser.add_argument("--strict-cruise", action="store_true",
                        help="별도 배포본의 순항 후보 임계값을 적용합니다")
    parser.add_argument("--package-cruise-hysteresis", action="store_true",
                        help="패키지 YAML의 내부 객체 순항/정지 전환 속도 3.5/3.0m/s를 시험합니다")
    parser.add_argument("--from-route", action="store_true",
                        help="기록된 행동 경로 대신 route와 odometry 시각으로 독립 재생합니다")
    parser.add_argument("--simulation", action="store_true",
                        help="원본 노드의 is_simulation 설정을 적용합니다")
    parser.add_argument("--crosswalk-auto-activation", action="store_true",
                        help="원본 횡단보도 enable_rtc=false의 안전 기반 활성화를 적용합니다")
    parser.add_argument("--lane-change-approval-csv", type=Path,
                        help="외부 승인 입력의 frame,approved(0/1) CSV입니다. 지정하지 않으면 승인하지 않습니다")
    parser.add_argument("--lane-change-approval-timeline-csv", type=Path,
                        help="외부 승인 입력의 receipt_ns,approved(0/1) CSV입니다. 행동 경로 출력 수신 시각 이후의 프레임에 상태를 적용하는 진단입니다")
    parser.add_argument("--experimental-route-sequence-clip", action="store_true",
                        help="원시 중심선·목표점 선행 절단과 원본 후방 인덱스를 실험적으로 사용합니다")
    parser.add_argument("--odometry-receipt-lag-ms", type=float, default=0.0,
                        help="입력 시각 진단용: 행동 경로 수신보다 지정한 ms 앞의 odometry를 고릅니다")
    parser.add_argument("--route-backward-path-length", type=float, default=5.0,
                        help="행동 경로의 후방 길이; 원본 기본 설정은 5m입니다")
    parser.add_argument("--callback-clock", choices=("odometry", "recorded-behavior-receipt"),
                        default="odometry",
                        help="route 재생 콜백 시계; 중간 출력의 값은 입력으로 쓰지 않습니다")
    parser.add_argument("--input-snapshot-clock",
                        choices=("behavior-receipt", "root-marker-receipt"),
                        default="behavior-receipt",
                        help="진단용 입력 조회 상한: root 표식은 원본 콜백의 입력 조회 이후 발행됩니다")
    parser.add_argument("--with-recorded-behavior-control", action="store_true",
                        help="원본 행동 경로의 STOP/AUTONOMOUS 모드별 현재 차로 재선택을 시험합니다")
    parser.add_argument("--assert-route-behavior-reference", action="store_true",
                        help="정지선 기준 기록의 행동 경로 점 수·시작점·속도를 검사합니다")
    parser.add_argument("--assert-behavior-velocity-after-warmup", action="store_true",
                        help="첫 프레임 뒤 행동 속도 출력의 원본 점 수·위치·속도를 비교합니다")
    parser.add_argument("--assert-final-speed-after-warmup", action="store_true",
                        help="첫 프레임 뒤 최종 평활화의 점 수·속도를 원본과 비교합니다")
    parser.add_argument("--diagnose-behavior-velocity", action="store_true",
                        help="행동 속도 출력의 점 수·양 끝·첫 정지점을 프레임별로 표시합니다")
    parser.add_argument("--diagnose-route-reference", action="store_true",
                        help="원본 root_reference_path 표식과 목표점 연결 전 경로를 비교합니다")
    parser.add_argument("--diagnose-final-speed", action="store_true",
                        help="시나리오와 최종 평활화의 첫 정지점·종단 속도를 표시합니다")
    parser.add_argument("--diagnose-frame-index", type=int, action="append",
                        help="지정한 전체 재생 프레임의 단계별 위치와 차로 ID를 표시합니다")
    parser.add_argument("--diagnose-segments", action="store_true",
                        help="원본 대비 단계별 일치·점 수·수치 이탈의 연속 구간을 표시합니다")
    parser.add_argument("--diagnose-spatial-final", action="store_true",
                        help="점 수가 다른 최종 경로도 5cm 이내 최근접 선분으로 속도를 비교합니다")
    parser.add_argument("--allow-incomplete-source-chain", action="store_true",
                        help="다른 rosbag 탐색에서 최종 출력의 헤더 시각 대응 누락을 허용합니다")
    parser.add_argument("--allow-incomplete-calculation", action="store_true",
                        help="일부 프레임의 독립 종단 계산 실패를 결측값으로 기록합니다")
    parser.add_argument("--smoother-node-max-velocity", type=float, default=11.1,
                        help="외부 max_velocity 메시지가 없을 때 사용할 평활화 노드 설정값")
    parser.add_argument("--cruise-safe-distance-margin", type=float, default=4.0,
                        help="원본 순항 계획기의 런타임 안전거리 설정값 (기본 launch: 4m)")
    parser.add_argument("--cruise-stop-to-cruise-speed", type=float,
                        help="정지 상태 객체가 순항으로 바뀌는 원본 임계 속도 (m/s)")
    parser.add_argument("--cruise-cruise-to-stop-speed", type=float,
                        help="순항 상태 객체가 정지로 바뀌는 원본 임계 속도 (m/s)")
    parser.add_argument("--cruise-outside-predicted-path-count", type=int, default=3,
                        help="차로 밖 순항 충돌 검사에 사용할 신뢰도 상위 예측 경로 수")
    parser.add_argument("--cruise-max-lateral-time-margin", type=float, default=5.0,
                        help="원본 순항 후보의 횡방향 진입 시간 상한 (기본 launch: 5초)")
    parser.add_argument("--cruise-stop-collision-time-margin", type=float,
                        default=1.0,
                        help="내부 정지 객체의 미래 충돌 확인 여유 (기본 launch: 1초)")
    parser.add_argument("--enable-outside-stop-classes", action="store_true",
                        help="패키지 기본 YAML의 차로 밖 정지 객체 종류를 켜는 진단 설정")
    parser.add_argument("--cruise-outside-stop-predicted-path-count", type=int,
                        default=1)
    parser.add_argument("--cruise-outside-stop-lateral-time", type=float,
                        default=4.5)
    parser.add_argument("--cruise-pedestrian-deceleration", type=float,
                        default=0.5)
    parser.add_argument("--cruise-bicycle-deceleration", type=float,
                        default=0.5)
    parser.add_argument("--cruise-callback-clock",
                        choices=("frame", "recorded-optimizer-receipt",
                                 "recorded-motion-velocity-receipt"),
                        default="frame",
                        help="진단용 순항 콜백 시계; 원본 중간 출력의 수신 시각만 사용합니다")
    parser.add_argument("--export-series-csv", type=Path,
                        help="원본과 독립 최종 궤적의 ego 위치 시계열을 CSV로 저장합니다")
    args = parser.parse_args()
    if args.frames < 1 or args.offset < 0:
        parser.error("프레임 수와 시작 위치가 유효해야 합니다")
    if not math.isfinite(args.odometry_receipt_lag_ms) or args.odometry_receipt_lag_ms < 0:
        parser.error("odometry 지연값은 유효한 0 이상의 수여야 합니다")
    if not math.isfinite(args.route_backward_path_length) or args.route_backward_path_length < 0:
        parser.error("행동 경로 후방 길이는 유효한 0 이상의 수여야 합니다")
    if not math.isfinite(args.smoother_node_max_velocity) or args.smoother_node_max_velocity < 0:
        parser.error("평활화 노드 최대 속도가 유효해야 합니다")
    if not math.isfinite(args.cruise_safe_distance_margin) or args.cruise_safe_distance_margin < 0:
        parser.error("순항 안전거리 마진은 유효한 0 이상의 수여야 합니다")
    cruise_entry = (args.cruise_stop_to_cruise_speed if
                    args.cruise_stop_to_cruise_speed is not None else
                    (3.5 if args.package_cruise_hysteresis else 1.5))
    cruise_keep = (args.cruise_cruise_to_stop_speed if
                   args.cruise_cruise_to_stop_speed is not None else
                   (3.0 if args.package_cruise_hysteresis else 1.0))
    if (not math.isfinite(cruise_entry) or not math.isfinite(cruise_keep) or
            cruise_keep < 0 or cruise_entry < cruise_keep):
        parser.error("순항 전환 임계값은 0 이상이며 유지 속도가 진입 속도 이하여야 합니다")
    if args.cruise_outside_predicted_path_count < 1:
        parser.error("차로 밖 순항 예측 경로 수는 1 이상이어야 합니다")
    if args.cruise_outside_stop_predicted_path_count < 1:
        parser.error("차로 밖 정지 예측 경로 수는 1 이상이어야 합니다")
    if (not math.isfinite(args.cruise_outside_stop_lateral_time) or
            args.cruise_outside_stop_lateral_time < 0):
        parser.error("차로 밖 정지 진입 시간은 유효한 0 이상의 수여야 합니다")
    if (not math.isfinite(args.cruise_pedestrian_deceleration) or
            args.cruise_pedestrian_deceleration <= 0 or
            not math.isfinite(args.cruise_bicycle_deceleration) or
            args.cruise_bicycle_deceleration <= 0):
        parser.error("보행자·자전거 감속률은 유효한 양수여야 합니다")
    if (not math.isfinite(args.cruise_max_lateral_time_margin) or
            args.cruise_max_lateral_time_margin < 0):
        parser.error("순항 횡방향 진입 시간은 유효한 0 이상의 수여야 합니다")
    if (not math.isfinite(args.cruise_stop_collision_time_margin) or
            args.cruise_stop_collision_time_margin < 0):
        parser.error("정지 충돌 시간 여유는 유효한 0 이상의 수여야 합니다")
    if (args.cruise_callback_clock != "frame" and
            (not args.from_route or not args.with_cruise_candidates or
             args.callback_clock != "recorded-behavior-receipt")):
        parser.error("순항 콜백 진단에는 route 독립 재생, 후보, 행동 경로 수신 시계가 필요합니다")
    if args.callback_clock != "odometry" and not args.from_route:
        parser.error("--callback-clock은 --from-route와 함께 사용합니다")
    if (args.lane_change_approval_csv or args.lane_change_approval_timeline_csv) and not args.from_route:
        parser.error("차로 변경 승인 입력은 route 독립 재생에서만 사용합니다")
    if args.lane_change_approval_csv and args.lane_change_approval_timeline_csv:
        parser.error("프레임 승인과 수신 시각 승인은 동시에 지정할 수 없습니다")
    lane_change_approvals = {}
    if args.lane_change_approval_csv:
        with args.lane_change_approval_csv.open(newline="") as stream:
            for row in csv.DictReader(stream):
                frame = int(row["frame"])
                approved = row["approved"]
                if frame < 0 or frame in lane_change_approvals or approved not in ("0", "1"):
                    parser.error("차로 변경 승인 CSV에는 중복 없는 0 이상의 frame과 0/1 approved가 필요합니다")
                lane_change_approvals[frame] = approved == "1"
    approval_timeline = []
    if args.lane_change_approval_timeline_csv:
        with args.lane_change_approval_timeline_csv.open(newline="") as stream:
            for row in csv.DictReader(stream):
                receipt_ns = int(row["receipt_ns"])
                approved = row["approved"]
                if (receipt_ns < 0 or approved not in ("0", "1") or
                        (approval_timeline and receipt_ns <= approval_timeline[-1][0])):
                    parser.error("승인 수신 시각 CSV에는 오름차순 고유 receipt_ns와 0/1 approved가 필요합니다")
                approval_timeline.append((receipt_ns, approved == "1"))
    if args.diagnose_frame_index and (
            not args.from_route or
            args.callback_clock != "recorded-behavior-receipt"):
        parser.error("--diagnose-frame-index는 기록된 행동 경로 수신 시계와 함께 사용합니다")
    if args.assert_route_behavior_reference and \
            args.callback_clock != "recorded-behavior-receipt":
        parser.error("행동 경로 기준 검사는 기록된 행동 경로 수신 시계가 필요합니다")
    if args.assert_behavior_velocity_after_warmup and (
            not args.from_route or
            args.callback_clock != "recorded-behavior-receipt"):
        parser.error("행동 속도 연속 검사는 route 독립 재생과 기록된 행동 경로 수신 시계가 필요합니다")
    if args.assert_final_speed_after_warmup and (
            not args.from_route or
            args.callback_clock != "recorded-behavior-receipt"):
        parser.error("최종 속도 연속 검사는 route 독립 재생과 기록된 행동 경로 수신 시계가 필요합니다")
    if args.diagnose_route_reference and (
            not args.from_route or
            args.callback_clock != "recorded-behavior-receipt"):
        parser.error("기준 경로 진단에는 route와 기록된 행동 경로 수신 시계가 필요합니다")
    if args.export_series_csv and (not args.from_route or
            args.callback_clock != "recorded-behavior-receipt"):
        parser.error("시계열 CSV는 route 독립 재생의 기록된 행동 경로 수신 시계가 필요합니다")
    map_record = next(messages(args.bag, "/map/vector_map"), None)
    if map_record is None:
        raise ValueError("이진 지도가 없습니다")
    route_record = next(messages(args.bag, "/planning/mission_planning/route"), None)
    if args.from_route and route_record is None:
        raise ValueError("route 입력이 없습니다")
    paths = ([] if args.from_route else
             list(messages(args.bag, PREFIX + "path_with_lane_id")))
    targets = ([] if args.from_route else
               list(messages(args.bag, PREFIX + "path")))
    motion_records = [] if args.from_route else [
        (receipt, decode_trajectory(raw)) for receipt, raw in messages(
            args.bag,
            "/planning/scenario_planning/lane_driving/motion_planning/"
            "motion_velocity_planner/trajectory")]
    motion_targets = {decoded["stamp_ns"]: decoded
                      for _, decoded in motion_records}
    motion_receipts = {decoded["stamp_ns"]: receipt
                       for receipt, decoded in motion_records}
    stop_debug = ([(receipt,
                    decode_float32_multi_array_stamped(raw)["data"])
                   for receipt, raw in messages(
                       args.bag,
                       "/planning/scenario_planning/lane_driving/"
                       "motion_planning/obstacle_cruise_planner/"
                       "debug/stop_planning_info")]
                  if args.with_cruise_candidates else [])
    stop_debug_times = [receipt for receipt, _ in stop_debug]
    cruise_debug = ([(receipt,
                     decode_float32_multi_array_stamped(raw)["data"])
                    for receipt, raw in messages(
                        args.bag,
                        "/planning/scenario_planning/lane_driving/"
                        "motion_planning/obstacle_cruise_planner/"
                        "debug/cruise_planning_info")]
                   if args.export_series_csv and args.with_cruise_candidates
                   else [])
    cruise_debug_times = [receipt for receipt, _ in cruise_debug]
    slowdown_debug = ([(receipt,
                       decode_float32_multi_array_stamped(raw)["data"])
                      for receipt, raw in messages(
                          args.bag,
                          "/planning/scenario_planning/lane_driving/"
                          "motion_planning/obstacle_cruise_planner/"
                          "debug/slow_down_planning_info")]
                      if args.export_series_csv and args.with_cruise_slowdown
                      else [])
    slowdown_debug_times = [receipt for receipt, _ in slowdown_debug]
    smooth_targets = {} if args.from_route else {
        decoded["stamp_ns"]: decoded for _, raw in messages(
            args.bag,
            "/planning/scenario_planning/lane_driving/motion_planning/"
            "path_smoother/path")
        for decoded in (decode_path(raw),)}
    optimizer_targets = {} if args.from_route else {
        decoded["stamp_ns"]: decoded for _, raw in messages(
            args.bag,
            "/planning/scenario_planning/lane_driving/motion_planning/"
            "path_optimizer/trajectory")
        for decoded in (decode_trajectory(raw),)}
    final_targets = {} if args.from_route else {
        decoded["stamp_ns"]: decoded for _, raw in messages(
            args.bag, "/planning/scenario_planning/lane_driving/trajectory")
        for decoded in (decode_trajectory(raw),)}
    odometry = list(messages(args.bag, "/localization/kinematic_state"))
    acceleration = list(messages(args.bag, "/localization/acceleration"))
    perception = list(messages(args.bag,
                               "/perception/object_recognition/objects"))
    signals = list(messages(args.bag,
                            "/perception/traffic_light_recognition/traffic_signals"))
    occupancy = (list(messages(args.bag,
                               "/perception/occupancy_grid_map/map"))
                 if args.with_occupancy_grid else [])
    operation_modes = (list(messages(args.bag, "/system/operation_mode/state"))
                       if args.from_route else [])
    external_limits = (list(messages(args.bag,
                                    "/planning/scenario_planning/max_velocity"))
                       if args.from_route and args.with_recorded_external_limit else [])
    behavior_clocks = (list(messages(args.bag, PREFIX + "path_with_lane_id"))
                       if args.from_route and args.callback_clock ==
                       "recorded-behavior-receipt" else [])
    optimizer_clock_times = ([receipt for receipt, _ in messages(
        args.bag, "/planning/scenario_planning/lane_driving/"
                  "motion_planning/path_optimizer/trajectory")]
        if args.cruise_callback_clock == "recorded-optimizer-receipt" else [])
    cruise_input_clock_times = ([receipt for receipt, _ in messages(
        args.bag, "/planning/scenario_planning/lane_driving/"
                  "motion_planning/motion_velocity_planner/trajectory")]
        if (args.cruise_callback_clock == "recorded-motion-velocity-receipt" or
            args.from_route and args.with_cruise_candidates and
            args.export_series_csv) else [])
    stage_clock_times = (
        optimizer_clock_times if args.cruise_callback_clock ==
        "recorded-optimizer-receipt" else cruise_input_clock_times)
    if args.cruise_callback_clock != "frame" and not stage_clock_times:
        raise ValueError("순항 콜백 진단에 필요한 중간 출력 수신 시각이 없습니다")
    if args.input_snapshot_clock == "root-marker-receipt" and (
            not args.from_route or
            args.callback_clock != "recorded-behavior-receipt"):
        parser.error("root 표식 입력 시계는 route의 행동 경로 수신 시계에서만 사용할 수 있습니다")
    root_markers = (list(messages(args.bag, PREFIX +
                         "behavior_path_planner/debug/root_reference_path"))
                    if args.diagnose_route_reference or
                    args.input_snapshot_clock == "root-marker-receipt" else [])
    root_marker_times = [time for time, _ in root_markers]
    if (not args.from_route and not paths) or not odometry or not perception:
        raise ValueError("필수 경로·차량 위치·인식 객체가 필요합니다")
    odometry_times = [time for time, _ in odometry]
    acceleration_times = [time for time, _ in acceleration]
    perception_times = [time for time, _ in perception]
    signal_times = [time for time, _ in signals]
    occupancy_times = [time for time, _ in occupancy]
    operation_mode_times = [time for time, _ in operation_modes]
    external_limit_times = [time for time, _ in external_limits]
    if args.from_route:
        start = max(route_record[0], odometry_times[0], perception_times[0])
        clock_records = (behavior_clocks if behavior_clocks else odometry)
        eligible = [(index, record) for index, record in enumerate(clock_records)
                    if record[0] >= start]
        if not behavior_clocks:
            eligible = eligible[::5]
    else:
        eligible = [(index, record) for index, record in enumerate(paths)
                    if record[0] >= max(odometry_times[0], perception_times[0])]
    chosen = eligible[args.offset:args.offset + args.frames]
    if not chosen:
        raise ValueError("필수 입력이 모두 도착한 뒤의 계획 프레임이 없습니다")
    root_marker_indices = [None] * len(chosen)
    if root_markers:
        for frame_index, (original_index, (receipt, _)) in enumerate(chosen):
            marker_index = bisect.bisect_right(root_marker_times, receipt) - 1
            previous_behavior_receipt = (
                behavior_clocks[original_index - 1][0]
                if original_index > 0 and behavior_clocks else -1)
            paired = (marker_index >= 0 and
                      root_marker_times[marker_index] > previous_behavior_receipt and
                      receipt - root_marker_times[marker_index] <= 50_000_000)
            root_marker_indices[frame_index] = marker_index if paired else None
    rows = [str(len(chosen))]
    if args.from_route:
        route = decode_lanelet_route(route_record[1])
        goal = route["goal"]
        rows.append(" ".join(map(str, (
            goal[0], goal[1], yaw(goal), goal[2], len(route["segments"]),
            route["uuid"]))))
        for section in route["segments"]:
            primitives = [item[0] for item in section["primitives"]]
            rows.append(" ".join(map(str, (
                section["preferred"][0], len(primitives), *primitives))))
    decoded_inputs = []
    frame_odometry = []
    snapshot_fallbacks = 0
    snapshot_input_changes = collections.Counter()
    cruise_stage_inputs = []
    cruise_clock_lags_ms = []
    cruise_clock_fallbacks = 0
    ages = {"차량": [], "인식": [], "신호": [], "점유 격자": []}
    if lane_change_approvals and max(lane_change_approvals) >= len(chosen):
        parser.error("차로 변경 승인 frame이 재생 프레임 범위를 벗어났습니다")
    if approval_timeline:
        approval_times = [event[0] for event in approval_timeline]
        for frame_index, (_, (receipt, _)) in enumerate(chosen):
            event_index = bisect.bisect_right(approval_times, receipt) - 1
            lane_change_approvals[frame_index] = (
                approval_timeline[event_index][1] if event_index >= 0 else False)
    for frame_index, (_, (receipt, raw)) in enumerate(chosen):
        snapshot_receipt = receipt
        if args.input_snapshot_clock == "root-marker-receipt":
            marker_index = root_marker_indices[frame_index]
            if marker_index is not None:
                snapshot_receipt = root_marker_times[marker_index]
            else:
                snapshot_fallbacks += 1
        path = ({"points": [], "lane_ids": [], "left_bound": [],
                 "right_bound": []} if args.from_route else
                decode_path(raw, with_lane_ids=True))
        decoded_inputs.append(path)
        odom_record = latest(
            odometry, odometry_times,
            snapshot_receipt - round(args.odometry_receipt_lag_ms * 1_000_000))
        object_record = latest(perception, perception_times, snapshot_receipt)
        accel_record = latest(acceleration, acceleration_times, snapshot_receipt)
        signal_record = latest(signals, signal_times, snapshot_receipt)
        occupancy_record = latest(occupancy, occupancy_times, snapshot_receipt)
        if odom_record is None or object_record is None:
            raise ValueError("계획 프레임 이전의 필수 입력이 없습니다")
        ego = decode_odometry(odom_record[1])
        frame_odometry.append((odom_record[0], ego))
        perceived = decode_predicted_objects(object_record[1])
        objects = perceived["objects"]
        accel = (decode_acceleration(accel_record[1])["acceleration_mps2"]
                 if accel_record else 0.0)
        observed = (decode_traffic_light_groups(signal_record[1])
                    if signal_record else {"stamp_ns": 0, "groups": []})
        if args.from_route:
            mode_record = latest(operation_modes, operation_mode_times,
                                 snapshot_receipt)
            limit_record = latest(external_limits, external_limit_times,
                                  snapshot_receipt)
            mode = (decode_operation_mode(mode_record[1]) if mode_record else
                    {"mode": 0, "is_autoware_control_enabled": False})
            external_limit = (decode_velocity_limit(limit_record[1])
                              if limit_record else None)
            maximum_velocity = (external_limit["max_velocity_mps"]
                                if external_limit else
                                args.smoother_node_max_velocity)
        for label, record in (("차량", odom_record),
                              ("인식", object_record),
                              ("신호", signal_record),
                              ("점유 격자", occupancy_record)):
            if record:
                ages[label].append((receipt - record[0]) * 1e-6)
                if record[0] > snapshot_receipt:
                    raise AssertionError("입력 조회 상한 뒤의 메시지를 사용했습니다")
                if record[0] <= snapshot_receipt < receipt:
                    latest_at_output = latest(
                        {"차량": odometry, "인식": perception,
                         "신호": signals, "점유 격자": occupancy}[label],
                        {"차량": odometry_times, "인식": perception_times,
                         "신호": signal_times, "점유 격자": occupancy_times}[label],
                        receipt)
                    if latest_at_output and latest_at_output[0] != record[0]:
                        snapshot_input_changes[label] += 1
        rows.append(" ".join(map(str, (
            receipt * 1e-9, ego["x"], ego["y"], ego["z"],
            ego["orientation_x"], ego["orientation_y"],
            ego["orientation_z"], ego["orientation_w"],
            ego["velocity_mps"], accel,
            perceived["stamp_ns"] * 1e-9,
            len(path["points"])))))
        if args.from_route:
            rows.append(" ".join(map(str, (
                ego["stamp_ns"] * 1e-9, int(mode["mode"] in (1, 2)),
                int(mode["is_autoware_control_enabled"]),
                maximum_velocity, args.smoother_node_max_velocity,
                int(external_limit is not None),
                int(external_limit["use_constraints"]) if external_limit else 0,
                external_limit["min_acceleration_mps2"] if external_limit else -1.0,
                external_limit["max_jerk_mps3"] if external_limit else 1.0,
                external_limit["min_jerk_mps3"] if external_limit else -1.0,
                int(lane_change_approvals.get(frame_index, False)),
                *((int(mode["mode"] == 2),)
                  if args.with_recorded_behavior_control else ())))))
        for point, lane_ids in zip(path["points"], path["lane_ids"]):
            rows.append(" ".join(map(str, (
                *point[:10], len(lane_ids), *lane_ids))))
        for bound in (path["left_bound"], path["right_bound"]):
            rows.append(str(len(bound)))
            rows.extend(f"{point[0]} {point[1]}" for point in bound)
        groups = []
        for group in observed["groups"]:
            elements = [(color, shape) for color, shape, status, _ in
                        group["elements"]]
            groups.append((group["id"], elements))
        rows.append(str(len(groups)))
        for identifier, elements in groups:
            rows.append(f"{identifier} {observed['stamp_ns'] * 1e-9} "
                        f"{len(elements)}")
            rows.extend(f"{color} {shape}" for color, shape in elements)
        if occupancy_record:
            grid = decode_occupancy_grid(occupancy_record[1])
            origin = grid["origin"]
            rows.append(" ".join(map(str, (
                1, grid["width"], grid["height"],
                grid["resolution_m"], origin[0], origin[1], yaw(origin)))))
            rows.append(" ".join(map(str, grid["values"])))
        else:
            rows.append("0")
        if args.cruise_callback_clock != "frame":
            # The recorded trajectory is only a clock. The downstream path
            # remains the independently calculated optimizer result.
            clock_index = bisect.bisect_left(stage_clock_times, receipt)
            clock = (stage_clock_times[clock_index]
                     if clock_index < len(stage_clock_times) and
                     stage_clock_times[clock_index] - receipt < 90_000_000
                     else receipt)
            if clock == receipt:
                cruise_clock_fallbacks += 1
            else:
                cruise_clock_lags_ms.append((clock - receipt) * 1e-6)
            cruise_odom_record = latest(odometry, odometry_times, clock)
            cruise_object_record = latest(perception, perception_times, clock)
            cruise_accel_record = latest(acceleration, acceleration_times, clock)
            if cruise_odom_record is None or cruise_object_record is None:
                raise ValueError("순항 콜백 시각 이전의 필수 입력이 없습니다")
            cruise_ego = decode_odometry(cruise_odom_record[1])
            cruise_objects = decode_predicted_objects(cruise_object_record[1])
            cruise_accel = (decode_acceleration(
                cruise_accel_record[1])["acceleration_mps2"]
                if cruise_accel_record else 0.0)
            cruise_stage_inputs.append((clock, cruise_odom_record[0],
                                        cruise_object_record[0],
                                        cruise_objects["stamp_ns"],
                                        cruise_accel_record[0]
                                        if cruise_accel_record else None,
                                        cruise_accel))
            rows.append(" ".join(map(str, (
                clock * 1e-9, cruise_ego["x"], cruise_ego["y"],
                cruise_ego["z"], cruise_ego["orientation_x"],
                cruise_ego["orientation_y"], cruise_ego["orientation_z"],
                cruise_ego["orientation_w"],
                cruise_ego["velocity_mps"], cruise_accel))))
            rows.append(str(cruise_objects["stamp_ns"] * 1e-9))
            append_objects(rows, cruise_objects["objects"])
        else:
            cruise_stage_inputs.append((receipt, odom_record[0],
                                        object_record[0],
                                        perceived["stamp_ns"],
                                        accel_record[0] if accel_record else None,
                                        accel))
        append_objects(rows, objects)
    if args.input_snapshot_clock == "root-marker-receipt":
        print(f"행동 입력 조회 상한: root 표식 {len(chosen) - snapshot_fallbacks}/"
              f"{len(chosen)}프레임, 대응 표식 없음 {snapshot_fallbacks}프레임; "
              f"행동 경로 수신 시각까지 새 메시지: {dict(snapshot_input_changes)}")
    if args.cruise_callback_clock != "frame":
        print(f"순항 콜백 진단: {args.cruise_callback_clock} "
              f"{len(cruise_clock_lags_ms)}/"
              f"{len(chosen)}, 공통 시계 대체 {cruise_clock_fallbacks}, "
              f"행동 경로 뒤 중앙 지연 "
              f"{statistics.median(cruise_clock_lags_ms) if cruise_clock_lags_ms else float('nan'):.1f}ms")
    with tempfile.TemporaryDirectory() as directory:
        map_file = Path(directory) / "지도.bin"
        map_file.write_bytes(decode_lanelet_map(map_record[1]))
        command = [str(args.runner), str(map_file)]
        if args.without_dynamic_stop:
            command.append("--without-dynamic-stop")
        for module in args.disable_velocity_module:
            command.extend(("--disable-velocity-module", module))
        for frame_index in args.diagnose_frame_index or []:
            command.extend(("--diagnose-velocity-frame", str(frame_index)))
        if args.with_cruise_candidates:
            command.append("--with-cruise-candidates")
        if args.with_cruise_slowdown:
            command.append("--with-cruise-slowdown")
        if args.cruise_callback_clock != "frame":
            command.append("--cruise-callback-snapshot")
        if args.with_recorded_external_limit:
            command.append("--use-recorded-external-limit")
        if args.strict_cruise:
            command.append("--strict-cruise")
        if args.package_cruise_hysteresis:
            command.append("--package-cruise-hysteresis")
        if args.enable_outside_stop_classes:
            command.append("--enable-outside-stop-classes")
        if args.from_route:
            command.append("--from-route")
        if args.with_recorded_behavior_control:
            if not args.from_route:
                parser.error("행동 경로 제어 모드 진단에는 --from-route가 필요합니다")
            command.append("--with-recorded-behavior-control")
        if args.simulation:
            command.append("--simulation")
        if args.crosswalk_auto_activation:
            command.append("--crosswalk-auto-activation")
        if args.experimental_route_sequence_clip:
            command.append("--experimental-route-sequence-clip")
        command.extend(("--route-backward-path-length",
                        str(args.route_backward_path_length)))
        command.extend(("--cruise-safe-distance-margin",
                        str(args.cruise_safe_distance_margin)))
        if args.cruise_stop_to_cruise_speed is not None:
            command.extend(("--cruise-stop-to-cruise-speed",
                            str(args.cruise_stop_to_cruise_speed)))
        if args.cruise_cruise_to_stop_speed is not None:
            command.extend(("--cruise-cruise-to-stop-speed",
                            str(args.cruise_cruise_to_stop_speed)))
        command.extend(("--cruise-outside-predicted-path-count",
                        str(args.cruise_outside_predicted_path_count)))
        command.extend(("--cruise-max-lateral-time-margin",
                        str(args.cruise_max_lateral_time_margin)))
        command.extend(("--cruise-stop-collision-time-margin",
                        str(args.cruise_stop_collision_time_margin)))
        command.extend(("--cruise-outside-stop-predicted-path-count",
                        str(args.cruise_outside_stop_predicted_path_count)))
        command.extend(("--cruise-outside-stop-lateral-time",
                        str(args.cruise_outside_stop_lateral_time)))
        command.extend(("--cruise-pedestrian-deceleration",
                        str(args.cruise_pedestrian_deceleration)))
        command.extend(("--cruise-bicycle-deceleration",
                        str(args.cruise_bicycle_deceleration)))
        try:
            completed = subprocess.run(
                command,
                input="\n".join(rows) + "\n", text=True,
                capture_output=True, check=True)
        except subprocess.CalledProcessError as error:
            if error.stderr:
                print(error.stderr, file=sys.stderr)
            raise
        if (args.diagnose_behavior_velocity or args.diagnose_frame_index) and completed.stderr:
            print(completed.stderr)
    output = iter(completed.stdout.splitlines())
    summaries = []
    selected_outputs = []
    smoothed_outputs = []
    validated_outputs = []
    behavior_outputs = []
    route_reference_outputs = []
    route_goal_refined_outputs = []
    elastic_outputs = []
    optimized_outputs = []
    for index in range(len(chosen)):
        raw_fields = next(output).split()
        if len(raw_fields) != 30:
            raise ValueError("종단 출력 필드 수가 다릅니다")
        fields = (list(map(int, raw_fields[:18])) +
                  [float(raw_fields[18]), int(raw_fields[19])] +
                  list(map(float, raw_fields[20:23])) +
                  list(map(int, raw_fields[23:30])))
        if fields[0] != index:
            raise ValueError("종단 출력 프레임 순서가 다릅니다")
        points = [tuple(map(float, next(output).split()))
                  for _ in range(fields[2])]
        motion_points = [tuple(map(float, next(output).split()))
                         for _ in range(fields[5])]
        final_points = [tuple(map(float, next(output).split()))
                        for _ in range(fields[6])]
        if args.from_route:
            if len(fields) != 30:
                raise ValueError("최종 단계 출력 필드가 없습니다")
            selected_outputs.append([tuple(map(float, next(output).split()))
                                     for _ in range(fields[10])])
            smoothed_outputs.append([tuple(map(float, next(output).split()))
                                     for _ in range(fields[11])])
            validated_outputs.append([tuple(map(float, next(output).split()))
                                      for _ in range(fields[12])])
            behavior_outputs.append([tuple(map(float, next(output).split()))
                                     for _ in range(fields[14])])
            elastic_outputs.append([tuple(map(float, next(output).split()))
                                    for _ in range(fields[3])])
            optimized_outputs.append([tuple(map(float, next(output).split()))
                                      for _ in range(fields[4])])
            route_reference_outputs.append([
                tuple(map(float, next(output).split()))
                for _ in range(fields[15])])
            route_goal_refined_outputs.append([
                tuple(map(float, next(output).split()))
                for _ in range(fields[16])])
        summaries.append((fields, points, motion_points, final_points))
    if not args.allow_incomplete_calculation and any(
            fields[1] != 0 or fields[2] < 2 or fields[7] != 1
            for fields, *_ in summaries):
        raise AssertionError("종단 재생에서 계산 또는 최종 검증 단계가 빠졌습니다")
    if (args.expected_complete_frames is not None and
            len(summaries) != args.expected_complete_frames):
        raise AssertionError("종단 재생의 예상 프레임 수가 다릅니다")
    if args.from_route:
        if args.diagnose_route_reference:
            root_match_counts = collections.Counter()
            for index, (_, (receipt, _)) in enumerate(chosen):
                marker_index = root_marker_indices[index]
                if marker_index is None:
                    root_match_counts["대응 표식 없음"] += 1
                    print(f"기준 경로 frame={index}: 대응 표식 없음")
                    continue
                source_points = decode_marker_array(
                    root_markers[marker_index][1])[0]["points"]
                independent = route_reference_outputs[index]
                if len(source_points) != len(independent):
                    root_match_counts["점 수 차이"] += 1
                    print(f"기준 경로 frame={index}: 점 수 독립={len(independent)} "
                          f"원본={len(source_points)}")
                    print(f"  시작 독립={independent[0][:2]} 원본={source_points[0][:2]}; "
                          f"종점 독립={independent[-1][:2]} 원본={source_points[-1][:2]}")
                    first_shift = next((j for j in range(min(len(source_points),
                                                             len(independent)))
                                        if math.hypot(independent[j][0] -
                                                      source_points[j][0],
                                                      independent[j][1] -
                                                      source_points[j][1]) > 0.001), None)
                    if first_shift is not None:
                        print(f"  첫 인덱스 차이={first_shift}: "
                              f"독립={independent[first_shift][:2]} "
                              f"원본={source_points[first_shift][:2]}")
                    for j in (0, 18, 35, 45, 50, 60, 70, 74, 75):
                        if j >= len(source_points):
                            continue
                        p = source_points[j]
                        k = min(range(len(independent)), key=lambda k:
                                math.hypot(independent[k][0]-p[0],
                                           independent[k][1]-p[1]))
                        print(f"  원본[{j}] 가까운 독립[{k}] 차이="
                              f"{math.hypot(independent[k][0]-p[0],independent[k][1]-p[1]):.6f}m")
                    continue
                errors = [math.hypot(a[0]-b[0], a[1]-b[1])
                          for a, b in zip(independent, source_points)]
                first = next((j for j, e in enumerate(errors) if e > 0.001), None)
                root_match_counts[
                    "좌표 일치" if first is None else "좌표 차이"] += 1
                worst = max(range(len(errors)), key=errors.__getitem__)
                print(f"기준 경로 frame={index}: 첫 1mm 차이={first}, "
                      f"최대={errors[worst]:.6f}m@{worst}, "
                      f"독립={independent[worst][:2]}, 원본={source_points[worst][:2]}")
                if index in (args.diagnose_frame_index or []):
                    print(f"  시작 독립={independent[0][:2]} "
                          f"원본={source_points[0][:2]}; "
                          f"종점 독립={independent[-1][:2]} "
                          f"원본={source_points[-1][:2]}")
            print(f"기준 경로 표식 대응: {dict(root_match_counts)}")
        if not args.allow_incomplete_calculation and any(
                fields[10] < 2 or fields[11] < 2 or fields[12] < 2
                for fields, *_ in summaries):
            raise AssertionError("route 독립 재생에서 시나리오·최종 스무더·validator 출력이 빠졌습니다")
        print(f"route 독립 재생 {len(summaries)}프레임: "
              f"현재 코어 완료 {sum(fields[1] == 0 for fields, *_ in summaries)}, "
              f"차로 주행 경로 생성 {sum(fields[6] > 0 for fields, *_ in summaries)}, "
              f"scenario 선택 {sum(fields[10] > 0 for fields, *_ in summaries)}, "
              f"최종 스무더 출력 {sum(fields[11] > 0 for fields, *_ in summaries)}, "
              f"validator 출력 {sum(fields[12] > 0 for fields, *_ in summaries)}")
        print("코어 상태 분포: " + str(dict(collections.Counter(
            fields[1] for fields, *_ in summaries))))
        for frame_index in args.diagnose_frame_index or []:
            if frame_index < 0 or frame_index >= len(chosen):
                raise ValueError(f"진단 프레임 {frame_index}이 범위를 벗어났습니다")
            _, (_, raw) = chosen[frame_index]
            source_message = decode_path(raw, with_lane_ids=True)
            source_path = source_message["points"]
            ego = decode_odometry(odometry[bisect.bisect_right(
                odometry_times, chosen[frame_index][1][0]) - 1][1])
            print(f"프레임 {frame_index}: ego=({ego['x']:.3f}, {ego['y']:.3f}) "
                  f"행동 독립={len(behavior_outputs[frame_index])} "
                  f"원본={len(source_path)} 선택={len(selected_outputs[frame_index])} "
                  f"평활화={len(smoothed_outputs[frame_index])}")
            lane_ids = sorted({lane_id for ids in source_message["lane_ids"]
                               for lane_id in ids})
            print(f"  원본 lane IDs={lane_ids}")
            for label, points in (("행동 독립", behavior_outputs[frame_index]),
                                  ("행동 원본", source_path),
                                  ("선택", selected_outputs[frame_index]),
                                  ("평활화", smoothed_outputs[frame_index])):
                xyz = [(p[0], p[1], p[7] if label == "행동 원본" else p[2])
                       for p in points]
                metrics = planned_metrics_at_ego(xyz, ego["x"], ego["y"])
                print(f"  {label}: 시작={xyz[0] if xyz else None} "
                      f"끝={xyz[-1] if xyz else None} ego거리={metrics[3]}")
        if args.callback_clock == "recorded-behavior-receipt":
            stage_topics = (
                ("Elastic Band", "/planning/scenario_planning/lane_driving/"
                 "motion_planning/path_smoother/path", decode_path),
                ("경로 최적화", "/planning/scenario_planning/lane_driving/"
                 "motion_planning/path_optimizer/trajectory", decode_trajectory),
                ("동작 속도", "/planning/scenario_planning/lane_driving/"
                 "motion_planning/motion_velocity_planner/trajectory", decode_trajectory),
                ("차로 주행", "/planning/scenario_planning/lane_driving/trajectory",
                 decode_trajectory),
                ("시나리오 선택", "/planning/scenario_planning/"
                 "scenario_selector/trajectory", decode_trajectory),
                ("최종 스무더", "/planning/scenario_planning/"
                 "velocity_smoother/trajectory", decode_trajectory),
                ("최종 validator", "/planning/scenario_planning/trajectory",
                 decode_trajectory),
            )
            source_by_stage = {
                label: {decoded["stamp_ns"]: (decoded | {"receipt_ns": stage_receipt})
                        for stage_receipt, raw in messages(args.bag, topic)
                        for decoded in (decoder(raw),)}
                for label, topic, decoder in stage_topics
            }
            if args.diagnose_frame_index:
                first_frame_receipt = chosen[0][1][0]
                print("첫 독립 프레임 전 원본 단계 출력: " + str({
                    label: sum(item["receipt_ns"] < first_frame_receipt
                               for item in records.values())
                    for label, records in source_by_stage.items()}))
            speed_records = [(receipt, decode_path(raw)) for receipt, raw in
                             messages(args.bag, PREFIX + "path")]
            speed_times = [receipt for receipt, _ in speed_records]
            source_final_by_frame = [None] * len(chosen)
            first_differences = collections.Counter()
            first_difference_by_frame = {}
            first_example = None
            first_geometry = None
            stage_stats = collections.defaultdict(lambda: [0, 0, 0])
            stage_numeric_errors = collections.defaultdict(list)
            stage_segments = collections.defaultdict(list)
            final_spatial = []
            behavior_start_errors = []
            worst_behavior_position = None
            worst_final_speed = None
            worst_final_position = None
            compared = 0

            def compare_stage(independent, original):
                if original is None:
                    return None
                points = original["points"]
                if len(independent) != len(points):
                    return ("점 수", len(independent), len(points))
                if not points:
                    return None
                original_xyz = [((p["x"], p["y"], p["velocity"])
                                 if isinstance(p, dict) else
                                 (p[0], p[1], p[7])) for p in points]
                maximum_position = max(math.hypot(a[0] - b[0], a[1] - b[1])
                                       for a, b in zip(independent, original_xyz))
                maximum_speed = max(abs(a[2] - b[2]) for a, b in
                                    zip(independent, original_xyz))
                if maximum_position > 0.001 or maximum_speed > 0.001:
                    return ("수치", round(maximum_position, 6),
                            round(maximum_speed, 6))
                return None

            def spatial_final(independent, original):
                reference = original["points"]
                if len(reference) < 2:
                    return (0, len(independent), 0.0)
                matched = []
                for point in independent:
                    best_distance2 = float("inf")
                    best_speed = 0.0
                    for first, second in zip(reference, reference[1:]):
                        dx = second["x"] - first["x"]
                        dy = second["y"] - first["y"]
                        length2 = dx * dx + dy * dy
                        ratio = (max(0.0, min(1.0,
                            ((point[0] - first["x"]) * dx +
                             (point[1] - first["y"]) * dy) / length2))
                            if length2 > 1.0e-12 else 0.0)
                        distance2 = ((point[0] - first["x"] - ratio * dx) ** 2 +
                                     (point[1] - first["y"] - ratio * dy) ** 2)
                        if distance2 < best_distance2:
                            best_distance2 = distance2
                            best_speed = (first["velocity"] + ratio *
                                (second["velocity"] - first["velocity"]))
                    if best_distance2 <= 0.05 ** 2:
                        matched.append(abs(point[2] - best_speed))
                return (len(matched), len(independent), max(matched, default=0.0))

            for index, (_, (receipt, raw)) in enumerate(chosen):
                speed_index = bisect.bisect_left(speed_times, receipt)
                if speed_index >= len(speed_records) or \
                        speed_times[speed_index] - receipt > 80_000_000:
                    continue
                compared += 1
                source_speed = speed_records[speed_index][1]
                stamp = source_speed["stamp_ns"]
                source_final_by_frame[index] = source_by_stage["최종 validator"].get(stamp)
                if args.assert_behavior_velocity_after_warmup and index > 0:
                    independent_speed = summaries[index][1]
                    original_speed = source_speed["points"]
                    if len(independent_speed) != len(original_speed):
                        raise AssertionError(
                            f"행동 속도 frame={index} 점 수: "
                            f"독립 {len(independent_speed)}, 원본 {len(original_speed)}")
                    maximum_position = max(math.hypot(a[0]-b[0], a[1]-b[1])
                                           for a, b in zip(independent_speed, original_speed))
                    maximum_speed = max(abs(a[2]-b[7])
                                        for a, b in zip(independent_speed, original_speed))
                    if maximum_position > 0.003 or maximum_speed > 0.001:
                        raise AssertionError(
                            f"행동 속도 frame={index} 최대 위치·속도 차이: "
                            f"{maximum_position:.6f}m, {maximum_speed:.6f}m/s")
                if args.diagnose_behavior_velocity:
                    independent_speed = summaries[index][1]
                    original_speed = source_speed["points"]
                    def path_detail(points):
                        if not points:
                            return {"count": 0}
                        xyz = [((p["x"], p["y"], p["velocity"])
                                if isinstance(p, dict) else
                                (p[0], p[1], p[7] if len(p) > 3 else p[2]))
                               for p in points]
                        stop = next((j for j, p in enumerate(xyz)
                                     if abs(p[2]) < 0.01), None)
                        return {"count": len(xyz), "start": xyz[0],
                                "end": xyz[-1], "first_stop": stop,
                                "stop_xy": xyz[stop][:2] if stop is not None else None}
                    print(f"행동 속도 진단 frame={index} "
                          f"독립={path_detail(independent_speed)} "
                          f"원본={path_detail(original_speed)}")
                    if index == 1:
                        xyz = [(p[0], p[1], p[7]) for p in original_speed]
                        if len(independent_speed) == len(xyz):
                            worst = max(range(len(xyz)), key=lambda j:
                                math.hypot(independent_speed[j][0]-xyz[j][0],
                                           independent_speed[j][1]-xyz[j][1]))
                            print(f"  행동 속도 최대 위치 차이 점 {worst}: "
                                  f"독립={independent_speed[worst]} 원본={xyz[worst]}")
                        for label, sequence in (("독립", independent_speed), ("원본", xyz)):
                            arcs = [0.0]
                            for a, b in zip(sequence, sequence[1:]):
                                arcs.append(arcs[-1] + math.hypot(b[0]-a[0], b[1]-a[1]))
                            print(f"  {label} 행동 속도[90:99] " + str([
                                (j, round(arcs[j], 6), round(arcs[j]-round(arcs[j]), 6),
                                 tuple(round(v, 6) for v in sequence[j][:2]))
                                for j in range(90, min(99, len(sequence)))]))
                        missed = []
                        for j, point in enumerate(xyz):
                            closest = min(range(len(independent_speed)), key=lambda k:
                                math.hypot(independent_speed[k][0]-point[0],
                                           independent_speed[k][1]-point[1]))
                            distance = math.hypot(independent_speed[closest][0]-point[0],
                                                  independent_speed[closest][1]-point[1])
                            if distance > 0.005:
                                missed.append((j, closest, round(distance, 6), point))
                        print(f"  원본 행동 속도에서 독립 경로와 5mm 넘게 떨어진 점 "
                              f"{len(missed)}개: {missed[:30]}")
                    if index == 0:
                        source_behavior = decode_path(raw, with_lane_ids=True)["points"]
                        for label, path_points in (("목표점 보정 직후", route_goal_refined_outputs[index]),
                                                   ("출력 재표본화 직후", behavior_outputs[index])):
                            count = min(len(path_points), len(source_behavior))
                            errors = [math.hypot(path_points[j][0]-source_behavior[j][0],
                                                 path_points[j][1]-source_behavior[j][1])
                                      for j in range(count)]
                            print(f"  {label}: 독립 {len(path_points)}점 원본 {len(source_behavior)}점; "
                                  f"첫 1mm 차이 {next((j for j,e in enumerate(errors) if e > 0.001), None)}; "
                                  f"최대 공통 인덱스 차이 {max(errors) if errors else None}")
                        for label, path_points in (("독립 행동경로", behavior_outputs[index]),
                                                   ("원본 행동경로", decode_path(raw, with_lane_ids=True)["points"])):
                            arcs = [0.0]
                            for a, b in zip(path_points, path_points[1:]):
                                arcs.append(arcs[-1] + math.hypot(b[0]-a[0], b[1]-a[1]))
                            extra = [(j, round(s, 4), round(abs(s-round(s)), 4))
                                     for j, s in enumerate(arcs) if abs(s-round(s)) >= 0.01]
                            print(f"  {label} arc 총 {arcs[-1]:.6f}m; 1m 격자와 1cm 이상 다른 원본점 {len(extra)}개; 샘플 {extra[::max(1,len(extra)//12)]}")
                        source_behavior = decode_path(raw, with_lane_ids=True)["points"]
                        for j in (18, 35, 40, 45, 48, 49, 50, 53, 57, 61, 65, 69, 73, 74, 75):
                            a, b = behavior_outputs[index][j], source_behavior[j]
                            print(f"  행동경로[{j}] 차이 {math.hypot(a[0]-b[0],a[1]-b[1]):.6f} "
                                  f"독립={a[:2]} 원본={b[:2]}")
                        xyz = [((p["x"], p["y"], p["velocity"])
                                if isinstance(p, dict) else (p[0], p[1], p[7]))
                               for p in original_speed]
                        for j in (0, 10, 30, 50, 70, 85, 89, 90, 91, 100,
                                  120, 140, 150, 160, 170, len(xyz)-1):
                            if j >= len(xyz):
                                continue
                            nearest = min(range(len(independent_speed)), key=lambda k:
                                          math.hypot(independent_speed[k][0]-xyz[j][0],
                                                     independent_speed[k][1]-xyz[j][1]))
                            print(f"  원본[{j}]={xyz[j]} 가까운 독립[{nearest}]="
                                  f"{independent_speed[nearest]}")
                stages = [
                    ("행동 경로", behavior_outputs[index],
                     decode_path(raw, with_lane_ids=True)),
                    ("행동 속도", summaries[index][1], source_speed),
                    ("Elastic Band", elastic_outputs[index],
                     source_by_stage["Elastic Band"].get(stamp)),
                    ("경로 최적화", optimized_outputs[index],
                     source_by_stage["경로 최적화"].get(stamp)),
                    ("동작 속도", summaries[index][2],
                     source_by_stage["동작 속도"].get(stamp)),
                    ("차로 주행", summaries[index][3],
                     source_by_stage["차로 주행"].get(stamp)),
                    ("시나리오 선택", selected_outputs[index],
                     source_by_stage["시나리오 선택"].get(stamp)),
                    ("최종 스무더", smoothed_outputs[index],
                     source_by_stage["최종 스무더"].get(stamp)),
                    ("최종 validator", validated_outputs[index],
                     source_by_stage["최종 validator"].get(stamp)),
                ]
                if index in (args.diagnose_frame_index or []):
                    ego = decode_odometry(odometry[bisect.bisect_right(
                        odometry_times, receipt) - 1][1])
                    print(f"  순항 후보={summaries[index][0][8]} "
                          f"정지 선택={bool(summaries[index][0][9])}")
                    for stage_name, independent, original in stages:
                        independent_metrics = planned_metrics_at_ego(
                            independent, ego["x"], ego["y"])
                        source_xyz = ([
                            (p[0], p[1], p[7]) if not isinstance(p, dict)
                            else (p["x"], p["y"], p["velocity"])
                            for p in original["points"]] if original else [])
                        original_metrics = planned_metrics_at_ego(
                            source_xyz, ego["x"], ego["y"]) \
                            if original else (None, None, None, None)
                        print(f"  단계 {stage_name}: 점수={len(independent)}/"
                              f"{len(original['points']) if original else None} "
                              f"ego속도={independent_metrics[0]}/{original_metrics[0]} "
                              f"첫정지거리={independent_metrics[2]}/{original_metrics[2]}")
                        if stage_name == "Elastic Band" and source_xyz:
                            stage_receipt = original["receipt_ns"]
                            stage_odom_index = bisect.bisect_right(
                                odometry_times, stage_receipt) - 1
                            if stage_odom_index >= 0:
                                stage_ego = decode_odometry(odometry[stage_odom_index][1])
                                print(f"    Elastic Band 수신 시각 최신 odometry="
                                      f"({stage_ego['x']:.6f}, {stage_ego['y']:.6f}) "
                                      f"수신 차={stage_receipt - odometry_times[stage_odom_index]}ns")
                            def arc_length(points):
                                return sum(math.hypot(b[0] - a[0], b[1] - a[1])
                                           for a, b in zip(points, points[1:]))
                            print(f"    Elastic Band 시작={independent[0][:2]}/{source_xyz[0][:2]} "
                                  f"끝={independent[-1][:2]}/{source_xyz[-1][:2]} "
                                  f"호길이={arc_length(independent):.6f}/"
                                  f"{arc_length(source_xyz):.6f}m")
                            first = next((j for j, (a, b) in enumerate(
                                zip(independent, source_xyz))
                                if math.hypot(a[0]-b[0], a[1]-b[1]) > 0.001), None)
                            if first is not None:
                                print(f"    Elastic Band 첫 1mm 차이[{first}]="
                                      f"{independent[first][:2]}/{source_xyz[first][:2]}")
                if args.diagnose_final_speed:
                    for stage_name, independent, original in stages:
                        if stage_name not in ("시나리오 선택", "최종 스무더") or original is None:
                            continue
                        observed = original["points"]
                        source_velocity = [p["velocity"] for p in observed]
                        independent_velocity = [p[2] for p in independent]
                        first_zero = lambda values: next((j for j, v in enumerate(values)
                            if abs(v) < 0.001), None)
                        print(f"최종 속도 진단 frame={index} {stage_name}: "
                              f"독립/원본 점 수 {len(independent)}/{len(observed)}, "
                              f"첫 정지 {first_zero(independent_velocity)}/{first_zero(source_velocity)}, "
                              f"끝 8점 속도 {independent_velocity[-8:]}/{source_velocity[-8:]}")
                for label, independent, source in stages:
                    if source is None:
                        if args.diagnose_segments:
                            stage_segments[label].append((index, "원본 없음", None))
                        if args.assert_final_speed_after_warmup and index > 0 and label == "최종 스무더":
                            raise AssertionError("최종 평활화 원본 출력 대응이 없습니다")
                        continue
                    stage_stats[label][0] += 1
                    stage_stats[label][1] += len(independent) == len(source["points"])
                    difference = compare_stage(independent, source)
                    stage_stats[label][2] += difference is None
                    if args.diagnose_segments:
                        stage_segments[label].append((
                            index, "일치" if difference is None else difference[0],
                            difference))
                    if args.diagnose_spatial_final and label == "최종 스무더":
                        final_spatial.append((index, *spatial_final(independent, source)))
                    if (args.assert_final_speed_after_warmup and index > 0 and
                            label == "최종 스무더" and
                            len(independent) != len(source["points"])):
                        raise AssertionError("첫 프레임 뒤 최종 평활화 점 수가 원본과 다릅니다")
                    if independent and len(independent) == len(source["points"]):
                        original_xyz = [((p["x"], p["y"], p["velocity"])
                                         if isinstance(p, dict) else
                                         (p[0], p[1], p[7]))
                                        for p in source["points"]]
                        if (args.assert_final_speed_after_warmup and index > 0 and
                                label == "최종 스무더" and
                                max(abs(a[2] - b[2]) for a, b in zip(
                                    independent, original_xyz)) > 0.001):
                            raise AssertionError("첫 프레임 뒤 최종 평활화 속도가 원본과 다릅니다")
                        stage_numeric_errors[label].append((
                            max(math.hypot(a[0] - b[0], a[1] - b[1])
                                for a, b in zip(independent, original_xyz)),
                            max(abs(a[2] - b[2]) for a, b in
                                zip(independent, original_xyz))))
                        if label == "행동 경로":
                            behavior_start_errors.append(math.hypot(
                                independent[0][0] - original_xyz[0][0],
                                independent[0][1] - original_xyz[0][1]))
                            point_index = max(range(len(independent)), key=lambda j:
                                math.hypot(independent[j][0] - original_xyz[j][0],
                                           independent[j][1] - original_xyz[j][1]))
                            error = math.hypot(
                                independent[point_index][0] -
                                original_xyz[point_index][0],
                                independent[point_index][1] -
                                original_xyz[point_index][1])
                            if worst_behavior_position is None or \
                                    error > worst_behavior_position[0]:
                                worst_behavior_position = (
                                    error, index, point_index,
                                    len(independent),
                                    independent[point_index][:2],
                                    original_xyz[point_index][:2])
                        if label == "최종 validator":
                            arc_m = 0.0
                            for point_index, (actual, expected) in enumerate(
                                    zip(independent, original_xyz)):
                                if point_index:
                                    previous = independent[point_index - 1]
                                    arc_m += math.hypot(
                                        actual[0] - previous[0],
                                        actual[1] - previous[1])
                                detail = (index, point_index,
                                          len(independent), arc_m,
                                          actual[:2], actual[2], expected[2],
                                          stamp)
                                speed_error = abs(actual[2] - expected[2])
                                position_error = math.hypot(
                                    actual[0] - expected[0],
                                    actual[1] - expected[1])
                                if worst_final_speed is None or \
                                        speed_error > worst_final_speed[0]:
                                    worst_final_speed = (speed_error, detail)
                                if worst_final_position is None or \
                                        position_error > worst_final_position[0]:
                                    worst_final_position = (position_error,
                                                            detail)
                first = next(((label, difference) for label, independent, source
                              in stages if (difference := compare_stage(
                                  independent, source)) is not None), None)
                label = first[0] if first else "비교 가능 단계 일치"
                first_differences[label] += 1
                first_difference_by_frame[index] = (
                    label, first[1][0] if first else "")
                if first_example is None and first is not None:
                    first_example = (index, stamp, first)
                    source_path = stages[0][2]["points"]
                    independent_path = stages[0][1]
                    if independent_path and source_path:
                        first_geometry = (
                            (independent_path[0][:2], independent_path[-1][:2]),
                            ((source_path[0][0], source_path[0][1]),
                             (source_path[-1][0], source_path[-1][1])))
            print(f"행동 속도 수신/헤더 기준 대응 {compared}프레임; "
                  f"첫 차이 단계 {dict(first_differences)}")
            if args.export_series_csv:
                args.export_series_csv.parent.mkdir(parents=True, exist_ok=True)
                with args.export_series_csv.open("w", newline="") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(("frame", "receipt_ns", "odometry_receipt_ns",
                                     "odometry_stamp_ns", "ego_x", "ego_y",
                                     "measured_speed_mps", "source_stamp_ns",
                                     "source_count", "independent_count",
                                     "source_planned_speed_mps",
                                     "independent_planned_speed_mps",
                                     "source_planned_acceleration_mps2",
                                     "independent_planned_acceleration_mps2",
                                     "source_first_stop_distance_m",
                                     "independent_first_stop_distance_m",
                                     "source_ego_path_offset_m",
                                     "independent_ego_path_offset_m",
                                     "source_cruise_debug_receipt_ns",
                                     "source_cruise_obstacle_distance_raw_m",
                                     "source_cruise_obstacle_distance_filtered_m",
                                     "independent_cruise_candidate_count",
                                     "independent_cruise_obstacle_distance_filtered_m",
                                     "cruise_clock_receipt_ns",
                                     "cruise_odometry_receipt_ns",
                                     "cruise_objects_receipt_ns",
                                     "cruise_objects_stamp_ns",
                                     "cruise_acceleration_receipt_ns",
                                     "cruise_acceleration_mps2",
                                     "source_slowdown_debug_receipt_ns",
                                     "source_slowdown_object_count",
                                     "independent_slowdown_candidate_count",
                                     "independent_slowdown_start_arc_m",
                                     "independent_slowdown_end_arc_m",
                                     "independent_slowdown_target_velocity_mps",
                                     "first_difference_stage",
                                     "first_difference_kind",
                                     "lane_change_approved_input",
                                     "independent_behavior_action_kind",
                                     "independent_lane_change_candidate_count",
                                     "independent_lane_change_first_safe_index",
                                     "independent_lane_change_module_state",
                                     "independent_lane_change_outside_lanes",
                                     "independent_lane_change_collision_count",
                                     "independent_lane_change_other_unsafe_count"))
                    for index, (_, (receipt, _)) in enumerate(chosen):
                        odom_receipt, ego = frame_odometry[index]
                        source = source_final_by_frame[index]
                        source_points = source["points"] if source else []
                        source_metrics = planned_metrics_at_ego(
                            source_points, ego["x"], ego["y"])
                        independent = validated_outputs[index]
                        independent_metrics = planned_metrics_at_ego(
                            independent, ego["x"], ego["y"])
                        source_cruise = None
                        source_slowdown = None
                        input_index = bisect.bisect_left(
                            cruise_input_clock_times, receipt)
                        if input_index < len(cruise_input_clock_times):
                            input_receipt = cruise_input_clock_times[input_index]
                            if input_receipt - receipt < 90_000_000:
                                debug_index = bisect.bisect_left(
                                    cruise_debug_times, input_receipt)
                                if debug_index < len(cruise_debug) and \
                                        cruise_debug[debug_index][0] - \
                                        input_receipt < 80_000_000:
                                    source_cruise = cruise_debug[debug_index]
                                slowdown_index = bisect.bisect_left(
                                    slowdown_debug_times, input_receipt)
                                if slowdown_index < len(slowdown_debug) and \
                                        slowdown_debug[slowdown_index][0] - \
                                        input_receipt < 80_000_000:
                                    source_slowdown = slowdown_debug[slowdown_index]
                        fields = summaries[index][0]
                        cruise_clock, cruise_odom, cruise_objects, \
                            cruise_objects_stamp, cruise_accel_receipt, \
                            cruise_accel_value = cruise_stage_inputs[index]
                        writer.writerow((index, receipt, odom_receipt,
                                         ego["stamp_ns"], ego["x"], ego["y"],
                                         ego["velocity_mps"],
                                         source["stamp_ns"] if source else "",
                                         len(source_points), len(independent),
                                         source_metrics[0], independent_metrics[0],
                                         source_metrics[1], independent_metrics[1],
                                         source_metrics[2], independent_metrics[2],
                                         source_metrics[3], independent_metrics[3],
                                         source_cruise[0] if source_cruise else "",
                                         (source_cruise[1][3] if source_cruise and
                                          len(source_cruise[1]) > 3 else ""),
                                         (source_cruise[1][4] if source_cruise and
                                          len(source_cruise[1]) > 4 else ""),
                                         fields[17],
                                         fields[18] if fields[18] >= 0 else "",
                                         cruise_clock, cruise_odom,
                                         cruise_objects, cruise_objects_stamp,
                                         cruise_accel_receipt, cruise_accel_value,
                                         source_slowdown[0] if source_slowdown else "",
                                         (len(source_slowdown[1]) // 7
                                          if source_slowdown else ""),
                                         fields[19],
                                         fields[20] if fields[20] >= 0 else "",
                                         fields[21] if fields[21] >= 0 else "",
                                         fields[22] if fields[22] >= 0 else "",
                                         *first_difference_by_frame.get(index, ("", "")),
                                         int(lane_change_approvals.get(index, False)),
                                         fields[23], fields[24], fields[25],
                                         fields[26], fields[27], fields[28],
                                         fields[29]))
                print(f"시계열 CSV 저장: {args.export_series_csv}")
            if not args.allow_incomplete_source_chain and (
                    compared != len(summaries) or
                    stage_stats["최종 validator"][0] != compared):
                raise AssertionError("원본 종단 출력의 수신·헤더 시각 대응이 누락됐습니다")
            if args.assert_route_behavior_reference and (
                    stage_stats["행동 경로"][1] != compared or
                    not behavior_start_errors or
                    max(behavior_start_errors) > 0.001 or
                    max(error[1] for error in
                        stage_numeric_errors["행동 경로"]) > 0.001):
                raise AssertionError("원본 행동 경로의 점 수·시작점·속도 대응이 회귀했습니다")
            print("단계별 비교/점 수 일치/점별 1mm·0.001m/s 이내: " +
                  ", ".join(f"{label} {values[0]}/{values[1]}/{values[2]}"
                            for label, values in stage_stats.items()))
            if args.diagnose_segments:
                for label in ("행동 경로", "행동 속도", "시나리오 선택", "최종 스무더"):
                    records = stage_segments[label]
                    runs = []
                    for frame_index, kind, detail in records:
                        if runs and runs[-1][2] == kind and runs[-1][1] + 1 == frame_index:
                            runs[-1] = (runs[-1][0], frame_index, kind, runs[-1][3])
                        else:
                            runs.append((frame_index, frame_index, kind, detail))
                    print(f"구간 진단 {label}: {len(runs)}구간, "
                          f"처음 {runs[:12]}, 마지막 {runs[-6:] if len(runs) > 12 else []}")
            if args.diagnose_spatial_final:
                for begin, end in ((0, 0), (1, 47), (48, 62), (63, len(summaries) - 1)):
                    subset = [item for item in final_spatial
                              if begin <= item[0] <= end]
                    if not subset:
                        continue
                    print(f"공간 속도 진단 {begin}~{end}: 대응 {len(subset)}프레임, "
                          f"5cm 이내 점 {sum(item[1] for item in subset)}/"
                          f"{sum(item[2] for item in subset)}, "
                          f"속도 최대 {max(item[3] for item in subset):.6f}m/s "
                          f"@{max(subset, key=lambda item: item[3])[0]}, "
                          f"0.1m/s 초과 프레임 {sum(item[3] > 0.1 for item in subset)}, "
                          f"1m/s 초과 프레임 {sum(item[3] > 1.0 for item in subset)}")
            if args.assert_final_speed_after_warmup:
                warm_errors = stage_numeric_errors["최종 스무더"][1:]
                print(f"첫 프레임 뒤 최종 속도 최대 차이: "
                      f"{max((item[1] for item in warm_errors), default=0.0):.9f}m/s")
            for label in ("행동 경로", "행동 속도", "차로 주행", "최종 스무더",
                          "최종 validator"):
                errors = stage_numeric_errors[label]
                if errors:
                    print(f"{label} 점 수 일치 프레임의 인덱스별 최대 위치·속도 차이: "
                          f"{max(item[0] for item in errors):.6f}m, "
                          f"{max(item[1] for item in errors):.6f}m/s; "
                          f"프레임별 최대 차이 중앙값 "
                          f"{statistics.median(item[0] for item in errors):.6f}m, "
                          f"{statistics.median(item[1] for item in errors):.6f}m/s")
            print(f"최종 출력 최대 속도 차이 위치: {worst_final_speed}")
            print(f"최종 출력 최대 위치 차이 위치: {worst_final_position}")
            print(f"행동 경로 최대 위치 차이 지점: {worst_behavior_position}")
            if first_example:
                print(f"첫 차이 예시 {first_example}")
                if first_geometry is not None:
                    print(f"첫 행동 경로 시작·끝 독립/원본 {first_geometry}")
            print("행동 경로 원본 메시지는 비교와 콜백 수신 시각에만 사용했습니다. "
                  "정확한 원본 callback ID와 내부 상태 일치는 확인되지 않았습니다.")
            return
        final_records = [(receipt, decode_trajectory(raw))
                         for receipt, raw in messages(
                             args.bag,
                             "/planning/scenario_planning/lane_driving/trajectory")]
        final_times = [receipt for receipt, _ in final_records]
        matched = 0
        count_equal = 0
        point_errors = []
        first_difference = None
        for frame_index, ((_, _, _, final_points), (_, (receipt, _))) in enumerate(
                zip(summaries, chosen)):
            index = bisect.bisect_left(final_times, receipt)
            if index >= len(final_records) or final_times[index] - receipt > 100_000_000:
                continue
            matched += 1
            recorded = final_records[index][1]["points"]
            if len(final_points) == len(recorded):
                count_equal += 1
                error = max(
                    math.hypot(actual[0] - expected["x"],
                               actual[1] - expected["y"])
                    for actual, expected in zip(final_points, recorded))
                point_errors.append(error)
                if first_difference is None and error > 0.001:
                    first_difference = (frame_index, "위치", error)
            elif first_difference is None:
                first_difference = (frame_index, "점 수",
                                    (len(final_points), len(recorded)))
        print(f"수신 시각 이후 100ms 이내 첫 원본 차로 주행 경로: {matched}프레임, "
              f"점 수 일치 {count_equal}프레임" +
              (f", 일치한 점 수의 최대 위치 차이 {max(point_errors):.6f}m"
               if point_errors else ""))
        if first_difference is not None:
            print(f"추정 대응의 첫 차이: {first_difference}")
        print("기록된 행동 경로·평활화·최적화 출력은 계산 입력에 사용하지 않았습니다. "
              "원본 출력의 대응은 수신 시각 추정이며 동일 콜백을 보장하지 않습니다. "
              "시나리오·최종 스무더는 독립 코어에서 계산했습니다.")
        return
    comparisons = []
    point_errors = []
    velocity_errors = []
    decoded_targets = [(receipt, decode_path(raw))
                       for receipt, raw in targets]
    target_times = [receipt for receipt, _ in decoded_targets]
    used_targets = set()
    motion_counts = []
    final_counts = []
    motion_velocity_errors = []
    final_velocity_errors = []
    stage_counts = collections.defaultdict(lambda: [0, 0])
    first_count_difference = collections.Counter()
    complete_stage_reference_count = 0
    stop_activity = collections.Counter()
    for index, (fields, points, motion_points, final_points) in enumerate(summaries):
        receipt = chosen[index][1][0]
        source = decoded_inputs[index]["points"]
        if not source:
            continue
        begin = bisect.bisect_left(target_times, receipt - 20_000_000)
        matches = []
        for target_index in range(begin, len(decoded_targets)):
            target_receipt, target = decoded_targets[target_index]
            if target_receipt > receipt + 100_000_000:
                break
            target_points = target["points"]
            if target_index in used_targets or not target_points:
                continue
            endpoint_error = max(
                math.hypot(a[0] - b[0], a[1] - b[1])
                for a, b in ((source[0], target_points[0]),
                             (source[-1], target_points[-1])))
            if endpoint_error <= 0.001:
                delay_ms = (target_receipt - receipt) * 1e-6
                matches.append((abs(delay_ms - 20.0), target_index))
        if not matches:
            continue
        target_index = min(matches)[1]
        used_targets.add(target_index)
        original = decoded_targets[target_index][1]["points"]
        if args.diagnose_behavior_velocity:
            stop_a = next((j for j, p in enumerate(points)
                           if abs(p[2]) < 0.01), None)
            stop_b = next((j for j, p in enumerate(original)
                           if abs(p[7]) < 0.01), None)
            print(f"행동 속도 입력 분리 frame={index} "
                  f"독립={len(points)}점/정지{stop_a}, "
                  f"원본={len(original)}점/정지{stop_b}")
            if index == 1:
                for j in range(90, min(101, len(points), len(original))):
                    print(f"  행동속도[{j}] 독립={points[j]} "
                          f"원본={(original[j][0],original[j][1],original[j][7])}")
                missing = []
                for j, p in enumerate(original):
                    near = min(range(len(points)), key=lambda k:
                               math.hypot(points[k][0]-p[0], points[k][1]-p[1]))
                    dist = math.hypot(points[near][0]-p[0], points[near][1]-p[1])
                    if dist > 0.05:
                        missing.append((j, near, round(dist, 4), p[:2], p[7]))
                print(f"  원본 대비 독립 근접점 5cm 이상 차이: {missing[:25]}")
                for item in missing[:5]:
                    point = item[3]
                    input_near = min(range(len(source)), key=lambda k:
                                     math.hypot(source[k][0]-point[0],
                                                source[k][1]-point[1]))
                    print(f"  누락점 주변 원본 입력 경로[{input_near}]="
                          f"{source[input_near][:2]}")
        target_stamp = decoded_targets[target_index][1]["stamp_ns"]
        if args.with_cruise_candidates and target_stamp in motion_receipts:
            debug_index = bisect.bisect_left(
                stop_debug_times, motion_receipts[target_stamp])
            if (debug_index < len(stop_debug) and
                    stop_debug_times[debug_index] -
                    motion_receipts[target_stamp] <= 100_000_000):
                original_active = (len(stop_debug[debug_index][1]) >= 4 and
                                   stop_debug[debug_index][1][3] > 0.0)
                independent_active = fields[9] > 0
                stop_activity["matched"] += 1
                stop_activity["original"] += original_active
                stop_activity["independent"] += independent_active
                stop_activity["joint"] += original_active and independent_active
        stage_pairs = (
            ("행동 속도", fields[2], len(original)),
            ("평활화", fields[3],
             len(smooth_targets[target_stamp]["points"])
             if target_stamp in smooth_targets else None),
            ("경로 최적화", fields[4],
             len(optimizer_targets[target_stamp]["points"])
             if target_stamp in optimizer_targets else None),
            ("동작 속도", fields[5],
             len(motion_targets[target_stamp]["points"])
             if target_stamp in motion_targets else None),
            ("최종 차로", fields[6],
             len(final_targets[target_stamp]["points"])
             if target_stamp in final_targets else None),
        )
        for label, independent_count, original_count in stage_pairs:
            if original_count is not None:
                stage_counts[label][0] += 1
                stage_counts[label][1] += independent_count == original_count
        if all(original_count is not None for _, _, original_count in stage_pairs):
            complete_stage_reference_count += 1
            first = next((label for label, independent_count, original_count
                          in stage_pairs if independent_count != original_count),
                         "전 단계 점 수 일치")
            first_count_difference[first] += 1
        for independent, originals, counts, errors in (
                (motion_points, motion_targets, motion_counts,
                 motion_velocity_errors),
                (final_points, final_targets, final_counts,
                 final_velocity_errors)):
            observed = originals.get(target_stamp)
            if observed:
                recorded = observed["points"]
                counts.append(len(independent) == len(recorded))
                if len(independent) == len(recorded):
                    errors.append(max(abs(actual[2] - expected["velocity"])
                                      for actual, expected in
                                      zip(independent, recorded)))
        if points and original:
            error = math.hypot(points[-1][0] - original[-1][0],
                               points[-1][1] - original[-1][1])
            comparisons.append((len(points) == len(original), error))
            if len(points) == len(original):
                point_errors.append(max(
                    math.hypot(actual[0] - expected[0],
                               actual[1] - expected[1])
                    for actual, expected in zip(points, original)))
                velocity_errors.append(max(
                    abs(actual[2] - expected[7])
                    for actual, expected in zip(points, original)))
    print(f"종단 재생 {len(summaries)}프레임, 완결 "
          f"{sum(row[0][1] == 0 for row in summaries)}프레임, "
          f"행동 속도 출력 {sum(row[0][2] > 0 for row in summaries)}프레임")
    if args.with_cruise_candidates:
        print(f"순항 정지 후보 {sum(fields[8] for fields, *_ in summaries)}개, "
              f"정지점 선택 {sum(fields[9] for fields, *_ in summaries)}프레임")
        if stop_activity["matched"]:
            print(f"원본 정지 디버그 추정 대응 {stop_activity['matched']}프레임, "
                  f"원본 활성 {stop_activity['original']}, 독립 활성 "
                  f"{stop_activity['independent']}, 동시 활성 "
                  f"{stop_activity['joint']}")
    for label, values in ages.items():
        if values:
            print(f"{label} 수신 시각 차이 중앙값 "
                  f"{statistics.median(values):.1f}ms, 최대 {max(values):.1f}ms")
    if comparisons:
        print(f"원본 행동 속도 시각·끝점 추정 짝 {len(comparisons)}프레임, "
              f"점 수 일치 {sum(match for match, _ in comparisons)}프레임, "
              f"종점 위치 차이 최대 {max(error for _, error in comparisons):.6f}m")
        if point_errors:
            print(f"점 수 일치 경로의 인덱스 위치 차이 최대 "
                  f"{max(point_errors):.6f}m, 속도 차이 최대 "
                  f"{max(velocity_errors):.6f}m/s")
    for label, counts, errors in (("동작 속도", motion_counts,
                                   motion_velocity_errors),
                                  ("최종 차로", final_counts,
                                   final_velocity_errors)):
        if counts:
            print(f"원본 {label} 공통 시각 {len(counts)}프레임, "
                  f"점 수 일치 {sum(counts)}프레임" +
                  (f", 점 수 일치 최대 속도 차이 {max(errors):.6f}m/s"
                   if errors else ""))
    for label in ("평활화", "경로 최적화"):
        compared, matched = stage_counts[label]
        if compared:
            print(f"원본 {label} 공통 시각 {compared}프레임, "
                  f"점 수 일치 {matched}프레임")
    if complete_stage_reference_count:
        ordered = ("행동 속도", "평활화", "경로 최적화",
                   "동작 속도", "최종 차로", "전 단계 점 수 일치")
        print(f"전 단계 원본 출력이 있는 {complete_stage_reference_count}프레임의 "
              "첫 점 수 차이: " + ", ".join(
                  f"{label} {first_count_difference[label]}" for label in ordered))
    print("원본 내부 콜백 ID·모듈 상태·실행 매개변수는 이 재생으로 복원되지 않습니다")


if __name__ == "__main__":
    main()
