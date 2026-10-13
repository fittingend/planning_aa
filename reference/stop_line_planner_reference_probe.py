"""원본 정지선·신호등·횡단보도·보행로 플러그인의 출력점을 수집합니다."""

import argparse
import copy
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import deserialize_message
from autoware_map_msgs.msg import LaneletMapBin
from autoware_internal_debug_msgs.msg import Float64MultiArrayStamped, StringStamped
from autoware_perception_msgs.msg import (
    PredictedObjects, PredictedPath, TrafficLightElement, TrafficLightGroup,
    TrafficLightGroupArray,
)
from autoware_planning_msgs.msg import Path as PlanningPath
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tier4_planning_msgs.msg import PathWithLaneId
from visualization_msgs.msg import MarkerArray

from scenario_selector_reference_probe import load, spin_for


def record(points):
    return [{
        "x": point.pose.position.x,
        "y": point.pose.position.y,
        "z": point.pose.position.z,
        "qx": point.pose.orientation.x,
        "qy": point.pose.orientation.y,
        "qz": point.pose.orientation.z,
        "qw": point.pose.orientation.w,
        "velocity": point.longitudinal_velocity_mps,
        "lateral_velocity": point.lateral_velocity_mps,
        "heading_rate": point.heading_rate_rps,
    } for point in points]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sample-fraction", type=float, default=0.25)
    parser.add_argument("--path-start-index", type=int, default=0)
    parser.add_argument("--path-end-index", type=int)
    parser.add_argument("--synthetic-path-stop-index", type=int)
    parser.add_argument("--ego-from-path-start", action="store_true")
    parser.add_argument("--ego-from-path-index", type=int)
    parser.add_argument("--ego-velocity", type=float)
    parser.add_argument("--module", choices=("stop_line", "traffic_light", "crosswalk", "walkway", "intersection", "detection_area", "speed_bump", "no_drivable_lane", "no_stopping_area", "merge_from_private"),
                        default="stop_line")
    parser.add_argument("--modules", help="쉼표로 구분한 원본 플러그인 실행 순서")
    parser.add_argument("--minimum-outputs", type=int, default=5)
    parser.add_argument("--stop-line", nargs=4, type=float,
                        metavar=("X1", "Y1", "X2", "Y2"))
    parser.add_argument("--front-offset", type=float)
    parser.add_argument("--source-osm-regulatory-id", type=int)
    parser.add_argument("--synthetic-regulatory-id", type=int)
    parser.add_argument("--map-bin-file", type=Path)
    parser.add_argument("--pedestrian-signal-color", choices=("red", "yellow", "green"))
    parser.add_argument("--pedestrian-signal-id", type=int, default=15921)
    parser.add_argument("--empty-objects", action="store_true")
    parser.add_argument("--detailed-objects", action="store_true")
    parser.add_argument("--object-id-prefix")
    parser.add_argument("--synthetic-point", nargs=3, type=float,
                        metavar=("X", "Y", "Z"))
    parser.add_argument("--synthetic-point-frames", type=int)
    parser.add_argument("--object-shift-y", type=float, default=0.0)
    parser.add_argument("--only-selected-object", action="store_true")
    parser.add_argument("--enable-occlusion", action="store_true")
    parser.add_argument("--occupancy-fill-unknown", action="store_true")
    parser.add_argument("--occupancy-fill-free", action="store_true")
    parser.add_argument("--occupancy-unknown-frames", type=int)
    parser.add_argument("--signal-yellow-after-frames", type=int)
    parser.add_argument("--occupancy-unknown-patch", nargs=3, type=float,
                        metavar=("X", "Y", "SIZE"))
    parser.add_argument("--occupancy-grid-origin", nargs=2, type=float,
                        metavar=("X", "Y"))
    parser.add_argument("--occlusion-mask-object", "--synthetic-car",
                        nargs=3, type=float,
                        metavar=("X", "Y", "SPEED"))
    parser.add_argument("--synthetic-car-yaw", type=float, default=0.0)
    parser.add_argument("--synthetic-car-prediction-duration", type=float)
    parser.add_argument("--synthetic-car-label", type=int, default=1)
    parser.add_argument("--synthetic-car-confidence", type=float, default=1.0)
    parser.add_argument("--synthetic-car-path-y-shift", type=float, default=0.0)
    parser.add_argument("--synthetic-second-path-confidence", type=float)
    parser.add_argument("--synthetic-object-frames", type=int)
    parser.add_argument("--crosswalk-enable-rtc", action="store_true")
    parser.add_argument("--enable-stuck-in-intersection", action="store_true")
    parser.add_argument("--marker-output", type=Path,
                        help="교차로 원본 디버그 표식의 좌표를 별도 파일에 기록합니다")
    parser.add_argument("--replace-lane-id", nargs=2, type=int,
                        metavar=("기존ID", "대체ID"),
                        help="진단용으로 입력 경로의 차로 ID를 치환합니다")
    args = parser.parse_args()
    timestamp, path_bytes = load(
        args.bag,
        "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id",
        args.sample_fraction,
    )
    if args.map_bin_file:
        map_bytes = args.map_bin_file.read_bytes()
    else:
        _, map_bytes = load(args.bag, "/map/vector_map")
    topics = (
        ("/localization/kinematic_state", Odometry),
        ("/localization/acceleration", AccelWithCovarianceStamped),
        ("/perception/object_recognition/objects", PredictedObjects),
        ("/perception/occupancy_grid_map/map", OccupancyGrid),
    )
    messages = []
    for topic, kind in topics:
        _, data = load(args.bag, topic, near=timestamp)
        messages.append(deserialize_message(data, kind))
    if args.occupancy_grid_origin:
        messages[3].info.origin.position.x = args.occupancy_grid_origin[0]
        messages[3].info.origin.position.y = args.occupancy_grid_origin[1]
    if args.occupancy_fill_unknown:
        messages[3].data = [50] * len(messages[3].data)
    if args.occupancy_fill_free:
        messages[3].data = [0] * len(messages[3].data)
    if args.occupancy_unknown_frames is not None:
        if args.occupancy_unknown_frames < 0:
            raise ValueError("미확인 격자 프레임 수는 음수가 될 수 없습니다")
        messages[3].data = [50] * len(messages[3].data)
        free_grid = copy.deepcopy(messages[3])
        free_grid.data = [0] * len(free_grid.data)
    if args.occupancy_unknown_patch:
        grid = messages[3]
        x, y, size = args.occupancy_unknown_patch
        if size <= 0:
            raise ValueError("미확인 격자 영역의 크기는 양수여야 합니다")
        grid.data = [0] * len(grid.data)
        yaw = 2.0 * math.atan2(grid.info.origin.orientation.z,
                               grid.info.origin.orientation.w)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        for row in range(grid.info.height):
            for column in range(grid.info.width):
                local_x = (column + 0.5) * grid.info.resolution
                local_y = (row + 0.5) * grid.info.resolution
                world_x = grid.info.origin.position.x + cosine * local_x - sine * local_y
                world_y = grid.info.origin.position.y + sine * local_x + cosine * local_y
                if abs(world_x - x) <= size / 2 and abs(world_y - y) <= size / 2:
                    grid.data[row * grid.info.width + column] = 50
    if args.empty_objects:
        messages[2] = PredictedObjects()
        messages[2].header.frame_id = "map"
    elif args.object_id_prefix:
        selected = [obj for obj in messages[2].objects
                    if bytes(obj.object_id.uuid).hex().startswith(args.object_id_prefix)]
        if not selected:
            raise RuntimeError("지정한 객체 ID를 찾지 못했습니다")
        for obj in selected:
            obj.kinematics.initial_pose_with_covariance.pose.position.y += args.object_shift_y
            for predicted in obj.kinematics.predicted_paths:
                for pose in predicted.path:
                    pose.position.y += args.object_shift_y
        if args.only_selected_object:
            messages[2].objects = selected
    if args.occlusion_mask_object:
        if not messages[2].objects:
            raise RuntimeError("가림 차폐 객체의 형식으로 사용할 실측 객체가 없습니다")
        x, y, speed = args.occlusion_mask_object
        obj = copy.deepcopy(messages[2].objects[0])
        obj.classification[0].label = args.synthetic_car_label
        pose = obj.kinematics.initial_pose_with_covariance.pose
        pose.position.x, pose.position.y = x, y
        pose.orientation.x = pose.orientation.y = 0.0
        pose.orientation.z = math.sin(args.synthetic_car_yaw / 2.0)
        pose.orientation.w = math.cos(args.synthetic_car_yaw / 2.0)
        obj.kinematics.initial_twist_with_covariance.twist.linear.x = speed
        obj.kinematics.initial_twist_with_covariance.twist.linear.y = 0.0
        obj.shape.dimensions.x = obj.shape.dimensions.y = 3.0
        obj.kinematics.predicted_paths = []
        if args.synthetic_car_prediction_duration is not None:
            prediction = PredictedPath()
            prediction.confidence = args.synthetic_car_confidence
            prediction.time_step.sec = 0
            prediction.time_step.nanosec = 500000000
            for index in range(int(args.synthetic_car_prediction_duration / 0.5) + 1):
                predicted_pose = copy.deepcopy(pose)
                distance = speed * 0.5 * index
                predicted_pose.position.x += math.cos(args.synthetic_car_yaw) * distance
                predicted_pose.position.y += (
                    math.sin(args.synthetic_car_yaw) * distance +
                    args.synthetic_car_path_y_shift)
                prediction.path.append(predicted_pose)
            obj.kinematics.predicted_paths = [prediction]
            if args.synthetic_second_path_confidence is not None:
                second = copy.deepcopy(prediction)
                second.confidence = args.synthetic_second_path_confidence
                for predicted_pose in second.path:
                    predicted_pose.position.y -= args.synthetic_car_path_y_shift
                obj.kinematics.predicted_paths.append(second)
        messages[2].objects = [obj]
    path = deserialize_message(path_bytes, PathWithLaneId)
    if args.replace_lane_id is not None:
        old_id, new_id = args.replace_lane_id
        for point in path.points:
            point.lane_ids = [new_id if lane_id == old_id else lane_id
                              for lane_id in point.lane_ids]
    if args.synthetic_path_stop_index is not None:
        if not 0 <= args.synthetic_path_stop_index < len(path.points):
            raise ValueError("합성 경로 정지점 인덱스가 범위를 벗어났습니다")
        path.points[args.synthetic_path_stop_index].point.longitudinal_velocity_mps = 0.0
    if args.path_end_index is not None:
        if args.path_end_index < 1 or args.path_end_index >= len(path.points):
            raise ValueError("경로 끝 인덱스가 범위를 벗어났습니다")
        path.points = path.points[:args.path_end_index + 1]
    if args.ego_from_path_index is not None:
        if args.ego_from_path_index < 0 or args.ego_from_path_index >= len(path.points):
            raise ValueError("차량 위치로 사용할 경로 인덱스가 범위를 벗어났습니다")
        messages[0].pose.pose = copy.deepcopy(
            path.points[args.ego_from_path_index].point.pose)
    if args.path_start_index < 0 or args.path_start_index >= len(path.points) - 1:
        raise ValueError("경로 시작 인덱스가 범위를 벗어났습니다")
    path.points = path.points[args.path_start_index:]
    if args.ego_from_path_start:
        messages[0].pose.pose.position = copy.deepcopy(
            path.points[0].point.pose.position)
        messages[0].pose.pose.orientation = copy.deepcopy(
            path.points[0].point.pose.orientation)
    if args.ego_velocity is not None:
        messages[0].twist.twist.linear.x = args.ego_velocity
    lanelet_map = deserialize_message(map_bytes, LaneletMapBin)
    pointcloud = PointCloud2()
    pointcloud.header.frame_id = "map"
    empty_pointcloud = point_cloud2.create_cloud_xyz32(
        Header(frame_id="map"), [(2500.0, 23000.0, 0.0)])
    if args.synthetic_point:
        pointcloud = point_cloud2.create_cloud_xyz32(
            Header(frame_id="map"), [tuple(args.synthetic_point)])

    log_path = args.output.with_suffix(".log")
    with log_path.open("w") as log:
        plugin_names = {
            "stop_line": "autoware::behavior_velocity_planner::StopLineModulePlugin",
            "traffic_light": "autoware::behavior_velocity_planner::TrafficLightModulePlugin",
            "crosswalk": "autoware::behavior_velocity_planner::CrosswalkModulePlugin",
            "walkway": "autoware::behavior_velocity_planner::WalkwayModulePlugin",
            "intersection": "autoware::behavior_velocity_planner::IntersectionModulePlugin",
            "detection_area": "autoware::behavior_velocity_planner::DetectionAreaModulePlugin",
            "speed_bump": "autoware::behavior_velocity_planner::SpeedBumpModulePlugin",
            "no_drivable_lane": "autoware::behavior_velocity_planner::NoDrivableLaneModulePlugin",
            "no_stopping_area": "autoware::behavior_velocity_planner::NoStoppingAreaModulePlugin",
            "merge_from_private": "autoware::behavior_velocity_planner::MergeFromPrivateModulePlugin",
        }
        selected_modules = (args.modules.split(",") if args.modules else [args.module])
        if not selected_modules or any(name not in plugin_names for name in selected_modules):
            raise ValueError("알 수 없는 계획 플러그인이 지정되었습니다")
        plugin = "[" + ", ".join(plugin_names[name] for name in selected_modules) + "]"
        process = subprocess.Popen(
            ["ros2", "launch", "/probe/reference/stop_line_planner_probe.launch.xml",
             f"module:={plugin}",
             f"occlusion_enable:={str(args.enable_occlusion).lower()}",
             f"crosswalk_enable_rtc:={str(args.crosswalk_enable_rtc).lower()}",
             f"stuck_in_intersection:={str(args.enable_stuck_in_intersection).lower()}",
             f"publish_debug_path:={str(bool(args.marker_output)).lower()}",
             f"ttc_debug_lane:={25001 if args.module == 'intersection' else 0}"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        rclpy.init()
        node = Node("stop_line_planner_reference_probe")
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publishers = [
            node.create_publisher(LaneletMapBin, "/probe/map", map_qos),
            node.create_publisher(Odometry, "/probe/odometry", 10),
            node.create_publisher(AccelWithCovarianceStamped, "/probe/acceleration", 10),
            node.create_publisher(PredictedObjects, "/probe/objects", 10),
            node.create_publisher(OccupancyGrid, "/probe/occupancy", 10),
            node.create_publisher(PointCloud2, "/probe/pointcloud", 10),
        ]
        signal_pub = node.create_publisher(
            TrafficLightGroupArray, "/probe/traffic_signals", 10)
        signal_message = None
        if args.pedestrian_signal_color:
            signal_message = TrafficLightGroupArray()
            group = TrafficLightGroup()
            group.traffic_light_group_id = args.pedestrian_signal_id
            element = TrafficLightElement()
            element.color = {
                "red": TrafficLightElement.RED,
                "yellow": TrafficLightElement.AMBER,
                "green": TrafficLightElement.GREEN,
            }[args.pedestrian_signal_color]
            element.shape = TrafficLightElement.CIRCLE
            group.elements = [element]
            signal_message.traffic_light_groups = [group]
        path_pub = node.create_publisher(PathWithLaneId, "/probe/input_path", 10)
        received = []
        collision_info = []
        intersection_decisions = []
        intersection_decision_times = []
        intersection_ego_ttc = []
        intersection_markers = []
        intersection_debug_paths = []
        intersection_frames = []
        detection_frames = []
        no_drivable_frames = []
        no_stopping_frames = []
        no_stopping_missing_outputs = 0
        node.create_subscription(PlanningPath, "/probe/output_path", received.append, 10)
        if args.module == "intersection":
            if args.marker_output:
                node.create_subscription(
                    MarkerArray,
                    "/probe/behavior_velocity_planner/debug/intersection",
                    intersection_markers.append, 10,
                )
                node.create_subscription(
                    PathWithLaneId,
                    "/probe/behavior_velocity_planner/debug/path_with_lane_id/intersection",
                    intersection_debug_paths.append, 10,
                )
            node.create_subscription(
                String,
                "/probe/behavior_velocity_planner/debug/intersection/decision_state",
                lambda message: (intersection_decisions.append(message),
                                 intersection_decision_times.append(time.monotonic())), 10,
            )
            node.create_subscription(
                Float64MultiArrayStamped,
                "/probe/behavior_velocity_planner/debug/intersection/ego_ttc",
                intersection_ego_ttc.append, 10,
            )
        if args.module == "crosswalk":
            node.create_subscription(
                StringStamped,
                "/probe/behavior_velocity_planner/debug/collision_info",
                collision_info.append, 10,
            )
        try:
            spin_for(node, 3.0)
            empty_objects = PredictedObjects()
            empty_objects.header.frame_id = "map"
            for frame in range(30):
                output_count_before_frame = len(received)
                frame_messages = list(messages)
                if (args.synthetic_object_frames is not None and
                        frame >= args.synthetic_object_frames):
                    frame_messages[2] = empty_objects
                if (args.occupancy_unknown_frames is not None and
                        frame >= args.occupancy_unknown_frames):
                    frame_messages[3] = free_grid
                active_pointcloud = (
                    empty_pointcloud if args.synthetic_point_frames is not None and
                    frame >= args.synthetic_point_frames else pointcloud)
                for publisher, message in zip(
                    publishers, (lanelet_map, *frame_messages, active_pointcloud)
                ):
                    publisher.publish(message)
                if (signal_message is not None and
                        args.signal_yellow_after_frames is not None and
                        frame >= args.signal_yellow_after_frames):
                    signal_message.traffic_light_groups[0].elements[0].color = (
                        TrafficLightElement.AMBER)
                if signal_message is not None:
                    signal_message.stamp = node.get_clock().now().to_msg()
                    signal_pub.publish(signal_message)
                spin_for(node, 0.1)
                path_pub.publish(path)
                spin_for(node, 0.2)
                if args.module == "no_stopping_area":
                    no_stopping_missing_outputs = (
                        no_stopping_missing_outputs + 1
                        if len(received) == output_count_before_frame else 0)
                    if no_stopping_missing_outputs >= 3:
                        raise RuntimeError(
                            "정차 금지 구역 원본 출력이 연속 세 프레임 중단되었습니다")
                if (args.occupancy_unknown_frames is not None or
                        args.signal_yellow_after_frames is not None):
                    intersection_frames.append({
                        "time_s": time.monotonic(),
                        "unknown": (frame < args.occupancy_unknown_frames
                                    if args.occupancy_unknown_frames is not None
                                    else args.occupancy_fill_unknown),
                        "signal_color": ("yellow" if
                                         args.signal_yellow_after_frames is not None and
                                         frame >= args.signal_yellow_after_frames else
                                         args.pedestrian_signal_color),
                        "decision": intersection_decisions[-1].data
                        if intersection_decisions else "",
                        "output": record(received[-1].points)
                        if received else [],
                    })
                if args.synthetic_point_frames is not None:
                    detection_frames.append({
                        "time_s": time.monotonic(),
                        "has_point": frame < args.synthetic_point_frames,
                        "output": record(received[-1].points)
                        if received else [],
                    })
                if args.module == "no_drivable_lane":
                    no_drivable_frames.append({
                        "frame": frame,
                        "output": record(received[-1].points)
                        if received else [],
                    })
                if args.module == "no_stopping_area" and (
                        args.synthetic_object_frames is not None):
                    no_stopping_frames.append({
                        "time_s": time.monotonic(),
                        "has_vehicle": frame < args.synthetic_object_frames,
                        "output": record(received[-1].points)
                        if received else [],
                    })
                if len(received) >= args.minimum_outputs:
                    break
            if not received:
                raise RuntimeError("원본 정지선 planner 출력을 받지 못했습니다")
            if args.marker_output:
                marker_values = [{
                    "namespace": marker.ns,
                    "id": marker.id,
                    "type": marker.type,
                    "position": [marker.pose.position.x,
                                 marker.pose.position.y],
                    "points": [[point.x, point.y] for point in marker.points],
                } for batch in intersection_markers
                  for marker in batch.markers
                  if marker.ns in ("first_attention_area",
                                   "second_attention_area",
                                   "parked_targets",
                                   "occlusion_polygons",
                                   "stuck_vehicle_detect_area",
                                   "nearest_occlusion_projection")]
                args.marker_output.write_text(json.dumps({
                    "markers": marker_values,
                    "debug_paths": [
                        {"points": record([point.point for point in path.points]),
                         "lane_ids": [list(point.lane_ids) for point in path.points]}
                        for path in intersection_debug_paths],
                }, ensure_ascii=False, indent=2))
            result = {
                "bag_timestamp": timestamp,
                "modules": selected_modules,
                "ego": {
                    "x": messages[0].pose.pose.position.x,
                    "y": messages[0].pose.pose.position.y,
                    "velocity_mps": messages[0].twist.twist.linear.x,
                    "acceleration_mps2": messages[1].accel.accel.linear.x,
                },
                "input": record([point.point for point in path.points]),
                "input_lane_ids": [list(point.lane_ids) for point in path.points],
                "output": record(received[-1].points),
            }
            if args.module == "no_stopping_area":
                result["received_output_count"] = len(received)
            if args.map_bin_file:
                result["map_bin_file"] = args.map_bin_file.name
            if args.synthetic_path_stop_index is not None:
                result["synthetic_path_stop_index"] = args.synthetic_path_stop_index
            if args.synthetic_point:
                result["synthetic_point"] = dict(zip(
                    ("x", "y", "z"), args.synthetic_point))
            if detection_frames:
                first_time = detection_frames[0]["time_s"]
                for frame in detection_frames:
                    frame["time_s"] -= first_time
                result["detection_frames"] = detection_frames
            if no_stopping_frames:
                first_time = no_stopping_frames[0]["time_s"]
                for frame in no_stopping_frames:
                    frame["time_s"] -= first_time
                result["no_stopping_frames"] = no_stopping_frames
            if no_drivable_frames:
                result["no_drivable_frames"] = no_drivable_frames
            if args.module == "intersection":
                result["intersection_decisions"] = [
                    message.data for message in intersection_decisions]
                if signal_message is not None:
                    result["intersection_signal"] = {
                        "regulatory_id": args.pedestrian_signal_id,
                        "color": args.pedestrian_signal_color,
                        "shape": "circle",
                    }
                if intersection_decision_times:
                    first_decision_time = intersection_decision_times[0]
                    result["intersection_decision_times_s"] = [
                        stamp - first_decision_time
                        for stamp in intersection_decision_times]
                if intersection_ego_ttc:
                    latest_ttc = intersection_ego_ttc[-1]
                    result["intersection_ego_ttc"] = {
                        "shape": [dimension.size for dimension in latest_ttc.layout.dim],
                        "data": list(latest_ttc.data),
                    }
                if intersection_frames:
                    first_time = intersection_frames[0]["time_s"]
                    for frame in intersection_frames:
                        frame["time_s"] -= first_time
                    result["intersection_frames"] = intersection_frames
                if args.enable_occlusion:
                    grid = messages[3]
                    result["intersection_occupancy"] = {
                        "width": grid.info.width,
                        "height": grid.info.height,
                        "resolution": grid.info.resolution,
                        "origin_x": grid.info.origin.position.x,
                        "origin_y": grid.info.origin.position.y,
                        "origin_qz": grid.info.origin.orientation.z,
                        "origin_qw": grid.info.origin.orientation.w,
                        "fill_unknown": args.occupancy_fill_unknown,
                        "fill_free": args.occupancy_fill_free,
                        "unknown_patch": args.occupancy_unknown_patch,
                        "unknown_frames": args.occupancy_unknown_frames,
                    }
                if args.occlusion_mask_object:
                    result["synthetic_car"] = {
                        "x": args.occlusion_mask_object[0],
                        "y": args.occlusion_mask_object[1],
                        "speed_mps": args.occlusion_mask_object[2],
                        "yaw_rad": args.synthetic_car_yaw,
                        "prediction_duration_s": args.synthetic_car_prediction_duration,
                        "label": args.synthetic_car_label,
                        "confidence": args.synthetic_car_confidence,
                        "path_y_shift": args.synthetic_car_path_y_shift,
                        "second_path_confidence": args.synthetic_second_path_confidence,
                        "object_frames": args.synthetic_object_frames,
                    }
            if args.module == "no_stopping_area" and args.occlusion_mask_object:
                result["synthetic_car"] = {
                    "x": args.occlusion_mask_object[0],
                    "y": args.occlusion_mask_object[1],
                    "speed_mps": args.occlusion_mask_object[2],
                    "label": args.synthetic_car_label,
                    "yaw_rad": args.synthetic_car_yaw,
                    "length_m": 3.0,
                    "width_m": 3.0,
                }
            if args.stop_line is not None:
                result["stop_line"] = dict(zip(
                    ("x1", "y1", "x2", "y2"), args.stop_line))
            if args.front_offset is not None:
                result["front_offset_m"] = args.front_offset
            if args.source_osm_regulatory_id is not None and "stop_line" in result:
                result["stop_line"]["source_osm_regulatory_id"] = (
                    args.source_osm_regulatory_id)
            if args.synthetic_regulatory_id is not None and "stop_line" in result:
                result["stop_line"]["synthetic_regulatory_id"] = (
                    args.synthetic_regulatory_id)
            if args.module == "crosswalk":
                if signal_message is not None:
                    result["pedestrian_signal"] = {
                        "regulatory_id": args.pedestrian_signal_id,
                        "color": args.pedestrian_signal_color,
                        "shape": "circle",
                    }
                result["collision_info"] = [message.data for message in collision_info]
                grid = messages[3]
                result["occupancy_grid"] = {
                    "width": grid.info.width,
                    "height": grid.info.height,
                    "resolution": grid.info.resolution,
                    "origin_x": grid.info.origin.position.x,
                    "origin_y": grid.info.origin.position.y,
                    "origin_qz": grid.info.origin.orientation.z,
                    "origin_qw": grid.info.origin.orientation.w,
                    "fill_unknown": args.occupancy_fill_unknown,
                    "occlusion_enabled": args.enable_occlusion,
                    "crosswalk_enable_rtc": args.crosswalk_enable_rtc,
                    "stuck_in_intersection": args.enable_stuck_in_intersection,
                    "unknown_patch": args.occupancy_unknown_patch,
                    "mask_object": args.occlusion_mask_object,
                }
                if args.object_id_prefix:
                    result["object_adjustment"] = {
                        "id_prefix": args.object_id_prefix,
                        "shift_y_m": args.object_shift_y,
                        "only_selected_object": args.only_selected_object,
                    }
                result["objects"] = [
                    {
                        "label": obj.classification[0].label
                        if obj.classification else None,
                        "x": obj.kinematics.initial_pose_with_covariance.pose.position.x,
                        "y": obj.kinematics.initial_pose_with_covariance.pose.position.y,
                        "velocity_x": obj.kinematics.initial_twist_with_covariance.twist.linear.x,
                        "velocity_y": obj.kinematics.initial_twist_with_covariance.twist.linear.y,
                        "length": obj.shape.dimensions.x,
                        "width": obj.shape.dimensions.y,
                        "predicted_paths": len(obj.kinematics.predicted_paths),
                        **({
                            "id": bytes(obj.object_id.uuid).hex(),
                            "paths": [
                                {
                                    "confidence": predicted.confidence,
                                    "poses": [
                                        {
                                            "x": pose.position.x,
                                            "y": pose.position.y,
                                            "qz": pose.orientation.z,
                                            "qw": pose.orientation.w,
                                        }
                                        for pose in predicted.path
                                    ],
                                }
                                for predicted in obj.kinematics.predicted_paths
                            ],
                        } if args.detailed_objects else {}),
                    }
                    for obj in messages[2].objects
                ]
            args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
            print(json.dumps({"input": len(result["input"]),
                              "output": len(result["output"])}), flush=True)
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
            node.destroy_node()
            rclpy.shutdown()
    if "The scene plugin" in log_path.read_text():
        raise RuntimeError("원본 계획 플러그인을 로드하지 못했습니다")


if __name__ == "__main__":
    main()
