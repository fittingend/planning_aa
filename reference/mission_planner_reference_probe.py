"""원본 목적지 플래너의 체크포인트 경로를 rosbag 지도와 비교용으로 기록합니다."""

import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import deserialize_message
from autoware_map_msgs.msg import LaneletMapBin
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_planning_msgs.msg import LaneletRoute
from nav_msgs.msg import Odometry
from tier4_planning_msgs.msg import PathWithLaneId, RouteState
from tier4_planning_msgs.srv import ClearRoute, SetWaypointRoute

from stop_line_planner_reference_probe import load


def spin_for(node, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sample-fraction", type=float, default=0.25)
    parser.add_argument("--goal-path-index", type=int, default=50)
    parser.add_argument("--waypoint-path-index", type=int, action="append", default=[])
    parser.add_argument("--waypoint-offset-x", type=float, default=0.0)
    parser.add_argument("--waypoint-offset-y", type=float, default=0.0)
    parser.add_argument("--start-offset-x", type=float, default=0.0)
    parser.add_argument("--start-offset-y", type=float, default=0.0)
    parser.add_argument("--start-yaw-offset-deg", type=float, default=0.0)
    parser.add_argument("--goal-yaw-offset-deg", type=float, default=0.0)
    parser.add_argument("--goal-z-offset", type=float, default=0.0)
    parser.add_argument("--goal-offset-x", type=float, default=0.0)
    parser.add_argument("--goal-offset-y", type=float, default=0.0)
    parser.add_argument("--allow-failure", action="store_true")
    parser.add_argument("--map-bin-file", type=Path)
    parser.add_argument("--consider-no-drivable-lanes", action="store_true")
    parser.add_argument("--clear-after-set", action="store_true")
    parser.add_argument("--reject-after-set", action="store_true")
    parser.add_argument("--publish-stop-mode", action="store_true")
    args = parser.parse_args()
    timestamp, path_bytes = load(
        args.bag,
        "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id",
        args.sample_fraction)
    if args.map_bin_file:
        map_bytes = args.map_bin_file.read_bytes()
    else:
        _, map_bytes = load(args.bag, "/map/vector_map")
    _, odom_bytes = load(args.bag, "/localization/kinematic_state", near=timestamp)
    path = deserialize_message(path_bytes, PathWithLaneId)
    lanelet_map = deserialize_message(map_bytes, LaneletMapBin)
    odometry = deserialize_message(odom_bytes, Odometry)
    odometry.pose.pose.position.x += args.start_offset_x
    odometry.pose.pose.position.y += args.start_offset_y
    if args.start_yaw_offset_deg:
        orientation = odometry.pose.pose.orientation
        yaw = 2 * math.atan2(orientation.z, orientation.w)
        yaw += math.radians(args.start_yaw_offset_deg)
        orientation.x = orientation.y = 0.0
        orientation.z = math.sin(yaw / 2)
        orientation.w = math.cos(yaw / 2)
    if not 0 <= args.goal_path_index < len(path.points):
        raise ValueError("목표 경로 인덱스가 범위를 벗어났습니다")

    log_path = args.output.with_suffix(".log")
    with log_path.open("w") as log:
        process = subprocess.Popen(
            ["ros2", "launch", "/probe/reference/mission_planner_probe.launch.xml",
             f"consider_no_drivable_lanes:={str(args.consider_no_drivable_lanes).lower()}"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Node("mission_planner_reference_probe")
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        map_pub = node.create_publisher(LaneletMapBin, "/probe/map", durable)
        odom_pub = node.create_publisher(Odometry, "/probe/odometry", 10)
        mode_pub = node.create_publisher(
            OperationModeState,
            "/probe/mission_planner/input/operation_mode_state", 10)
        stop_mode = OperationModeState()
        stop_mode.mode = OperationModeState.STOP
        routes = []
        states = []
        node.create_subscription(LaneletRoute, "/probe/route", routes.append, durable)
        node.create_subscription(RouteState, "/probe/mission_planner/state",
                                 states.append, durable)
        client = node.create_client(SetWaypointRoute, "/probe/set_waypoint_route")
        clear_client = node.create_client(ClearRoute, "/probe/clear_route")
        try:
            for _ in range(50):
                map_pub.publish(lanelet_map)
                odom_pub.publish(odometry)
                if args.publish_stop_mode:
                    mode_pub.publish(stop_mode)
                spin_for(node, 0.2)
                if client.service_is_ready() and states and states[-1].state == RouteState.UNSET:
                    break
            if not client.service_is_ready() or not states or states[-1].state != RouteState.UNSET:
                raise RuntimeError("원본 목적지 경로 서비스가 준비되지 않았습니다")
            request = SetWaypointRoute.Request()
            request.header.frame_id = "map"
            request.header.stamp = node.get_clock().now().to_msg()
            request.goal_pose = path.points[args.goal_path_index].point.pose
            request.goal_pose.position.x += args.goal_offset_x
            request.goal_pose.position.y += args.goal_offset_y
            request.goal_pose.position.z += args.goal_z_offset
            if args.goal_yaw_offset_deg:
                orientation = request.goal_pose.orientation
                yaw = 2 * math.atan2(orientation.z, orientation.w)
                yaw += math.radians(args.goal_yaw_offset_deg)
                orientation.x = orientation.y = 0.0
                orientation.z = math.sin(yaw / 2)
                orientation.w = math.cos(yaw / 2)
            for index in args.waypoint_path_index:
                if not 0 <= index < len(path.points):
                    raise ValueError("경유점 경로 인덱스가 범위를 벗어났습니다")
                waypoint = path.points[index].point.pose
                waypoint.position.x += args.waypoint_offset_x
                waypoint.position.y += args.waypoint_offset_y
                request.waypoints.append(waypoint)
            request.allow_modification = True
            future = client.call_async(request)
            deadline = time.monotonic() + 20.0
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not future.done():
                raise RuntimeError("원본 목적지 경로 응답 시간이 초과되었습니다")
            response = future.result()
            if response is None:
                raise RuntimeError("원본 목적지 경로 응답을 읽지 못했습니다")
            if not response.status.success and not args.allow_failure:
                raise RuntimeError(f"원본 목적지 경로 요청 실패: {response}")
            spin_for(node, 1.0)
            if not routes and response.status.success:
                raise RuntimeError("원본 목적지 경로를 수신하지 못했습니다")
            route = routes[-1] if routes else LaneletRoute()
            state_after_set = [int(state.state) for state in states]
            route_count_after_set = len(routes)
            rejected_status = None
            if args.reject_after_set and response.status.success:
                if args.publish_stop_mode:
                    for _ in range(5):
                        mode_pub.publish(stop_mode)
                        spin_for(node, 0.1)
                rejected_request = copy.deepcopy(request)
                rejected_request.header.stamp = node.get_clock().now().to_msg()
                rejected_request.goal_pose.position.x += 10000.0
                rejected_future = client.call_async(rejected_request)
                deadline = time.monotonic() + 10.0
                while not rejected_future.done() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                if not rejected_future.done() or rejected_future.result() is None:
                    raise RuntimeError("원본 잘못된 목표 요청의 응답을 받지 못했습니다")
                rejected_response = rejected_future.result()
                spin_for(node, 0.5)
                rejected_status = {
                    "success": bool(rejected_response.status.success),
                    "error_code": int(rejected_response.status.code),
                    "states": [int(state.state) for state in states],
                    "route_count": len(routes),
                    "route_goal_x": routes[-1].goal_pose.position.x
                        if routes else None,
                }
            clear_status = None
            if args.clear_after_set and response.status.success:
                if not clear_client.wait_for_service(timeout_sec=3.0):
                    raise RuntimeError("원본 경로 해제 서비스가 준비되지 않았습니다")
                clear_future = clear_client.call_async(ClearRoute.Request())
                deadline = time.monotonic() + 10.0
                while not clear_future.done() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                if not clear_future.done() or clear_future.result() is None:
                    raise RuntimeError("원본 경로 해제 응답을 받지 못했습니다")
                clear_response = clear_future.result()
                spin_for(node, 0.5)
                clear_status = {
                    "success": bool(clear_response.status.success),
                    "error_code": int(clear_response.status.code),
                    "states": [int(state.state) for state in states],
                    "route_count": len(routes),
                }
            result = {
                "bag_timestamp": timestamp,
                "start": {
                    "x": odometry.pose.pose.position.x,
                    "y": odometry.pose.pose.position.y,
                    "yaw_qz": odometry.pose.pose.orientation.z,
                    "yaw_qw": odometry.pose.pose.orientation.w,
                },
                "goal": {
                    "x": request.goal_pose.position.x,
                    "y": request.goal_pose.position.y,
                    "z": request.goal_pose.position.z,
                    "yaw_qz": request.goal_pose.orientation.z,
                    "yaw_qw": request.goal_pose.orientation.w,
                },
                "goal_path_index": args.goal_path_index,
                "waypoints": [{
                    "x": waypoint.position.x,
                    "y": waypoint.position.y,
                    "yaw_qz": waypoint.orientation.z,
                    "yaw_qw": waypoint.orientation.w,
                } for waypoint in request.waypoints],
                "segments": [{
                    "preferred": section.preferred_primitive.id,
                    "primitives": [primitive.id for primitive in section.primitives],
                } for section in route.segments],
                "goal_height_m": route.goal_pose.position.z if response.status.success else None,
                "success": bool(response.status.success),
                "error_code": int(response.status.code),
                "consider_no_drivable_lanes": args.consider_no_drivable_lanes,
            }
            if args.clear_after_set:
                result["lifecycle"] = {
                    "state_after_set": state_after_set,
                    "route_count_after_set": route_count_after_set,
                    "clear": clear_status,
                }
            if args.reject_after_set:
                result["rejected_after_set"] = rejected_status
            args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
            print(json.dumps({"segments": len(result["segments"])}), flush=True)
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
