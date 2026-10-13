"""Run the original route selector main/MRM lifecycle on one K-City route."""

import argparse
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import deserialize_message
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_map_msgs.msg import LaneletMapBin
from autoware_planning_msgs.msg import LaneletRoute
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tier4_planning_msgs.msg import PathWithLaneId, RouteState
from tier4_planning_msgs.srv import ClearRoute, SetWaypointRoute

from stop_line_planner_reference_probe import load


def spin_until(node, predicate, timeout=15.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not predicate():
        raise RuntimeError("route selector 원본 노드 응답 시간이 초과되었습니다")


def call(node, client, request, events, name):
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"{name} 서비스가 준비되지 않았습니다")
    future = client.call_async(request)
    spin_until(node, future.done)
    response = future.result()
    if response is None:
        raise RuntimeError(f"{name} 응답을 읽지 못했습니다")
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    return {
        "name": name,
        "success": bool(response.status.success),
        "code": int(response.status.code),
        "events_after": len(events),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--goal-path-index", type=int, default=50)
    parser.add_argument("--mrm-goal-path-index", type=int, default=45)
    parser.add_argument("--replace-main-during-mrm", type=int)
    parser.add_argument("--invalid-mrm-first", action="store_true")
    parser.add_argument("--invalid-main-after-set", action="store_true")
    parser.add_argument("--independent-goal-echo", action="store_true")
    args = parser.parse_args()

    stamp, path_bytes = load(
        args.bag,
        "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id",
        0.25)
    _, map_bytes = load(args.bag, "/map/vector_map")
    _, odom_bytes = load(args.bag, "/localization/kinematic_state", near=stamp)
    path = deserialize_message(path_bytes, PathWithLaneId)
    lanelet_map = deserialize_message(map_bytes, LaneletMapBin)
    odometry = deserialize_message(odom_bytes, Odometry)

    log_path = args.output.with_suffix(".log")
    with log_path.open("w") as log:
        launch_name = ("route_selector_independent_echo_probe.launch.xml"
                       if args.independent_goal_echo else
                       "route_selector_probe.launch.xml")
        process = subprocess.Popen(
            ["ros2", "launch", "/probe/reference/" + launch_name],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        echo_process = None
        if args.independent_goal_echo:
            echo_process = subprocess.Popen(
                ["python3", "/probe/ros/goal_pose_visualizer.py",
                 "--route-topic", "/probe/mission_planner/route",
                 "--goal-topic", "/probe/echo_back_goal_pose"],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            rclpy.init()
            node = Node("route_selector_reference_probe")
        except BaseException:
            if echo_process is not None:
                os.killpg(echo_process.pid, signal.SIGINT)
                echo_process.wait(timeout=8)
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=8)
            if rclpy.ok():
                rclpy.shutdown()
            raise
        durable = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        map_pub = node.create_publisher(LaneletMapBin, "/probe/map", durable)
        odom_pub = node.create_publisher(Odometry, "/probe/odometry", 10)
        mode_pub = node.create_publisher(OperationModeState, "/probe/operation_mode", 10)
        events = []

        def pose_values(pose):
            position = pose.position
            orientation = pose.orientation
            return [position.x, position.y, position.z,
                    orientation.x, orientation.y, orientation.z, orientation.w]

        def goal_callback(msg):
            events.append({
                "channel": "goal_echo", "kind": "pose",
                "frame_id": msg.header.frame_id,
                "stamp_ns": int(msg.header.stamp.sec) * 1000000000 +
                            int(msg.header.stamp.nanosec),
                "goal": [msg.pose.position.x, msg.pose.position.y],
                "goal_pose": pose_values(msg.pose),
            })

        node.create_subscription(PoseStamped, "/probe/echo_back_goal_pose",
                                 goal_callback, durable)

        def state_callback(channel):
            def receive(msg):
                events.append({"channel": channel, "kind": "state", "value": int(msg.state)})
            return receive

        def route_callback(channel):
            def receive(msg):
                events.append({
                    "channel": channel, "kind": "route",
                    "uuid": [int(value) for value in msg.uuid.uuid],
                    "segments": [int(part.preferred_primitive.id) for part in msg.segments],
                    "goal": [msg.goal_pose.position.x, msg.goal_pose.position.y],
                    "goal_pose": pose_values(msg.goal_pose),
                    "frame_id": msg.header.frame_id,
                    "stamp_ns": int(msg.header.stamp.sec) * 1000000000 +
                                int(msg.header.stamp.nanosec),
                })
            return receive

        for channel in ("main", "mrm", "planner"):
            base = ("/probe/route_selector/" + channel if channel != "planner"
                    else "/probe/mission_planner")
            node.create_subscription(RouteState, base + "/state",
                                     state_callback(channel), durable)
            node.create_subscription(LaneletRoute, base + "/route",
                                     route_callback(channel), durable)

        selector = "/probe/route_selector/"
        main_client = node.create_client(SetWaypointRoute, selector + "main/set_waypoint_route")
        mrm_client = node.create_client(SetWaypointRoute, selector + "mrm/set_waypoint_route")
        mrm_clear = node.create_client(ClearRoute, selector + "mrm/clear_route")
        mode = OperationModeState()
        mode.mode = OperationModeState.STOP
        try:
            for _ in range(60):
                map_pub.publish(lanelet_map)
                odom_pub.publish(odometry)
                mode_pub.publish(mode)
                for _ in range(4):
                    rclpy.spin_once(node, timeout_sec=0.05)
                ready = all(any(event["channel"] == channel and
                                event["kind"] == "state" and event["value"] == RouteState.UNSET
                                for event in events) for channel in ("main", "mrm", "planner"))
                if ready and main_client.service_is_ready():
                    break
            if not ready:
                raise RuntimeError("원본 main/MRM 초기 상태가 UNSET이 되지 않았습니다")

            def route_request(index):
                request = SetWaypointRoute.Request()
                request.header.frame_id = "map"
                request.header.stamp = node.get_clock().now().to_msg()
                request.goal_pose = copy.deepcopy(path.points[index].point.pose)
                request.allow_modification = True
                return request

            operations = []
            operations.append(call(node, main_client, route_request(args.goal_path_index),
                                   events, "main/set_waypoint_route"))
            if args.invalid_main_after_set:
                invalid_main = route_request(args.goal_path_index)
                invalid_main.goal_pose.position.x += 10000.0
                operations.append(call(node, main_client, invalid_main,
                                       events, "main/set_waypoint_route_invalid"))
            elif args.invalid_mrm_first:
                invalid_mrm = route_request(args.mrm_goal_path_index)
                invalid_mrm.goal_pose.position.x += 10000.0
                operations.append(call(node, mrm_client, invalid_mrm,
                                       events, "mrm/set_waypoint_route_invalid"))
            if not args.invalid_main_after_set:
                operations.append(call(node, mrm_client, route_request(args.mrm_goal_path_index),
                                       events, "mrm/set_waypoint_route"))
                if args.replace_main_during_mrm is not None:
                    operations.append(call(node, main_client,
                                           route_request(args.replace_main_during_mrm),
                                           events, "main/set_waypoint_route_during_mrm"))
                operations.append(call(node, mrm_clear, ClearRoute.Request(), events,
                                       "mrm/clear_route"))
            result = {
                "goal_echo_implementation": (
                    "planning-aa" if args.independent_goal_echo else "original"),
                "bag_timestamp": stamp,
                "goal_path_index": args.goal_path_index,
                "mrm_goal_path_index": args.mrm_goal_path_index,
                "replace_main_during_mrm": args.replace_main_during_mrm,
                "invalid_mrm_first": args.invalid_mrm_first,
                "invalid_main_after_set": args.invalid_main_after_set,
                "operations": operations,
                "events": events,
            }
            planned_topics = {
                "/probe/mission_planner/route", "/probe/mission_planner/state",
                "/probe/route_selector/main/route", "/probe/route_selector/main/state",
                "/probe/route_selector/mrm/route", "/probe/route_selector/mrm/state",
                "/probe/echo_back_goal_pose",
            }
            result["topic_graph"] = {
                name: {
                    "types": types,
                    "publisher_durability": [
                        str(info.qos_profile.durability)
                        for info in node.get_publishers_info_by_topic(name)],
                    "publisher_reliability": [
                        str(info.qos_profile.reliability)
                        for info in node.get_publishers_info_by_topic(name)],
                    "publisher_depth": [
                        int(info.qos_profile.depth)
                        for info in node.get_publishers_info_by_topic(name)],
                }
                for name, types in node.get_topic_names_and_types()
                if name in planned_topics
            }
            result["route_services"] = {
                name: types for name, types in node.get_service_names_and_types()
                if name.startswith(("/probe/mission_planner/",
                                    "/probe/route_selector/")) and
                name.rsplit("/", 1)[-1] in
                {"clear_route", "set_waypoint_route", "set_lanelet_route"}
            }
            if len(result["topic_graph"]) != 7 or len(result["route_services"]) != 9:
                raise AssertionError("원본 route/state/echo graph 또는 9개 경로 서비스 누락")
            planner_routes = [event for event in events
                              if event["channel"] == "planner" and
                              event["kind"] == "route"]
            goal_echoes = [event for event in events
                           if event["channel"] == "goal_echo"]
            if len(planner_routes) != len(goal_echoes) or any(
                    route["goal"] != echo["goal"] or
                    route["goal_pose"] != echo["goal_pose"] or
                    route["frame_id"] != echo["frame_id"] or
                    route["stamp_ns"] != echo["stamp_ns"]
                    for route, echo in zip(planner_routes, goal_echoes)):
                raise AssertionError("원본 goal pose echo가 route 헤더·목표와 다릅니다")
            args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
            print(json.dumps({"successes": [op["success"] for op in operations],
                              "events": len(events)}), flush=True)
        finally:
            if echo_process is not None:
                os.killpg(echo_process.pid, signal.SIGINT)
                try:
                    echo_process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(echo_process.pid, signal.SIGKILL)
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
