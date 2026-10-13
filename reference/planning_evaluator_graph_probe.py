"""Observe current-image planning evaluator output type, QoS, and cadence."""

import argparse
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
from rosidl_runtime_py.utilities import get_message
from autoware_map_msgs.msg import LaneletMapBin
from autoware_planning_msgs.msg import LaneletRoute, PoseWithUuidStamped, Trajectory, TrajectoryPoint
from autoware_perception_msgs.msg import PredictedObject, PredictedObjects
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry

from stop_line_planner_reference_probe import load


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--synthetic-trajectory", action="store_true")
    parser.add_argument("--second-shift-y", type=float, default=0.0)
    parser.add_argument("--synthetic-acceleration", action="store_true")
    parser.add_argument("--synthetic-modified-goal", action="store_true")
    parser.add_argument("--synthetic-object-x", type=float)
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--bag-ego-offset-x", type=float, default=0.0)
    parser.add_argument("--bag-ego-offset-y", type=float, default=0.0)
    args = parser.parse_args()
    bag_map = bag_route = bag_odometry = None
    if args.bag:
        route_stamp, route_bytes = load(args.bag, "/planning/mission_planning/route")
        _, map_bytes = load(args.bag, "/map/vector_map")
        _, odometry_bytes = load(args.bag, "/localization/kinematic_state", near=route_stamp)
        bag_map = deserialize_message(map_bytes, LaneletMapBin)
        bag_route = deserialize_message(route_bytes, LaneletRoute)
        bag_odometry = deserialize_message(odometry_bytes, Odometry)
        bag_odometry.pose.pose.position.x += args.bag_ego_offset_x
        bag_odometry.pose.pose.position.y += args.bag_ego_offset_y
    with args.output.with_suffix(".log").open("w") as log:
        process = subprocess.Popen(
            ["ros2", "launch", "/probe/reference/planning_evaluator_graph_probe.launch.xml"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Node("planning_evaluator_graph_probe")
        trajectory_pub = node.create_publisher(
            Trajectory, "/probe/planning_evaluator/input/trajectory", 10)
        odometry_pub = node.create_publisher(
            Odometry, "/probe/planning_evaluator/input/odometry", 10)
        acceleration_pub = node.create_publisher(
            AccelWithCovarianceStamped,
            "/probe/planning_evaluator/input/acceleration", 10)
        modified_goal_pub = node.create_publisher(
            PoseWithUuidStamped,
            "/probe/planning_evaluator/input/modified_goal", 10)
        objects_pub = node.create_publisher(
            PredictedObjects,
            "/probe/planning_evaluator/input/objects", 10)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        map_pub = node.create_publisher(
            LaneletMapBin, "/probe/planning_evaluator/input/vector_map", durable)
        route_pub = node.create_publisher(
            LaneletRoute, "/probe/planning_evaluator/input/route", durable)
        try:
            topic = "/probe/planning_evaluator/metrics"
            deadline = time.monotonic() + 10.0
            graph_types = []
            while time.monotonic() < deadline:
                graph_types = dict(node.get_topic_names_and_types()).get(topic, [])
                if graph_types:
                    break
                rclpy.spin_once(node, timeout_sec=0.05)
            if len(graph_types) != 1:
                raise RuntimeError("원본 평가 출력 토픽을 찾지 못했습니다")
            metric_type = get_message(graph_types[0])
            receipts = []
            message_sizes = []
            nonempty_metrics = []
            first_two_nonempty = []
            shifted_metrics = []
            kinematic_metrics = []
            modified_goal_metrics = []
            obstacle_metrics = []
            lane_metrics = []
            shifted = False
            def receive(msg):
                receipts.append(time.monotonic_ns())
                if hasattr(msg, "metric_array"):
                    message_sizes.append(len(msg.metric_array))
                    if msg.metric_array and not nonempty_metrics:
                        nonempty_metrics.extend(
                            [metric.name, metric.value]
                            for metric in msg.metric_array)
                    if msg.metric_array and len(first_two_nonempty) < 2:
                        first_two_nonempty.append([
                            [metric.name, metric.value]
                            for metric in msg.metric_array])
                    if shifted and msg.metric_array and not shifted_metrics:
                        shifted_metrics.extend([
                            [metric.name, metric.value]
                            for metric in msg.metric_array])
                    if not kinematic_metrics and any(
                            metric.name == "kinematic_state/vel"
                            for metric in msg.metric_array):
                        kinematic_metrics.extend([
                            [metric.name, metric.value]
                            for metric in msg.metric_array
                            if metric.name.startswith("kinematic_state/")])
                    if not modified_goal_metrics and any(
                            metric.name == "modified_goal_longitudinal_deviation/min"
                            for metric in msg.metric_array):
                        modified_goal_metrics.extend([
                            [metric.name, metric.value]
                            for metric in msg.metric_array
                            if metric.name.startswith("modified_goal_")])
                    if not obstacle_metrics and any(
                            metric.name == "obstacle_ttc/min"
                            for metric in msg.metric_array):
                        obstacle_metrics.extend([
                            [metric.name, metric.value]
                            for metric in msg.metric_array
                            if metric.name.startswith("obstacle_")])
                    if not lane_metrics and any(
                            metric.name == "ego_lane_info/lane_id"
                            for metric in msg.metric_array):
                        lane_metrics.extend([
                            [metric.name, metric.value]
                            for metric in msg.metric_array
                            if metric.name.startswith("ego_lane_info/")])
                elif hasattr(msg, "status"):
                    message_sizes.append(len(msg.status))
            node.create_subscription(metric_type, topic, receive, 10)
            started = time.monotonic()
            deadline = started + 1.2
            while time.monotonic() < deadline:
                if bag_route:
                    map_pub.publish(bag_map)
                    route_pub.publish(bag_route)
                    odometry_pub.publish(bag_odometry)
                if args.synthetic_trajectory:
                    shifted = (args.second_shift_y != 0.0 and
                               time.monotonic() - started >= 0.6)
                    stamp = node.get_clock().now().to_msg()
                    trajectory = Trajectory()
                    trajectory.header.frame_id = "map"
                    trajectory.header.stamp = stamp
                    for x, velocity in ((0.0, 1.0), (1.0, 1.0), (2.0, 0.0)):
                        point = TrajectoryPoint()
                        point.pose.position.x = x
                        point.pose.position.y = args.second_shift_y if shifted else 0.0
                        point.pose.orientation.w = 1.0
                        point.longitudinal_velocity_mps = velocity
                        trajectory.points.append(point)
                    odometry = Odometry()
                    odometry.header.frame_id = "map"
                    odometry.header.stamp = stamp
                    odometry.pose.pose.orientation.w = 1.0
                    if args.synthetic_acceleration:
                        odometry.twist.twist.linear.x = 2.0
                    trajectory_pub.publish(trajectory)
                    odometry_pub.publish(odometry)
                    if args.synthetic_acceleration:
                        acceleration = AccelWithCovarianceStamped()
                        acceleration.header.frame_id = "base_link"
                        acceleration.header.stamp = stamp
                        acceleration.accel.accel.linear.x = 0.5
                        acceleration_pub.publish(acceleration)
                    if args.synthetic_modified_goal:
                        modified_goal = PoseWithUuidStamped()
                        modified_goal.header.frame_id = "map"
                        modified_goal.header.stamp = stamp
                        modified_goal.pose.position.x = 3.0
                        modified_goal.pose.position.y = 4.0
                        modified_goal.pose.orientation.w = 1.0
                        modified_goal_pub.publish(modified_goal)
                    if args.synthetic_object_x is not None:
                        objects = PredictedObjects()
                        objects.header.frame_id = "map"
                        objects.header.stamp = stamp
                        obj = PredictedObject()
                        obj.kinematics.initial_pose_with_covariance.pose.position.x = (
                            args.synthetic_object_x)
                        obj.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0
                        objects.objects.append(obj)
                        objects_pub.publish(objects)
                rclpy.spin_once(node, timeout_sec=0.05)
            publishers = [{
                "durability": str(info.qos_profile.durability),
                "reliability": str(info.qos_profile.reliability),
                "depth": info.qos_profile.depth,
            } for info in node.get_publishers_info_by_topic(topic)]
            debug_topic = "/probe/planning_evaluator/debug/processing_time_ms"
            debug_graph = {
                "types": dict(node.get_topic_names_and_types()).get(debug_topic, []),
                "publishers": [{
                    "durability": str(info.qos_profile.durability),
                    "reliability": str(info.qos_profile.reliability),
                } for info in node.get_publishers_info_by_topic(debug_topic)],
            }
            result = {
                "metrics_topic": topic,
                "types": graph_types,
                "publishers": publishers,
                "message_count": len(receipts),
                "message_sizes": message_sizes,
                "synthetic_trajectory": args.synthetic_trajectory,
                "first_nonempty_metrics": nonempty_metrics,
                "first_two_nonempty_metrics": first_two_nonempty,
                "second_shift_y": args.second_shift_y,
                "first_shifted_metrics": shifted_metrics,
                "synthetic_acceleration": args.synthetic_acceleration,
                "first_kinematic_metrics": kinematic_metrics,
                "synthetic_modified_goal": args.synthetic_modified_goal,
                "first_modified_goal_metrics": modified_goal_metrics,
                "synthetic_object_x": args.synthetic_object_x,
                "first_obstacle_metrics": obstacle_metrics,
                "bag": str(args.bag) if args.bag else None,
                "bag_ego_xy": ([bag_odometry.pose.pose.position.x,
                                bag_odometry.pose.pose.position.y]
                               if bag_odometry else None),
                "bag_ego_offset_xy": [args.bag_ego_offset_x,
                                      args.bag_ego_offset_y],
                "bag_route_primitive_ids": ([
                    [primitive.id for primitive in section.primitives]
                    for section in bag_route.segments] if bag_route else None),
                "first_lane_metrics": lane_metrics,
                "intervals_ms": [(b - a) / 1e6
                                 for a, b in zip(receipts, receipts[1:])],
                "processing_time_graph": debug_graph,
            }
            args.output.write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps({key: result[key] for key in
                              ("types", "publishers", "message_count", "message_sizes")}))
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
