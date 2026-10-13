"""Drive the original freespace planner on a synthetic free parking grid."""

import argparse
import copy
import gzip
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
from autoware_planning_msgs.msg import LaneletRoute, Trajectory
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool
from tier4_planning_msgs.msg import Scenario

from stop_line_planner_reference_probe import load


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--goal-forward-m", type=float, default=5.0)
    parser.add_argument("--duration-s", type=float, default=15.0)
    parser.add_argument("--observe-seconds", type=float, default=0.0)
    parser.add_argument("--complete-after-plan", action="store_true")
    parser.add_argument("--advance-first-segment", action="store_true")
    parser.add_argument("--obstacle-forward-m", type=float)
    parser.add_argument("--obstacle-half-length-m", type=float, default=0.5)
    parser.add_argument("--obstacle-half-width-m", type=float, default=0.5)
    parser.add_argument("--grid-reference", type=Path,
                        help="costmap generator 원본 JSON(.gz)의 occupancy 출력 사용")
    parser.add_argument("--no-adapt-expansion", action="store_true")
    args = parser.parse_args()
    _, odom_bytes = load(args.bag, "/localization/kinematic_state")
    original_odometry = deserialize_message(odom_bytes, Odometry)
    odometry = copy.deepcopy(original_odometry)
    odometry.header.frame_id = "map"
    odometry.twist.twist.linear.x = 0.0
    odometry.twist.twist.linear.y = 0.0
    odometry.twist.twist.angular.z = 0.0
    yaw = 2.0 * math.atan2(odometry.pose.pose.orientation.z,
                            odometry.pose.pose.orientation.w)
    ego_x = odometry.pose.pose.position.x
    ego_y = odometry.pose.pose.position.y
    goal_x = ego_x + args.goal_forward_m * math.cos(yaw)
    goal_y = ego_y + args.goal_forward_m * math.sin(yaw)
    route = LaneletRoute()
    route.header.frame_id = "map"
    route.start_pose = copy.deepcopy(odometry.pose.pose)
    route.goal_pose = copy.deepcopy(odometry.pose.pose)
    route.goal_pose.position.x = goal_x
    route.goal_pose.position.y = goal_y
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.resolution = 0.3
    grid.info.width = 233
    grid.info.height = 233
    grid.info.origin.position.x = ego_x - 34.95
    grid.info.origin.position.y = ego_y - 34.95
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * (grid.info.width * grid.info.height)
    if args.grid_reference:
        opener = gzip.open if args.grid_reference.suffix == ".gz" else open
        with opener(args.grid_reference, "rt") as stream:
            generated = json.load(stream)["output"]
        grid.info.resolution = generated["resolution_m"]
        grid.info.width = generated["width"]
        grid.info.height = generated["height"]
        grid.info.origin.position.x = generated["origin_x"]
        grid.info.origin.position.y = generated["origin_y"]
        grid.info.origin.position.z = generated["origin_z"]
        grid.data = generated["data"]
    occupied_cells = 0
    if args.obstacle_forward_m is not None:
        for iy in range(grid.info.height):
            for ix in range(grid.info.width):
                x = grid.info.origin.position.x + (ix + 0.5) * grid.info.resolution
                y = grid.info.origin.position.y + (iy + 0.5) * grid.info.resolution
                dx, dy = x - ego_x, y - ego_y
                forward = dx * math.cos(yaw) + dy * math.sin(yaw)
                lateral = -dx * math.sin(yaw) + dy * math.cos(yaw)
                if abs(forward - args.obstacle_forward_m) <= args.obstacle_half_length_m and \
                        abs(lateral) <= args.obstacle_half_width_m:
                    grid.data[iy * grid.info.width + ix] = 100
                    occupied_cells += 1
    scenario = Scenario()
    scenario.current_scenario = Scenario.PARKING
    scenario.activating_scenarios = [Scenario.PARKING]

    log_path = args.output.with_suffix(".log")
    with log_path.open("w") as log:
        process = subprocess.Popen(
            ["ros2", "launch", "/probe/reference/freespace_planner_probe.launch.xml",
             "adapt_expansion_distance:=" +
             ("false" if args.no_adapt_expansion else "true")],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Node("freespace_planner_reference_probe")
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        route_pub = node.create_publisher(LaneletRoute, "/probe/route", durable)
        grid_pub = node.create_publisher(OccupancyGrid, "/probe/occupancy_grid", 10)
        scenario_pub = node.create_publisher(Scenario, "/probe/scenario", 10)
        odom_pub = node.create_publisher(Odometry, "/probe/odometry", 10)
        trajectories = []
        completed = []
        trajectory_receipts = []
        full_pose_arrays = []
        def receive_trajectory(msg):
            trajectories.append(msg)
            trajectory_receipts.append(time.monotonic_ns())
        node.create_subscription(Trajectory, "/probe/trajectory",
                                 receive_trajectory, durable)
        node.create_subscription(Bool, "/probe/is_completed",
                                 lambda msg: completed.append(bool(msg.data)), durable)
        node.create_subscription(PoseArray,
                                 "/probe/freespace_planner/debug/pose_array",
                                 lambda msg: full_pose_arrays.append(msg), durable)
        try:
            deadline = time.monotonic() + args.duration_s
            route_sent = False
            while time.monotonic() < deadline:
                now = node.get_clock().now().to_msg()
                odometry.header.stamp = now
                grid.header.stamp = now
                if not route_sent and node.count_subscribers("/probe/route"):
                    route.header.stamp = now
                    route_pub.publish(route)
                    route_sent = True
                grid_pub.publish(grid)
                scenario_pub.publish(scenario)
                odom_pub.publish(odometry)
                rclpy.spin_once(node, timeout_sec=0.1)
                if trajectories and len(trajectories[-1].points) > 1:
                    break
            if not trajectories or len(trajectories[-1].points) <= 1:
                raise RuntimeError(f"원본 자유공간 경로가 발행되지 않았습니다: {log_path}")
            deadline = time.monotonic() + args.observe_seconds
            while time.monotonic() < deadline:
                odometry.header.stamp = node.get_clock().now().to_msg()
                grid.header.stamp = odometry.header.stamp
                grid_pub.publish(grid)
                scenario_pub.publish(scenario)
                odom_pub.publish(odometry)
                rclpy.spin_once(node, timeout_sec=0.1)
            path = trajectories[-1]
            output_topics = {
                name: {
                    "types": types,
                    "publisher_durability": [
                        str(info.qos_profile.durability)
                        for info in node.get_publishers_info_by_topic(name)],
                }
                for name, types in node.get_topic_names_and_types()
                if name in {"/probe/trajectory", "/probe/is_completed",
                            "/probe/freespace_planner/debug/pose_array",
                            "/probe/freespace_planner/debug/partial_pose_array",
                            "/probe/freespace_planner/debug/processing_time_ms"}
            }
            result = {
                "input": {
                    "ego": [ego_x, ego_y, yaw],
                    "goal": [goal_x, goal_y, yaw],
                    "grid": [grid.info.width, grid.info.height,
                             grid.info.resolution],
                    "scenario": "PARKING synthetic",
                    "grid_reference": str(args.grid_reference) if args.grid_reference else None,
                    "adapt_expansion_distance": not args.no_adapt_expansion,
                    "obstacle": None if args.obstacle_forward_m is None else {
                        "forward_m": args.obstacle_forward_m,
                        "half_length_m": args.obstacle_half_length_m,
                        "half_width_m": args.obstacle_half_width_m,
                        "occupied_cells": occupied_cells,
                    },
                },
                "output": {
                    "trajectory_count": len(trajectories),
                    "path_points": len(path.points),
                    "frame_id": path.header.frame_id,
                    "first": [path.points[0].pose.position.x,
                              path.points[0].pose.position.y,
                              path.points[0].longitudinal_velocity_mps],
                    "last": [path.points[-1].pose.position.x,
                             path.points[-1].pose.position.y,
                             path.points[-1].longitudinal_velocity_mps],
                    "points": [[point.pose.position.x,
                                point.pose.position.y,
                                point.pose.orientation.z,
                                point.pose.orientation.w,
                                point.longitudinal_velocity_mps]
                               for point in path.points],
                    "completed_history": completed,
                    "full_path_poses": [[pose.position.x, pose.position.y,
                                         pose.orientation.z, pose.orientation.w]
                                        for pose in full_pose_arrays[-1].poses]
                        if full_pose_arrays else [],
                },
                "graph": output_topics,
                "observation": {
                    "trajectory_intervals_ms": [
                        (b - a) / 1e6 for a, b in
                        zip(trajectory_receipts, trajectory_receipts[1:])],
                },
            }
            if args.complete_after_plan:
                odometry.pose.pose = copy.deepcopy(path.points[-1].pose)
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline and True not in completed:
                    odometry.header.stamp = node.get_clock().now().to_msg()
                    grid_pub.publish(grid)
                    scenario_pub.publish(scenario)
                    odom_pub.publish(odometry)
                    rclpy.spin_once(node, timeout_sec=0.1)
                if True not in completed:
                    raise RuntimeError("원본 완료 상태가 발행되지 않았습니다")
                for _ in range(5):
                    odometry.header.stamp = node.get_clock().now().to_msg()
                    odom_pub.publish(odometry)
                    rclpy.spin_once(node, timeout_sec=0.1)
                result["completion_transition"] = {
                    "completed_history": completed,
                    "trajectory_count": len(trajectories),
                    "all_final_velocities_zero": bool(trajectories) and all(
                        abs(point.longitudinal_velocity_mps) < 1e-6
                        for point in trajectories[-1].points),
                }
            if args.advance_first_segment:
                first_last = path.points[-1].pose.position
                odometry.pose.pose = copy.deepcopy(path.points[-1].pose)
                deadline = time.monotonic() + 5.0
                next_path = None
                while time.monotonic() < deadline:
                    odometry.header.stamp = node.get_clock().now().to_msg()
                    grid.header.stamp = odometry.header.stamp
                    grid_pub.publish(grid)
                    scenario_pub.publish(scenario)
                    odom_pub.publish(odometry)
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if trajectories and trajectories[-1].points:
                        last = trajectories[-1].points[-1].pose.position
                        if math.hypot(last.x - first_last.x,
                                      last.y - first_last.y) > 0.1:
                            next_path = trajectories[-1]
                            break
                if next_path is None:
                    raise RuntimeError("원본 다음 주차 구간이 발행되지 않았습니다")
                result["segment_transition"] = {
                    "completed_history": completed,
                    "point_count": len(next_path.points),
                    "points": [[point.pose.position.x,
                                point.pose.position.y,
                                point.longitudinal_velocity_mps]
                               for point in next_path.points],
                }
            args.output.write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result["output"]), flush=True)
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
