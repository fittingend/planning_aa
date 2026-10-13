"""기록된 지도·경로·궤적으로 원본 시나리오 선택기를 재생합니다."""

import argparse
import json
import os
import signal
import sqlite3
import subprocess
import time
from pathlib import Path
import copy

import rclpy
from rclpy.serialization import deserialize_message
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_map_msgs.msg import LaneletMapBin
from autoware_planning_msgs.msg import LaneletRoute, Trajectory
from nav_msgs.msg import Odometry
from tier4_planning_msgs.msg import Scenario
from std_msgs.msg import Bool


def load(path, topic, fraction=0.0, near=None):
    connection = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    topic_id = connection.execute(
        "SELECT id FROM topics WHERE name=?", (topic,)
    ).fetchone()[0]
    if near is None:
        count = connection.execute(
            "SELECT COUNT(*) FROM messages WHERE topic_id=?", (topic_id,)
        ).fetchone()[0]
        row = connection.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? "
            "ORDER BY timestamp LIMIT 1 OFFSET ?", (topic_id, int(count * fraction))
        ).fetchone()
    else:
        row = connection.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? "
            "ORDER BY ABS(timestamp-?) LIMIT 1", (topic_id, near)
        ).fetchone()
    connection.close()
    return row


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sample-fraction", type=float, default=0.25)
    parser.add_argument("--map-bin-file", type=Path)
    parser.add_argument("--synthetic-parking", action="store_true")
    parser.add_argument("--synthetic-parking-timeout", action="store_true")
    parser.add_argument("--synthetic-parking-empty-timeout", action="store_true")
    args = parser.parse_args()

    timestamp, trajectory_bytes = load(
        args.bag, "/planning/scenario_planning/lane_driving/trajectory",
        args.sample_fraction,
    )
    _, odometry_bytes = load(args.bag, "/localization/kinematic_state", near=timestamp)
    _, map_bytes = load(args.bag, "/map/vector_map")
    _, route_bytes = load(args.bag, "/planning/mission_planning/route")
    trajectory = deserialize_message(trajectory_bytes, Trajectory)
    odometry = deserialize_message(odometry_bytes, Odometry)
    lanelet_map = deserialize_message(map_bytes, LaneletMapBin)
    route = deserialize_message(route_bytes, LaneletRoute)
    if args.map_bin_file:
        lanelet_map = deserialize_message(args.map_bin_file.read_bytes(), LaneletMapBin)
    if (args.synthetic_parking or args.synthetic_parking_timeout or
            args.synthetic_parking_empty_timeout):
        odometry.twist.twist.linear.x = 0.0
    if args.synthetic_parking or args.synthetic_parking_empty_timeout:
        route.goal_pose.position.x = 0.0
        route.goal_pose.position.y = 0.0

    log_path = Path("/probe/reference/scenario_selector_decision.log")
    with log_path.open("w") as log:
        process = subprocess.Popen(
            ["ros2", "launch", "/probe/reference/scenario_selector_probe.launch.xml"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        rclpy.init()
        node = Node("scenario_selector_reference_probe")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        map_pub = node.create_publisher(LaneletMapBin, "/probe/map", qos)
        route_pub = node.create_publisher(LaneletRoute, "/probe/route", qos)
        odom_pub = node.create_publisher(Odometry, "/probe/odometry", 100)
        mode_pub = node.create_publisher(OperationModeState, "/probe/mode", 10)
        lane_pub = node.create_publisher(Trajectory, "/probe/lane_trajectory", 10)
        parking_pub = node.create_publisher(Trajectory, "/probe/parking_trajectory", 10)
        completed_pub = node.create_publisher(Bool, "/probe/parking_completed", 10)
        scenarios = []
        selected = []
        node.create_subscription(
            Scenario, "/probe/scenario",
            lambda message: scenarios.append(message.current_scenario), 10,
        )
        node.create_subscription(
            Trajectory, "/probe/selected_trajectory",
            lambda message: selected.append(message), 10,
        )
        mode = OperationModeState()
        mode.mode = OperationModeState.AUTONOMOUS
        mode.is_autoware_control_enabled = True
        try:
            spin_for(node, 2.0)
            for _ in range(30):
                map_pub.publish(lanelet_map)
                route_pub.publish(route)
                odom_pub.publish(odometry)
                mode_pub.publish(mode)
                spin_for(node, 0.1)
                if scenarios:
                    break
            if not scenarios:
                raise RuntimeError("원본 선택기의 시나리오 출력을 받지 못했습니다")
            parking_initial_scenario = scenarios[-1]
            parking_published = 0
            if args.synthetic_parking or args.synthetic_parking_empty_timeout:
                parking_trajectory = Trajectory()
                parking_trajectory.header = trajectory.header
                parking_point = copy.deepcopy(trajectory.points[0])
                parking_point.pose = copy.deepcopy(odometry.pose.pose)
                parking_point.longitudinal_velocity_mps = 0.0
                parking_trajectory.points = [parking_point]
                parking_trajectory.header.stamp = node.get_clock().now().to_msg()
                parking_pub.publish(parking_trajectory)
                spin_for(node, 0.4)
                parking_published = len(selected)
                if args.synthetic_parking:
                    completed_pub.publish(Bool(data=True))
                    odom_pub.publish(odometry)
                    spin_for(node, 0.8)
            empty_timeout_initial_scenario = scenarios[-1]
            empty_timeout_s = None
            if args.synthetic_parking_empty_timeout:
                start = time.monotonic()
                for _ in range(30):
                    odom_pub.publish(odometry)
                    spin_for(node, 0.2)
                    if scenarios and scenarios[-1] == Scenario.LANEDRIVING:
                        empty_timeout_s = time.monotonic() - start
                        break
            timeout_initial_scenario = scenarios[-1]
            timeout_s = None
            if args.synthetic_parking_timeout:
                trajectory.header.stamp = node.get_clock().now().to_msg()
                lane_pub.publish(trajectory)
                spin_for(node, 0.3)
                route.goal_pose.position.x = 0.0
                route.goal_pose.position.y = 0.0
                route_pub.publish(route)
                start = time.monotonic()
                for _ in range(40):
                    odom_pub.publish(odometry)
                    spin_for(node, 0.2)
                    if scenarios and scenarios[-1] == Scenario.PARKING:
                        timeout_s = time.monotonic() - start
                        break
            trajectory.header.stamp = node.get_clock().now().to_msg()
            lane_pub.publish(trajectory)
            spin_for(node, 0.5)
            fresh_count = len(selected)
            stale = Trajectory()
            stale.header = trajectory.header
            stale.points = trajectory.points
            stale.header.stamp.sec -= 3
            lane_pub.publish(stale)
            spin_for(node, 0.5)
            result = {
                "bag_timestamp": timestamp,
                "scenario": scenarios[-1],
                "ego": {"x": odometry.pose.pose.position.x,
                        "y": odometry.pose.pose.position.y,
                        "velocity": odometry.twist.twist.linear.x},
                "input_points": len(trajectory.points),
                "fresh_published": fresh_count,
                "stale_published": len(selected) - fresh_count,
                "output_points": len(selected[-1].points) if selected else None,
            }
            if args.synthetic_parking:
                result["synthetic_parking"] = {
                    "initial_scenario": parking_initial_scenario,
                    "completed_scenario": scenarios[-1],
                    "parking_path_published": parking_published,
                    "parking_lot_map": args.map_bin_file.name,
                    "goal_x": route.goal_pose.position.x,
                    "goal_y": route.goal_pose.position.y,
                }
            if args.synthetic_parking_empty_timeout:
                result["synthetic_parking_empty_timeout"] = {
                    "initial_scenario": empty_timeout_initial_scenario,
                    "completed_scenario": scenarios[-1],
                    "transition_delay_s": empty_timeout_s,
                    "parking_path_published": parking_published,
                }
            if args.synthetic_parking_timeout:
                result["synthetic_parking_timeout"] = {
                    "initial_scenario": timeout_initial_scenario,
                    "completed_scenario": scenarios[-1],
                    "transition_delay_s": timeout_s,
                    "goal_x": route.goal_pose.position.x,
                    "goal_y": route.goal_pose.position.y,
                }
            args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
            print(json.dumps(result), flush=True)
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
