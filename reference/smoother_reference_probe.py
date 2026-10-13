"""decision 이미지에서 원본 velocity smoother의 기준 궤적을 수집한다."""

import argparse
import json
import math
import os
import signal
import sqlite3
import subprocess
import time
from pathlib import Path

import rclpy
from autoware_adapi_v1_msgs.msg import OperationModeState
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from tier4_planning_msgs.msg import VelocityLimit
from rclpy.node import Node


OUTPUT = Path("/probe/reference/smoother_reference.json")
LOG = Path("/probe/reference/smoother_decision.log")
NODE = "/planning/scenario_planning/velocity_smoother"
TOPICS = {
    "output": f"{NODE}/trajectory",
    "raw": f"{NODE}/debug/trajectory_raw",
    "limited": f"{NODE}/debug/trajectory_external_velocity_limited",
    "lateral": f"{NODE}/debug/trajectory_lateral_acc_filtered",
    "steering": f"{NODE}/debug/trajectory_steering_rate_limited",
    "resampled": f"{NODE}/debug/trajectory_time_resampled",
    "forward": f"{NODE}/debug/forward_filtered_trajectory",
    "backward": f"{NODE}/debug/backward_filtered_trajectory",
    "merged": f"{NODE}/debug/merged_filtered_trajectory",
}


def points_to_json(message):
    return [
        {
            "x": point.pose.position.x,
            "y": point.pose.position.y,
            "z": point.pose.position.z,
            "orientation_x": point.pose.orientation.x,
            "orientation_y": point.pose.orientation.y,
            "orientation_z": point.pose.orientation.z,
            "orientation_w": point.pose.orientation.w,
            "velocity": point.longitudinal_velocity_mps,
            "acceleration": point.acceleration_mps2,
        }
        for point in message.points
    ]


def spin_for(node, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def load_bag_case(path, fraction):
    connection = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    topics = {
        name: (topic_id, type_name)
        for topic_id, name, type_name in connection.execute(
            "SELECT id, name, type FROM topics"
        )
    }

    def get_sample(topic, timestamp=None):
        topic_id, type_name = topics[topic]
        message_type = get_message(type_name)
        if timestamp is None:
            count = connection.execute(
                "SELECT COUNT(*) FROM messages WHERE topic_id=?", (topic_id,)
            ).fetchone()[0]
            row = connection.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id=? "
                "ORDER BY timestamp LIMIT 1 OFFSET ?",
                (topic_id, int(count * fraction)),
            ).fetchone()
        else:
            row = connection.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id=? "
                "ORDER BY ABS(timestamp-?) LIMIT 1",
                (topic_id, timestamp),
            ).fetchone()
        return row[0], deserialize_message(row[1], message_type)

    try:
        timestamp, trajectory = get_sample(
            "/planning/scenario_planning/scenario_selector/trajectory"
        )
        _, odometry = get_sample("/localization/kinematic_state", timestamp)
        _, acceleration = get_sample("/localization/acceleration", timestamp)
        return timestamp, trajectory, odometry, acceleration
    finally:
        connection.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--sample-fraction", type=float, default=0.5)
    parser.add_argument("--case", type=str)
    parser.add_argument("--synthetic-curve", action="store_true")
    parser.add_argument("--synthetic-reverse", action="store_true")
    parser.add_argument("--sequence", action="store_true")
    parser.add_argument("--sequence-frames", type=int, default=2)
    parser.add_argument("--single-publish-sequence", action="store_true")
    parser.add_argument("--external-limit-change", action="store_true")
    parser.add_argument("--initial-speed", type=float, default=5.0)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    with LOG.open("w") as log_file:
        process = subprocess.Popen(
            ["ros2", "launch", "/probe/reference/smoother_probe.launch.xml"],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        rclpy.init()
        node = Node("smoother_reference_probe")
        received = {}
        subscriptions = [
            node.create_subscription(
                Trajectory, topic,
                lambda message, key=key: received.__setitem__(key, points_to_json(message)),
                10,
            )
            for key, topic in TOPICS.items()
        ]
        odometry_pub = node.create_publisher(
            Odometry, "/localization/kinematic_state", 10
        )
        acceleration_pub = node.create_publisher(
            AccelWithCovarianceStamped, "/localization/acceleration", 10
        )
        trajectory_pub = node.create_publisher(
            Trajectory, "/planning/scenario_planning/scenario_selector/trajectory", 10
        )
        mode_pub = node.create_publisher(
            OperationModeState, "/system/operation_mode/state", 10
        )
        limit_pub = node.create_publisher(
            VelocityLimit, "/planning/scenario_planning/max_velocity", 10
        )

        try:
            spin_for(node, 3.0)

            if args.bag:
                bag_timestamp, trajectory, odometry, acceleration = load_bag_case(
                    args.bag, args.sample_fraction
                )
                case = args.case or "bag_sample"
            else:
                bag_timestamp = None
                case = (
                    "reverse_stop_100m" if args.synthetic_reverse
                    else "curved_stop_30m" if args.synthetic_curve
                    else "straight_stop_100m"
                )
                trajectory = Trajectory()
                trajectory.header.frame_id = "map"
                count = 61 if args.synthetic_curve else 101
                for index in range(count):
                    point = TrajectoryPoint()
                    x = index * 0.5 if args.synthetic_curve else float(index)
                    point.pose.position.x = x
                    if args.synthetic_curve:
                        point.pose.position.y = 4.0 * math.sin(x / 4.0)
                        yaw = math.atan(math.cos(x / 4.0))
                        point.pose.orientation.z = math.sin(yaw / 2.0)
                        point.pose.orientation.w = math.cos(yaw / 2.0)
                    else:
                        point.pose.orientation.w = 1.0
                    point.longitudinal_velocity_mps = 0.0 if index + 1 == count else 10.0
                    if args.synthetic_reverse:
                        point.longitudinal_velocity_mps *= -1.0
                    trajectory.points.append(point)

                odometry = Odometry()
                odometry.header.frame_id = "map"
                odometry.pose.pose.orientation.w = 1.0
                odometry.twist.twist.linear.x = (
                    -args.initial_speed if args.synthetic_reverse else args.initial_speed
                )
                acceleration = AccelWithCovarianceStamped()
                acceleration.header.frame_id = "map"
            mode = OperationModeState()
            mode.mode = OperationModeState.AUTONOMOUS
            mode.is_autoware_control_enabled = True

            for _ in range(30):
                stamp = node.get_clock().now().to_msg()
                trajectory.header.stamp = stamp
                odometry.header.stamp = stamp
                acceleration.header.stamp = stamp
                odometry_pub.publish(odometry)
                acceleration_pub.publish(acceleration)
                if args.sequence:
                    mode_pub.publish(mode)
                trajectory_pub.publish(trajectory)
                spin_for(node, 0.25)
                if "output" in received and "resampled" in received and (
                        not args.sequence or args.sequence_frames <= 2 or _ >= 4):
                    break
            if args.sequence and args.sequence_frames > 2:
                spin_for(node, 0.5)

            first_trajectories = None
            sequence_records = []
            if args.sequence:
                first_trajectories = dict(received)
                sequence_records.append({
                    "ego_x": odometry.pose.pose.position.x,
                    "ego_y": odometry.pose.pose.position.y,
                    "ego_velocity": odometry.twist.twist.linear.x,
                    "trajectories": dict(received),
                })
                case = f"{case}_second"
                if args.external_limit_change:
                    case += "_external_limit"
                limit = VelocityLimit()
                limit.max_velocity = 2.0
                limit.sender = "planning_core_probe"
                for frame_index in range(1, args.sequence_frames):
                    received.clear()
                    odometry.pose.pose.position.x += 2.0
                    if args.synthetic_curve:
                        phase = odometry.pose.pose.position.x / 4.0
                        odometry.pose.pose.position.y = 4.0 * math.sin(phase)
                        yaw = math.atan(math.cos(phase))
                        odometry.pose.pose.orientation.z = math.sin(yaw / 2.0)
                        odometry.pose.pose.orientation.w = math.cos(yaw / 2.0)
                    odometry.twist.twist.linear.x = max(
                        args.initial_speed - frame_index, 0.0)
                    if args.single_publish_sequence:
                        stamp = node.get_clock().now().to_msg()
                        trajectory.header.stamp = stamp
                        odometry.header.stamp = stamp
                        acceleration.header.stamp = stamp
                        odometry_pub.publish(odometry)
                        acceleration_pub.publish(acceleration)
                        mode_pub.publish(mode)
                        if args.external_limit_change:
                            limit.stamp = stamp
                            limit_pub.publish(limit)
                        spin_for(node, 0.15)
                        trajectory_pub.publish(trajectory)
                        deadline = time.monotonic() + 5.0
                        while time.monotonic() < deadline and (
                                "output" not in received or
                                "resampled" not in received):
                            rclpy.spin_once(node, timeout_sec=0.05)
                        spin_for(node, 0.2)
                        if "output" not in received or "resampled" not in received:
                            raise RuntimeError(
                                f"원본 스무더 {frame_index}번째 출력을 받지 못했습니다")
                        sequence_records.append({
                            "ego_x": odometry.pose.pose.position.x,
                            "ego_y": odometry.pose.pose.position.y,
                            "ego_velocity": odometry.twist.twist.linear.x,
                            "trajectories": dict(received),
                        })
                        continue
                    for _ in range(30):
                        stamp = node.get_clock().now().to_msg()
                        trajectory.header.stamp = stamp
                        odometry.header.stamp = stamp
                        acceleration.header.stamp = stamp
                        odometry_pub.publish(odometry)
                        acceleration_pub.publish(acceleration)
                        mode_pub.publish(mode)
                        if args.external_limit_change:
                            limit.stamp = stamp
                            limit_pub.publish(limit)
                        trajectory_pub.publish(trajectory)
                        spin_for(node, 0.25)
                        if "output" in received and "resampled" in received and (
                                args.sequence_frames <= 2 or _ >= 4):
                            break
                    if "output" not in received or "resampled" not in received:
                        raise RuntimeError(
                            f"원본 스무더 {frame_index}번째 출력을 받지 못했습니다")
                    if args.sequence_frames > 2:
                        spin_for(node, 0.5)
                    sequence_records.append({
                        "ego_x": odometry.pose.pose.position.x,
                        "ego_y": odometry.pose.pose.position.y,
                        "ego_velocity": odometry.twist.twist.linear.x,
                        "trajectories": dict(received),
                    })

            result = {
                "case": case,
                "bag_timestamp": bag_timestamp,
                "input": points_to_json(trajectory),
                "ego_velocity": odometry.twist.twist.linear.x,
                "ego_acceleration": acceleration.accel.accel.linear.x,
                "ego_pose": {
                    "x": odometry.pose.pose.position.x,
                    "y": odometry.pose.pose.position.y,
                    "z": odometry.pose.pose.position.z,
                    "orientation_x": odometry.pose.pose.orientation.x,
                    "orientation_y": odometry.pose.pose.orientation.y,
                    "orientation_z": odometry.pose.pose.orientation.z,
                    "orientation_w": odometry.pose.pose.orientation.w,
                },
                "debug_enabled_at_launch": True,
                "trajectories": received,
                "first_trajectories": first_trajectories,
                "sequence_records": sequence_records,
            }
            args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
            print(json.dumps({key: len(value) for key, value in received.items()}), flush=True)
            if "output" not in received or "resampled" not in received:
                raise RuntimeError("원본 스무더 출력과 재표본화 디버그 궤적을 받지 못했습니다")
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
