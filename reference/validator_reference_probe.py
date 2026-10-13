"""원본 validator의 검사 상태를 합성 경로와 기록 궤적에서 수집합니다."""

import argparse
import gzip
import json
import sqlite3
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.serialization import deserialize_message
from rclpy.node import Node
from nav_msgs.msg import Odometry
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from autoware_planning_validator.msg import PlanningValidatorStatus


STATUS_FIELDS = (
    "is_valid_size", "is_valid_finite_value", "is_valid_interval",
    "is_valid_relative_angle", "is_valid_curvature", "is_valid_lateral_acc",
    "is_valid_longitudinal_max_acc", "is_valid_longitudinal_min_acc",
    "is_valid_steering", "is_valid_steering_rate", "is_valid_velocity_deviation",
    "is_valid_distance_deviation", "is_valid_longitudinal_distance_deviation",
    "is_valid_forward_trajectory_length", "trajectory_size", "max_interval_distance",
    "max_relative_angle", "max_curvature", "max_lateral_acc",
    "max_longitudinal_acc", "min_longitudinal_acc", "max_steering",
    "max_steering_rate", "velocity_deviation", "distance_deviation",
    "longitudinal_distance_deviation", "forward_trajectory_length_required",
    "forward_trajectory_length_measured", "invalid_count",
)


def load_bag_sequence(path, fraction, count):
    connection = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    topics = {name: topic_id for topic_id, name in connection.execute(
        "SELECT id, name FROM topics"
    )}
    trajectory_topic = topics["/planning/scenario_planning/velocity_smoother/trajectory"]
    total = connection.execute(
        "SELECT COUNT(*) FROM messages WHERE topic_id=?", (trajectory_topic,)
    ).fetchone()[0]
    rows = connection.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? "
        "ORDER BY timestamp LIMIT ? OFFSET ?",
        (trajectory_topic, count, int(total * fraction)),
    ).fetchall()
    odometry_topic = topics["/localization/kinematic_state"]
    samples = []
    for timestamp, data in rows:
        trajectory = deserialize_message(data, Trajectory)
        odometry_data = connection.execute(
            "SELECT data FROM messages WHERE topic_id=? "
            "ORDER BY ABS(timestamp-?) LIMIT 1", (odometry_topic, timestamp)
        ).fetchone()[0]
        samples.append((timestamp, trajectory,
                        deserialize_message(odometry_data, Odometry)))
    connection.close()
    return samples


def trajectory_to_json(message):
    return [{
        "x": point.pose.position.x, "y": point.pose.position.y,
        "z": point.pose.position.z,
        "orientation_x": point.pose.orientation.x,
        "orientation_y": point.pose.orientation.y,
        "orientation_z": point.pose.orientation.z,
        "orientation_w": point.pose.orientation.w,
        "velocity": point.longitudinal_velocity_mps,
        "heading_rate": point.heading_rate_rps,
        "acceleration": point.acceleration_mps2,
    } for point in message.points]


def make_trajectory(points):
    result = Trajectory()
    result.header.frame_id = "map"
    for item in points:
        x, y, velocity = item[:3]
        point = TrajectoryPoint()
        point.pose.position.x = x
        point.pose.position.y = y
        point.pose.orientation.w = 1.0
        point.longitudinal_velocity_mps = velocity
        if len(item) > 3:
            point.acceleration_mps2 = item[3]
        result.points.append(point)
    return result


parser = argparse.ArgumentParser()
parser.add_argument("--bag", type=Path)
parser.add_argument("--sample-fraction", type=float, default=0.5)
parser.add_argument("--bag-sequence-count", type=int, default=1)
parser.add_argument("--invalidate-sequence-index", type=int)
parser.add_argument("--case")
parser.add_argument("--output", type=Path)
args = parser.parse_args()
if args.bag_sequence_count < 1:
    parser.error("연속 입력 개수는 1 이상이어야 합니다")

process = subprocess.Popen(
    [
        "ros2", "launch", "autoware_planning_validator", "planning_validator.launch.xml",
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
)
rclpy.init()
node = Node("validator_probe")
received = []
node.create_subscription(
    PlanningValidatorStatus,
    "/planning_validator/validation_status",
    lambda msg: received.append(msg),
    10,
)
pub_odom = node.create_publisher(Odometry, "/localization/kinematic_state", 10)
pub_traj = node.create_publisher(
    Trajectory, "/planning/scenario_planning/velocity_smoother/trajectory", 10
)


def spin_for(seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


try:
    spin_for(2.0)
    odom = Odometry()
    odom.header.frame_id = "map"
    odom.pose.pose.orientation.w = 1.0
    odom.twist.twist.linear.x = 10.0
    cases = [
        ("straight", [(0.0, 0.0, 10.0), (10.0, 0.0, 10.0), (20.0, 0.0, 10.0)]),
        ("short", [(0.0, 0.0, 10.0)]),
        ("corner", [(0.0, 0.0, 2.0), (2.0, 0.0, 2.0), (2.0, 2.0, 2.0)]),
        ("wide_gap", [(0.0, 0.0, 10.0), (150.0, 0.0, 10.0), (300.0, 0.0, 10.0)]),
        ("fast_corner", [(0.0, 0.0, 10.0), (2.0, 0.0, 10.0),
                         (2.0, 2.0, 10.0), (2.0, 20.0, 10.0)]),
        ("acceleration", [(0.0, 0.0, 10.0, 12.0),
                          (10.0, 0.0, 10.0, -12.0), (30.0, 0.0, 10.0, 0.0)]),
        ("distant", [(200.0, 0.0, 150.0), (210.0, 0.0, 150.0),
                     (220.0, 0.0, 150.0)]),
    ]
    if args.bag:
        samples = load_bag_sequence(
            args.bag, args.sample_fraction, args.bag_sequence_count)
        if args.invalidate_sequence_index is not None:
            if not 0 <= args.invalidate_sequence_index < len(samples):
                raise ValueError("무효 궤적 프레임 인덱스가 범위를 벗어났습니다")
            samples[args.invalidate_sequence_index][1].points = \
                samples[args.invalidate_sequence_index][1].points[:1]
        cases = [
            (f"{args.case or 'bag_sample'}_{index}"
             if args.bag_sequence_count > 1 else args.case or "bag_sample",
             None, timestamp,
             trajectory, sample_odom)
            for index, (timestamp, trajectory, sample_odom) in enumerate(samples)
        ]
    else:
        cases = [(name, points, None, None, odom) for name, points in cases]
    records = []
    for name, points, timestamp, bag_trajectory, odom in cases:
        previous_count = len(received)
        input_trajectory = bag_trajectory if args.bag else make_trajectory(points)
        for _ in range(10):
            pub_odom.publish(odom)
            spin_for(0.05)
        for _ in range(10):
            pub_traj.publish(input_trajectory)
            spin_for(0.1)
            if len(received) > previous_count:
                break
        if len(received) == previous_count:
            if args.bag_sequence_count > 1:
                raise RuntimeError(f"연속 입력 {name}의 상태를 받지 못했습니다")
            print(json.dumps({"case": name, "error": "no status"}), flush=True)
            continue
        status = received[-1]
        record = {
            "case": name,
            "bag_timestamp": timestamp if args.bag else None,
            "ego": {
                "x": odom.pose.pose.position.x,
                "y": odom.pose.pose.position.y,
                "velocity": odom.twist.twist.linear.x,
            },
            "trajectory": trajectory_to_json(input_trajectory),
            "status": {field: getattr(status, field) for field in STATUS_FIELDS},
        }
        records.append(record)
        print(json.dumps({"case": name, "size": len(input_trajectory.points),
                          "invalid_count": status.invalid_count}), flush=True)
    if args.output:
        serialized = json.dumps(records, ensure_ascii=False, indent=2) + "\n"
        if args.output.suffix == ".gz":
            with gzip.open(args.output, "wt") as stream:
                stream.write(serialized)
        else:
            args.output.write_text(serialized)
finally:
    process.terminate()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
    node.destroy_node()
    rclpy.shutdown()
