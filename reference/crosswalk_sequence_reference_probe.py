"""보행자 rosbag의 연속 경로를 원본 횡단보도 모듈에 순서대로 재생합니다."""

import argparse
import json
import os
import signal
import sqlite3
import subprocess
from pathlib import Path

import rclpy
from autoware_internal_debug_msgs.msg import StringStamped
from autoware_map_msgs.msg import LaneletMapBin
from autoware_perception_msgs.msg import PredictedObjects
from autoware_planning_msgs.msg import Path as PlanningPath
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from tier4_planning_msgs.msg import PathWithLaneId

from scenario_selector_reference_probe import load, spin_for
from stop_line_planner_reference_probe import record


def path_messages(bag, fraction, count):
    connection = sqlite3.connect(f"file:{bag}?mode=ro", uri=True)
    topic = connection.execute(
        "SELECT id FROM topics WHERE name=?",
        ("/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id",),
    ).fetchone()[0]
    total = connection.execute(
        "SELECT COUNT(*) FROM messages WHERE topic_id=?", (topic,)
    ).fetchone()[0]
    start = max(0, min(total - count, int(total * fraction)))
    rows = connection.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? "
        "ORDER BY timestamp LIMIT ? OFFSET ?", (topic, count, start)
    ).fetchall()
    connection.close()
    return rows


def record_object(obj):
    identifier = bytes(obj.object_id.uuid).hex()
    value = {
        "id": identifier,
        "label": obj.classification[0].label if obj.classification else None,
        "x": obj.kinematics.initial_pose_with_covariance.pose.position.x,
        "y": obj.kinematics.initial_pose_with_covariance.pose.position.y,
        "velocity_x": obj.kinematics.initial_twist_with_covariance.twist.linear.x,
        "velocity_y": obj.kinematics.initial_twist_with_covariance.twist.linear.y,
        "length": obj.shape.dimensions.x,
        "width": obj.shape.dimensions.y,
    }
    if identifier.startswith("a9b8"):
        value["paths"] = [{
            "poses": [{
                "x": pose.position.x,
                "y": pose.position.y,
                "qz": pose.orientation.z,
                "qw": pose.orientation.w,
            } for pose in predicted.path],
        } for predicted in obj.kinematics.predicted_paths]
    return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sample-fraction", type=float, default=0.38)
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--object-pose-shift-x-after-frame", nargs=2, type=float,
                        metavar=("FRAME", "SHIFT_X"))
    args = parser.parse_args()
    if args.count <= 0:
        raise ValueError("재생할 프레임 수는 양수여야 합니다")
    frames = path_messages(args.bag, args.sample_fraction, args.count)
    _, map_bytes = load(args.bag, "/map/vector_map")
    lanelet_map = deserialize_message(map_bytes, LaneletMapBin)
    topics = (
        ("/localization/kinematic_state", Odometry),
        ("/localization/acceleration", AccelWithCovarianceStamped),
        ("/perception/object_recognition/objects", PredictedObjects),
        ("/perception/occupancy_grid_map/map", OccupancyGrid),
    )
    log_path = args.output.with_suffix(".log")
    with log_path.open("w") as log:
        process = subprocess.Popen(
            ["ros2", "launch", "/probe/reference/stop_line_planner_probe.launch.xml",
             "module:=[autoware::behavior_velocity_planner::CrosswalkModulePlugin]",
             "crosswalk_enable_rtc:=true"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        rclpy.init()
        node = Node("crosswalk_sequence_reference_probe")
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publishers = [
            node.create_publisher(LaneletMapBin, "/probe/map", map_qos),
            node.create_publisher(Odometry, "/probe/odometry", 10),
            node.create_publisher(AccelWithCovarianceStamped, "/probe/acceleration", 10),
            node.create_publisher(PredictedObjects, "/probe/objects", 10),
            node.create_publisher(OccupancyGrid, "/probe/occupancy", 10),
            node.create_publisher(PointCloud2, "/probe/pointcloud", 10),
        ]
        path_pub = node.create_publisher(PathWithLaneId, "/probe/input_path", 10)
        outputs = []
        collisions = []
        node.create_subscription(PlanningPath, "/probe/output_path", outputs.append, 10)
        node.create_subscription(
            StringStamped,
            "/probe/behavior_velocity_planner/debug/collision_info",
            collisions.append, 10,
        )
        pointcloud = PointCloud2()
        pointcloud.header.frame_id = "map"
        result = []
        try:
            spin_for(node, 3.0)
            for frame_index, (timestamp, path_bytes) in enumerate(frames):
                path = deserialize_message(path_bytes, PathWithLaneId)
                messages = [
                    deserialize_message(load(args.bag, topic, near=timestamp)[1], kind)
                    for topic, kind in topics
                ]
                if args.object_pose_shift_x_after_frame:
                    after, shift = args.object_pose_shift_x_after_frame
                    if frame_index >= after:
                        for obj in messages[2].objects:
                            if bytes(obj.object_id.uuid).hex().startswith("a9b8"):
                                obj.kinematics.initial_pose_with_covariance.pose.position.x += shift
                previous_outputs = len(outputs)
                previous_collisions = len(collisions)
                replay_timestamp_ns = 0
                for _ in range(8):
                    replay_timestamp_ns = node.get_clock().now().nanoseconds
                    messages[0].header.stamp = node.get_clock().now().to_msg()
                    for publisher, message in zip(
                        publishers, (lanelet_map, *messages, pointcloud)
                    ):
                        publisher.publish(message)
                    spin_for(node, 0.04)
                    path_pub.publish(path)
                    spin_for(node, 0.12)
                    if len(outputs) > previous_outputs:
                        break
                if len(outputs) <= previous_outputs:
                    raise RuntimeError(f"연속 재생 {len(result)}번 프레임의 출력이 없습니다")
                result.append({
                    "bag_timestamp": timestamp,
                    "replay_timestamp_ns": replay_timestamp_ns,
                    "ego": {
                        "x": messages[0].pose.pose.position.x,
                        "y": messages[0].pose.pose.position.y,
                        "velocity_mps": messages[0].twist.twist.linear.x,
                        "acceleration_mps2": messages[1].accel.accel.linear.x,
                    },
                    "input": record([point.point for point in path.points]),
                    "output": record(outputs[-1].points),
                    "collision_info": [message.data
                                       for message in collisions[previous_collisions:]],
                    "objects": [record_object(obj) for obj in messages[2].objects],
                    "object_pose_shift_x": (
                        args.object_pose_shift_x_after_frame[1]
                        if args.object_pose_shift_x_after_frame and
                        frame_index >= args.object_pose_shift_x_after_frame[0]
                        else 0.0),
                })
            args.output.write_text(
                json.dumps(result, ensure_ascii=False, indent=2) + "\n")
            print(f"연속 재생 {len(result)}프레임, "
                  f"충돌 진단 {sum(bool(f['collision_info']) for f in result)}프레임")
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
