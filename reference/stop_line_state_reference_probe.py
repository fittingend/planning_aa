"""K-City 경로에 합성 정지 표지를 넣고 원본의 정차 상태 전환을 재생합니다."""

import argparse
import json
import os
import signal
import subprocess
from pathlib import Path

import rclpy
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--map-bin-file", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--module", choices=("stop_line", "merge_from_private"),
                        default="stop_line")
    parser.add_argument("--sample-fraction", type=float, default=0.35)
    parser.add_argument("--ego-path-index", type=int, default=5)
    args = parser.parse_args()
    timestamp, path_bytes = load(
        args.bag,
        "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id",
        args.sample_fraction)
    path = deserialize_message(path_bytes, PathWithLaneId)
    lanelet_map = deserialize_message(args.map_bin_file.read_bytes(), LaneletMapBin)
    topics = (
        ("/localization/kinematic_state", Odometry),
        ("/localization/acceleration", AccelWithCovarianceStamped),
        ("/perception/object_recognition/objects", PredictedObjects),
        ("/perception/occupancy_grid_map/map", OccupancyGrid),
    )
    messages = [deserialize_message(load(args.bag, topic, near=timestamp)[1], kind)
                for topic, kind in topics]
    pointcloud = PointCloud2()
    pointcloud.header.frame_id = "map"
    log_path = args.output.with_suffix(".log")
    with log_path.open("w") as log:
        process = subprocess.Popen(
            ["ros2", "launch", "/probe/reference/stop_line_planner_probe.launch.xml",
             "module:=[autoware::behavior_velocity_planner::StopLineModulePlugin]"
             if args.module == "stop_line" else
             "module:=[autoware::behavior_velocity_planner::IntersectionModulePlugin, "
             "autoware::behavior_velocity_planner::MergeFromPrivateModulePlugin]"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Node("stop_line_state_reference_probe")
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
        node.create_subscription(PlanningPath, "/probe/output_path", outputs.append, 10)
        frames = []
        try:
            spin_for(node, 3.0)
            for frame_index in range(7):
                odometry = messages[0]
                if frame_index > 0:
                    odometry.pose.pose = path.points[args.ego_path_index].point.pose
                    odometry.twist.twist.linear.x = 0.0
                previous = len(outputs)
                replay_timestamp_ns = 0
                for _ in range(8):
                    odometry.header.stamp = node.get_clock().now().to_msg()
                    for publisher, message in zip(
                            publishers, (lanelet_map, *messages, pointcloud)):
                        publisher.publish(message)
                    spin_for(node, 0.08)
                    replay_timestamp_ns = node.get_clock().now().nanoseconds
                    path_pub.publish(path)
                    spin_for(node, 0.15)
                    if len(outputs) > previous:
                        break
                if len(outputs) <= previous:
                    raise RuntimeError(f"원본 출력이 없는 프레임: {frame_index}")
                frames.append({
                    "replay_timestamp_ns": replay_timestamp_ns,
                    "ego": {
                        "x": odometry.pose.pose.position.x,
                        "y": odometry.pose.pose.position.y,
                        "velocity_mps": odometry.twist.twist.linear.x,
                        "acceleration_mps2": messages[1].accel.accel.linear.x,
                    },
                    "output": record(outputs[-1].points),
                })
                spin_for(node, 0.45)
            args.output.write_text(json.dumps({
                "bag_timestamp": timestamp,
                "module": args.module,
                "input_lane_ids": [[int(lane_id) for lane_id in point.lane_ids]
                                   for point in path.points],
                "map_bin_file": args.map_bin_file.name,
                "synthetic_regulatory_id": 900000007,
                "stop_line": {"x1": 2590.0, "y1": 23981.0,
                              "x2": 2602.0, "y2": 23981.0},
                "front_offset_m": 3.79,
                "input": record([point.point for point in path.points]),
                "frames": frames,
            }, ensure_ascii=False, indent=2) + "\n")
            print("정지 표지 연속 출력", len(frames), "프레임")
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
