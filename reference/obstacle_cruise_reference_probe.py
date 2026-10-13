"""기록된 연속 입력을 원본 obstacle cruise 노드에 같은 시계로 발행합니다."""

import argparse
import bisect
import copy
import json
import math
import os
from pathlib import Path
from collections import Counter
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelWithCovarianceStamped, Point32
from autoware_perception_msgs.msg import PredictedObjects
from autoware_perception_msgs.msg import PredictedPath
from autoware_planning_msgs.msg import Trajectory
from autoware_planning_msgs.msg import TrajectoryPoint
from autoware_internal_debug_msgs.msg import Float32MultiArrayStamped
from visualization_msgs.msg import MarkerArray

from offline_rosbag_cdr import decode_trajectory, messages


PREFIX = "/planning/scenario_planning/lane_driving/motion_planning/"


def load_valid_frames(bag, start, end, first_input_stamp):
    frames = []
    for stamp, payload in messages(bag, PREFIX + "motion_velocity_planner/trajectory"):
        if len(decode_trajectory(payload)["points"]) < 2:
            continue
        if stamp < first_input_stamp:
            continue
        frames.append((stamp, payload))
        if len(frames) >= end:
            break
    return frames[start:]


def nearest_prior(series, stamps, stamp):
    index = bisect.bisect_right(stamps, stamp) - 1
    return series[index][1] if index >= 0 else None


def synthetic_crossing_pedestrian(odometry, objects, trajectory,
                                  crossing_x, noncrossing_high_confidence,
                                  classification, curved_trajectory,
                                  polygon_footprint, disappear):
    """원본 노드의 차로 밖 보행자 분기를 확인하는 직선 합성 입력."""
    odometry.pose.pose.position.x = 0.0
    odometry.pose.pose.position.y = 0.0
    odometry.pose.pose.orientation.x = 0.0
    odometry.pose.pose.orientation.y = 0.0
    odometry.pose.pose.orientation.z = 0.0
    odometry.pose.pose.orientation.w = 1.0
    odometry.twist.twist.linear.x = 5.0
    trajectory.points = []
    for index in range(101):
        point = TrajectoryPoint()
        point.pose.position.x = float(index)
        if curved_trajectory:
            point.pose.position.y = 0.005 * index * index
            yaw = math.atan(0.01 * index)
            point.pose.orientation.z = math.sin(yaw * 0.5)
            point.pose.orientation.w = math.cos(yaw * 0.5)
        else:
            point.pose.orientation.w = 1.0
        point.longitudinal_velocity_mps = 5.0
        trajectory.points.append(point)
    if not objects.objects:
        raise ValueError("합성 검증에 사용할 PredictedObject 틀이 없습니다")
    obstacle = copy.deepcopy(objects.objects[0])
    obstacle.classification = obstacle.classification[:1]
    obstacle.classification[0].label = classification
    obstacle.classification[0].probability = 1.0
    obstacle.shape.type = 0  # BOUNDING_BOX
    obstacle.shape.dimensions.x = 4.0 if classification == 1 else 0.6
    obstacle.shape.dimensions.y = 2.0 if classification == 1 else 0.6
    if polygon_footprint:
        obstacle.shape.type = 2  # POLYGON
        obstacle.shape.footprint.points = [
            Point32(x=0.0, y=-0.3), Point32(x=0.4, y=0.0),
            Point32(x=0.0, y=0.3)]
    obstacle.kinematics.initial_pose_with_covariance.pose.position.x = crossing_x
    center_y = 0.005 * crossing_x * crossing_x if curved_trajectory else 0.0
    obstacle.kinematics.initial_pose_with_covariance.pose.position.y = center_y + 3.0
    obstacle.kinematics.initial_pose_with_covariance.pose.orientation.x = 0.0
    obstacle.kinematics.initial_pose_with_covariance.pose.orientation.y = 0.0
    obstacle.kinematics.initial_pose_with_covariance.pose.orientation.z = 0.0
    obstacle.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0
    obstacle.kinematics.initial_twist_with_covariance.twist.linear.x = 0.0
    obstacle.kinematics.initial_twist_with_covariance.twist.linear.y = -2.0
    predicted = PredictedPath()
    predicted.confidence = 1.0
    predicted.time_step.sec = 1
    predicted.path = []
    for y in (3.0, 1.0, -1.0):
        pose = copy.deepcopy(obstacle.kinematics.initial_pose_with_covariance.pose)
        pose.position.y = center_y + y
        predicted.path.append(pose)
    if noncrossing_high_confidence:
        ignored = copy.deepcopy(predicted)
        ignored.confidence = 2.0
        for pose in ignored.path:
            pose.position.y = center_y + 3.0
        obstacle.kinematics.predicted_paths = [ignored, predicted]
    else:
        obstacle.kinematics.predicted_paths = [predicted]
    if disappear:
        obstacle.kinematics.initial_pose_with_covariance.pose.position.y = 20.0
        obstacle.kinematics.initial_twist_with_covariance.twist.linear.y = 0.0
        for path in obstacle.kinematics.predicted_paths:
            for pose in path.path:
                pose.position.y = 20.0
    objects.objects = [obstacle]
    return odometry, objects, trajectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--start", type=int, default=478)
    parser.add_argument("--end", type=int, default=483)
    parser.add_argument("--safe-distance-margin", type=float, default=4.0)
    parser.add_argument("--cruise-to-stop-velocity", type=float, default=1.0)
    parser.add_argument("--stop-to-cruise-velocity", type=float, default=1.5)
    parser.add_argument("--collision-time-margin", type=float, default=1.0)
    parser.add_argument("--enable-outside-stop", action="store_true")
    parser.add_argument("--outside-stop-predicted-path-count", type=int,
                        default=1)
    parser.add_argument("--synthetic-crossing-pedestrian", action="store_true")
    parser.add_argument("--synthetic-crossing-x", type=float, default=15.0)
    parser.add_argument("--synthetic-classification", type=int,
                        choices=(1, 6, 7), default=7)
    parser.add_argument("--synthetic-curved-trajectory", action="store_true")
    parser.add_argument("--synthetic-polygon-footprint", action="store_true")
    parser.add_argument("--synthetic-disappear-after-first", action="store_true")
    parser.add_argument("--synthetic-slowdown-object", action="store_true")
    parser.add_argument("--synthetic-slowdown-longitudinal-velocity", type=float,
                        default=0.0)
    parser.add_argument("--synthetic-slowdown-exit-after", type=int, default=-1)
    parser.add_argument("--synthetic-slowdown-exit-y", type=float, default=2.48)
    parser.add_argument("--synthetic-noncrossing-high-confidence-path",
                        action="store_true")
    parser.add_argument("--capture-marker-geometry", action="store_true")
    args = parser.parse_args()
    streams = {
        "odometry": list(messages(args.bag, "/localization/kinematic_state")),
        "acceleration": list(messages(args.bag, "/localization/acceleration")),
        "objects": list(messages(args.bag, "/perception/object_recognition/objects")),
    }
    stamps = {name: [stamp for stamp, _ in rows]
              for name, rows in streams.items()}
    first_input_stamp = max(stamps["odometry"][0], stamps["objects"][0])
    frames = load_valid_frames(args.bag, args.start, args.end,
                               first_input_stamp)
    log_path = args.output.with_suffix(".log")
    with log_path.open("w") as log:
        process = subprocess.Popen(
            ["ros2", "launch", "/probe/reference/obstacle_cruise_probe.launch.xml",
             f"safe_distance_margin:={args.safe_distance_margin}",
             f"cruise_to_stop_velocity:={args.cruise_to_stop_velocity}",
             f"stop_to_cruise_velocity:={args.stop_to_cruise_velocity}",
             f"collision_time_margin:={args.collision_time_margin}",
             f"enable_outside_stop:={'true' if args.enable_outside_stop else 'false'}",
             f"outside_stop_predicted_path_count:={args.outside_stop_predicted_path_count}"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Node("obstacle_cruise_reference_probe")
        clock = node.create_publisher(Clock, "/clock", 10)
        pubs = {
            "odometry": node.create_publisher(Odometry, "/probe/odometry", 10),
            "acceleration": node.create_publisher(
                AccelWithCovarianceStamped, "/probe/acceleration", 10),
            "objects": node.create_publisher(PredictedObjects, "/probe/objects", 10),
            "trajectory": node.create_publisher(Trajectory, "/probe/trajectory", 10),
        }
        results = []
        debug = []
        debug_stamps = []
        cruise_debug = []
        slowdown_debug = []
        trajectories = []
        marker_sets = []
        def on_markers(msg):
            names = Counter(item.ns for item in msg.markers)
            leading = [item for item in msg.markers
                       if item.ns in ("obstacles_to_stop", "obstacles_to_cruise",
                                      "stop_collision_points",
                                      "slow_down_collision_points")]
            marker_sets.append({"counts": dict(names),
                                "leading": [{"namespace": item.ns,
                                             "x": item.pose.position.x,
                                             "y": item.pose.position.y}
                                            for item in leading[:3]],
                                "detection_points": [
                                    [point.x, point.y]
                                    for item in msg.markers
                                    if item.ns == "detection_polygons"
                                    for point in item.points]
                                    if args.capture_marker_geometry else None})
        def on_debug(msg):
            debug.append(list(msg.data))
            debug_stamps.append(msg.stamp.sec * 1_000_000_000 +
                                msg.stamp.nanosec)
        node.create_subscription(
            Float32MultiArrayStamped,
            "/probe/obstacle_cruise_planner/debug/stop_planning_info",
            on_debug, 10)
        node.create_subscription(
            Float32MultiArrayStamped,
            "/probe/obstacle_cruise_planner/debug/cruise_planning_info",
            lambda msg: cruise_debug.append(list(msg.data)), 10)
        node.create_subscription(
            Float32MultiArrayStamped,
            "/probe/obstacle_cruise_planner/debug/slow_down_planning_info",
            lambda msg: slowdown_debug.append(list(msg.data)), 10)
        node.create_subscription(Trajectory, "/probe/output",
                                 lambda msg: trajectories.append(len(msg.points)), 10)
        node.create_subscription(
            MarkerArray, "/probe/obstacle_cruise_planner/debug/marker",
            on_markers, 10)
        try:
            ready_deadline = time.monotonic() + 20.0
            while (time.monotonic() < ready_deadline and
                   node.count_subscribers("/probe/trajectory") == 0):
                rclpy.spin_once(node, timeout_sec=0.1)
            if node.count_subscribers("/probe/trajectory") == 0:
                raise RuntimeError(f"원본 노드가 궤적을 구독하지 않습니다: {log_path}")
            print("연결", {name: node.count_subscribers("/probe/" + name)
                         for name in pubs}, "정지 출력 publisher",
                  node.count_publishers(
                      "/probe/obstacle_cruise_planner/debug/stop_planning_info"),
                  flush=True)
            print("정지 출력 타입", [entry for entry in
                  node.get_topic_names_and_types() if "stop_planning_info" in entry[0]],
                  flush=True)
            for frame_index, (stamp, payload) in enumerate(frames, args.start):
                msg = Clock()
                msg.clock.sec, msg.clock.nanosec = divmod(stamp + 5_000_000,
                                                           1_000_000_000)
                odom = nearest_prior(streams["odometry"], stamps["odometry"], stamp)
                acc = nearest_prior(streams["acceleration"],
                                    stamps["acceleration"], stamp)
                objects = nearest_prior(streams["objects"], stamps["objects"], stamp)
                if any(item is None for item in (odom, acc, objects)):
                    raise RuntimeError("차량 상태 또는 객체 입력이 없습니다")
                odom_msg = deserialize_message(odom, Odometry)
                object_msg = deserialize_message(objects, PredictedObjects)
                trajectory_msg = deserialize_message(payload, Trajectory)
                if args.synthetic_crossing_pedestrian:
                    odom_msg, object_msg, trajectory_msg = (
                        synthetic_crossing_pedestrian(
                            odom_msg, object_msg, trajectory_msg,
                            args.synthetic_crossing_x,
                            args.synthetic_noncrossing_high_confidence_path,
                            args.synthetic_classification,
                            args.synthetic_curved_trajectory,
                            args.synthetic_polygon_footprint,
                            args.synthetic_disappear_after_first and
                            frame_index > args.start))
                    if args.synthetic_slowdown_object:
                        obstacle = object_msg.objects[0]
                        slowdown_y = (2.0 + 0.005 * args.synthetic_crossing_x ** 2
                                      if args.synthetic_curved_trajectory else 2.0)
                        if (args.synthetic_slowdown_exit_after >= 0 and
                                frame_index >= args.start + args.synthetic_slowdown_exit_after):
                            slowdown_y = args.synthetic_slowdown_exit_y
                        obstacle.kinematics.initial_pose_with_covariance.pose.position.y = slowdown_y
                        obstacle.kinematics.initial_twist_with_covariance.twist.linear.x = (
                            args.synthetic_slowdown_longitudinal_velocity)
                        obstacle.kinematics.initial_twist_with_covariance.twist.linear.y = 0.0
                        for predicted in obstacle.kinematics.predicted_paths:
                            for pose in predicted.path:
                                pose.position.y = slowdown_y
                for _ in range(5):
                    clock.publish(msg)
                    pubs["odometry"].publish(odom_msg)
                    pubs["acceleration"].publish(deserialize_message(
                        acc, AccelWithCovarianceStamped))
                    pubs["objects"].publish(object_msg)
                    rclpy.spin_once(node, timeout_sec=0.1)
                before = len(debug)
                before_cruise = len(cruise_debug)
                before_slowdown = len(slowdown_debug)
                pubs["trajectory"].publish(trajectory_msg)
                deadline = time.monotonic() + 8.0
                while time.monotonic() < deadline and len(debug) == before:
                    clock.publish(msg)
                    rclpy.spin_once(node, timeout_sec=0.1)
                for _ in range(5):
                    rclpy.spin_once(node, timeout_sec=0.05)
                results.append({"frame": frame_index, "record_stamp_ns": stamp,
                                "stop_debug": debug[-1] if len(debug) > before else None,
                                "cruise_debug": cruise_debug[-1]
                                    if len(cruise_debug) > before_cruise else None,
                                "slowdown_debug": slowdown_debug[-1]
                                    if len(slowdown_debug) > before_slowdown else None,
                                "debug_stamp_ns": debug_stamps[-1]
                                    if len(debug) > before else None,
                                "output_trajectory_count":
                                    trajectories[-1] if trajectories else None,
                                "markers": marker_sets[-1]
                                    if marker_sets else None})
            args.output.write_text(json.dumps(results, indent=2))
        finally:
            node.destroy_node()
            rclpy.shutdown()
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)


if __name__ == "__main__":
    main()
