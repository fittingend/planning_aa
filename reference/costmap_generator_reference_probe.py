"""기준 costmap generator에 실제 지도와 합성 주차 활성 신호를 넣어 격자를 수집합니다."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from autoware_map_msgs.msg import LaneletMapBin
from autoware_perception_msgs.msg import (ObjectClassification,
                                          PredictedObject, PredictedObjects,
                                          Shape)
from geometry_msgs.msg import Point32, TransformStamped
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from autoware_internal_debug_msgs.msg import Float64Stamped
from tier4_debug_msgs.msg import ProcessingTimeTree
from tier4_planning_msgs.msg import Scenario
from tf2_ros import StaticTransformBroadcaster

from stop_line_planner_reference_probe import load


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--with-object", action="store_true",
                        help="지도 위 합성 bounding box 객체 1개를 발행합니다")
    parser.add_argument("--object-offset-x", type=float, default=0.0)
    parser.add_argument("--object-shape", choices=("box", "cylinder", "polygon"),
                        default="box")
    parser.add_argument("--object-yaw-deg", type=float, default=0.0)
    parser.add_argument("--object-probability", type=float, default=0.8)
    parser.add_argument("--polygon-asymmetric", action="store_true")
    parser.add_argument("--with-point", action="store_true",
                        help="지도 위 합성 비지면 점 1개를 발행합니다")
    parser.add_argument("--point-z-offset", type=float, default=1.0)
    parser.add_argument("--second-point-x-offset", type=float,
                        help="두 번째 합성 점의 첫 점 기준 X 거리(m)")
    parser.add_argument("--observe-seconds", type=float, default=0.0,
                        help="첫 출력 뒤 발행 간격을 관찰할 시간")
    parser.add_argument("--metadata-only", action="store_true")
    parser.add_argument("--inactive-observation", action="store_true",
                        help="차로 주행 Scenario에서 격자 미발행과 처리 시간 발행을 관측합니다")
    parser.add_argument("--scenario-transition-observation", action="store_true",
                        help="주차 활성·비활성·재활성 발행 경계를 관측합니다")
    args = parser.parse_args()
    if args.inactive_observation and args.scenario_transition_observation:
        parser.error("비활성 단독 관측과 전환 관측은 함께 사용할 수 없습니다")
    if args.second_point_x_offset is not None and not args.with_point:
        parser.error("--second-point-x-offset에는 --with-point가 필요합니다")
    _, map_bytes = load(args.bag, "/map/vector_map")
    _, odom_bytes = load(args.bag, "/localization/kinematic_state")
    lanelet_map = deserialize_message(map_bytes, LaneletMapBin)
    odometry = deserialize_message(odom_bytes, Odometry)
    log_path = args.output.with_suffix(".log")
    with log_path.open("w") as log:
        process = subprocess.Popen(
            ["ros2", "launch", "/probe/reference/costmap_generator_probe.launch.xml",
             f"with_objects:={str(args.with_object).lower()}",
             f"with_points:={str(args.with_point).lower()}"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Node("costmap_generator_reference_probe")
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        map_pub = node.create_publisher(LaneletMapBin, "/probe/map", durable)
        scenario_pub = node.create_publisher(Scenario, "/probe/scenario", 1)
        object_pub = node.create_publisher(PredictedObjects, "/probe/objects", 1)
        point_pub = node.create_publisher(PointCloud2, "/probe/points",
                                          qos_profile_sensor_data)
        occupancy = []
        grid_maps = []
        processing_times = []
        processing_trees = []
        occupancy_receipts = []
        grid_map_receipts = []
        def receive_occupancy(msg):
            occupancy.append(msg)
            occupancy_receipts.append(time.monotonic_ns())
        def receive_grid_map(msg):
            grid_maps.append(msg)
            grid_map_receipts.append(time.monotonic_ns())
        node.create_subscription(OccupancyGrid, "/probe/occupancy_grid",
                                 receive_occupancy, 10)
        node.create_subscription(GridMap, "/probe/grid_map",
                                 receive_grid_map, 10)
        node.create_subscription(Float64Stamped,
                                 "/probe/costmap_generator/debug/processing_time_ms",
                                 lambda msg: processing_times.append(msg), 10)
        node.create_subscription(ProcessingTimeTree, "/probe/processing_time",
                                 lambda msg: processing_trees.append(msg), 10)
        static_tf = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = odometry.pose.pose.position.x
        transform.transform.translation.y = odometry.pose.pose.position.y
        transform.transform.translation.z = odometry.pose.pose.position.z
        transform.transform.rotation = odometry.pose.pose.orientation
        scenario = Scenario()
        scenario.current_scenario = (Scenario.LANEDRIVING if args.inactive_observation
                                     else Scenario.PARKING)
        scenario.activating_scenarios = [scenario.current_scenario]
        objects = PredictedObjects()
        objects.header.frame_id = "map"
        point = [odometry.pose.pose.position.x + 0.1,
                 odometry.pose.pose.position.y + 0.1,
                 odometry.pose.pose.position.z + args.point_z_offset]
        second_point = ([point[0] + args.second_point_x_offset,
                         point[1], point[2]]
                        if args.second_point_x_offset is not None else None)
        point_message = point_cloud2.create_cloud_xyz32(
            odometry.header, [point] + ([second_point] if second_point else []))
        point_message.header.frame_id = "map"
        if args.with_object:
            obj = PredictedObject()
            obj.object_id.uuid = [1] + [0] * 15
            obj.classification = [ObjectClassification(
                label=ObjectClassification.CAR,
                probability=args.object_probability)]
            obj.shape.type = {"box": Shape.BOUNDING_BOX,
                              "cylinder": Shape.CYLINDER,
                              "polygon": Shape.POLYGON}[args.object_shape]
            obj.shape.dimensions.x = 4.0
            obj.shape.dimensions.y = 2.0
            obj.shape.dimensions.z = 1.5
            if args.object_shape == "polygon":
                outline = ((2.0, 1.0), (2.0, -1.0),
                           (-2.0, -0.5), (-2.0, 0.5)) if args.polygon_asymmetric else (
                               (2.0, 1.0), (2.0, -1.0),
                               (-2.0, -1.0), (-2.0, 1.0))
                obj.shape.footprint.points = [
                    Point32(x=x, y=y) for x, y in outline]
            obj.kinematics.initial_pose_with_covariance.pose.position.x = (
                odometry.pose.pose.position.x + args.object_offset_x)
            obj.kinematics.initial_pose_with_covariance.pose.position.y = (
                odometry.pose.pose.position.y)
            obj.kinematics.initial_pose_with_covariance.pose.orientation.z = (
                math.sin(math.radians(args.object_yaw_deg) * 0.5))
            obj.kinematics.initial_pose_with_covariance.pose.orientation.w = (
                math.cos(math.radians(args.object_yaw_deg) * 0.5))
            objects.objects = [obj]
        try:
            if args.scenario_transition_observation:
                def publish_for(duration, scenario_value):
                    scenario.current_scenario = scenario_value
                    scenario.activating_scenarios = [scenario_value]
                    deadline = time.monotonic() + duration
                    while time.monotonic() < deadline:
                        transform.header.stamp = node.get_clock().now().to_msg()
                        static_tf.sendTransform(transform)
                        map_pub.publish(lanelet_map)
                        scenario_pub.publish(scenario)
                        rclpy.spin_once(node, timeout_sec=0.05)
                publish_for(2.0, Scenario.PARKING)
                active_before = (len(occupancy), len(grid_maps))
                publish_for(0.4, Scenario.LANEDRIVING)
                inactive_start = (len(occupancy), len(grid_maps))
                publish_for(0.9, Scenario.LANEDRIVING)
                inactive_end = (len(occupancy), len(grid_maps))
                publish_for(1.2, Scenario.PARKING)
                active_after = (len(occupancy), len(grid_maps))
                observation = {
                    "source": str(args.bag),
                    "scenario_sequence": ["PARKING", "LANEDRIVING", "PARKING"],
                    "active_before": active_before,
                    "inactive_start_after_settling": inactive_start,
                    "inactive_end": inactive_end,
                    "active_after": active_after,
                    "processing_time_count": len(processing_times),
                    "processing_tree_count": len(processing_trees),
                }
                args.output.write_text(json.dumps(observation))
                if (min(active_before) < 1 or inactive_start != inactive_end or
                        any(b <= a for a, b in zip(inactive_end, active_after))):
                    raise AssertionError(observation)
                return
            if args.inactive_observation:
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    map_pub.publish(lanelet_map)
                    scenario_pub.publish(scenario)
                    rclpy.spin_once(node, timeout_sec=0.05)
                observation = {
                    "source": str(args.bag),
                    "scenario": "LANE_DRIVING (합성 활성)",
                    "occupancy_count": len(occupancy),
                    "grid_map_count": len(grid_maps),
                    "processing_time_count": len(processing_times),
                    "processing_tree_count": len(processing_trees),
                }
                args.output.write_text(json.dumps(observation))
                if occupancy or grid_maps or not processing_times or not processing_trees:
                    raise AssertionError(observation)
                return
            deadline = time.monotonic() + 25.0
            while (time.monotonic() < deadline and
                   (not occupancy or not grid_maps)):
                transform.header.stamp = node.get_clock().now().to_msg()
                static_tf.sendTransform(transform)
                map_pub.publish(lanelet_map)
                scenario_pub.publish(scenario)
                if args.with_object:
                    objects.header.stamp = node.get_clock().now().to_msg()
                    object_pub.publish(objects)
                if args.with_point:
                    point_message.header.stamp = node.get_clock().now().to_msg()
                    point_pub.publish(point_message)
                rclpy.spin_once(node, timeout_sec=0.1)
            if not occupancy or not grid_maps:
                raise RuntimeError(f"원본 격자 출력이 없습니다. 로그: {log_path}")
            deadline = time.monotonic() + args.observe_seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            grid = occupancy[-1]
            layer_map = grid_maps[-1]
            output_topics = {
                name: {
                    "types": types,
                    "publisher_durability": [
                        str(info.qos_profile.durability)
                        for info in node.get_publishers_info_by_topic(name)],
                    "publisher_reliability": [
                        str(info.qos_profile.reliability)
                        for info in node.get_publishers_info_by_topic(name)],
                    "publisher_history": [
                        str(info.qos_profile.history)
                        for info in node.get_publishers_info_by_topic(name)],
                    "publisher_depth": [
                        info.qos_profile.depth
                        for info in node.get_publishers_info_by_topic(name)],
                }
                for name, types in node.get_topic_names_and_types()
                if name in {
                    "/probe/occupancy_grid", "/probe/grid_map",
                    "/probe/processing_time",
                    "/probe/costmap_generator/debug/processing_time_ms",
                }
            }
            result = {
                "source": str(args.bag),
                "input": {
                    "ego_x": odometry.pose.pose.position.x,
                    "ego_y": odometry.pose.pose.position.y,
                    "ego_z": odometry.pose.pose.position.z,
                    "scenario": "PARKING (합성 활성)",
                    "objects": ({"count": 1, "classification": "CAR",
                                 "probability": args.object_probability,
                                 "shape": ("BOUNDING_BOX" if args.object_shape == "box"
                                           else args.object_shape.upper()),
                                 "x": obj.kinematics.initial_pose_with_covariance.pose.position.x,
                                 "y": obj.kinematics.initial_pose_with_covariance.pose.position.y,
                                 "length_m": 4.0, "width_m": 2.0,
                                 "yaw_rad": math.radians(args.object_yaw_deg),
                                 "polygon_asymmetric": args.polygon_asymmetric}
                                if args.with_object else "입력 없음"),
                    "points": ({"count": 1 + int(second_point is not None),
                                "x": point[0], "y": point[1], "z": point[2],
                                **({"x2": second_point[0],
                                    "y2": second_point[1],
                                    "z2": second_point[2]}
                                   if second_point else {})}
                               if args.with_point else "입력 없음"),
                },
                "output": {
                    "frame_id": grid.header.frame_id,
                    "resolution_m": grid.info.resolution,
                    "width": grid.info.width,
                    "height": grid.info.height,
                    "origin_x": grid.info.origin.position.x,
                    "origin_y": grid.info.origin.position.y,
                    "origin_z": grid.info.origin.position.z,
                    "data": list(grid.data),
                },
                "grid_map": {
                    "frame_id": layer_map.header.frame_id,
                    "resolution_m": layer_map.info.resolution,
                    "length_x_m": layer_map.info.length_x,
                    "length_y_m": layer_map.info.length_y,
                    "center_x": layer_map.info.pose.position.x,
                    "center_y": layer_map.info.pose.position.y,
                    "center_z": layer_map.info.pose.position.z,
                    "layers": {name: list(data.data) for name, data in
                               zip(layer_map.layers, layer_map.data)},
                    "outer_start_index": layer_map.outer_start_index,
                    "inner_start_index": layer_map.inner_start_index,
                },
                "graph": output_topics,
                "observation": {
                    "occupancy_count": len(occupancy),
                    "grid_map_count": len(grid_maps),
                    "occupancy_stamps_ns": [
                        message.header.stamp.sec * 1_000_000_000 +
                        message.header.stamp.nanosec for message in occupancy],
                    "grid_map_stamps_ns": [
                        message.header.stamp.sec * 1_000_000_000 +
                        message.header.stamp.nanosec for message in grid_maps],
                    "occupancy_stamp_ns": (grid.header.stamp.sec * 1_000_000_000 +
                                           grid.header.stamp.nanosec),
                    "grid_map_stamp_ns": (
                        layer_map.header.stamp.sec * 1_000_000_000 +
                        layer_map.header.stamp.nanosec),
                    "source_odometry_stamp_ns": (
                        odometry.header.stamp.sec * 1_000_000_000 +
                        odometry.header.stamp.nanosec),
                    "occupancy_intervals_ms": [
                        (b - a) / 1e6 for a, b in
                        zip(occupancy_receipts, occupancy_receipts[1:])],
                    "grid_map_intervals_ms": [
                        (b - a) / 1e6 for a, b in
                        zip(grid_map_receipts, grid_map_receipts[1:])],
                },
            }
            if args.metadata_only:
                del result["output"]
                del result["grid_map"]
            args.output.write_text(json.dumps(result))
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
