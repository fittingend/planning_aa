#!/usr/bin/env python3
"""Inject a stationary pedestrian into an already routed KATECH AA stack."""

import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from autoware_perception_msgs.msg import (
    ObjectClassification, PredictedObject, PredictedObjects, Shape,
)
from autoware_planning_msgs.msg import LaneletRoute, Path, Trajectory
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--path-topic", default="")
    args = parser.parse_args()
    rclpy.init()
    node = Node("planning_aa_static_avoidance_probe")
    routes = []
    odometry = []
    trajectories = []
    paths = []
    node.create_subscription(
        LaneletRoute, "/planning/mission_planning/route",
        lambda message: routes.append(message),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    node.create_subscription(
        Odometry, "/localization/kinematic_state",
        lambda message: odometry.append(message), 10,
    )
    node.create_subscription(
        Trajectory, "/planning/scenario_planning/trajectory",
        lambda message: trajectories.append(message), 10,
    )
    if args.path_topic:
        node.create_subscription(
            Path, args.path_topic, lambda message: paths.append(message), 10,
        )
    publisher = node.create_publisher(
        PredictedObjects, "/perception/object_recognition/objects", 10,
    )
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not (
                routes and trajectories and odometry and
                (not args.path_topic or paths)):
            rclpy.spin_once(node, timeout_sec=0.1)
        if not routes or not trajectories or not odometry or (
                args.path_topic and not paths):
            raise RuntimeError("route, odometry and baseline paths must be active")
        route_uuid = bytes(routes[-1].uuid.uuid)
        baseline = trajectories[-1]
        trajectories.clear()
        baseline_path = paths[-1] if paths else None
        paths.clear()
        seen = 0
        max_stationary_endpoint_shift_m = 0.0
        max_stationary_path_shift_m = 0.0
        max_ego_displacement_m = 0.0
        minimum_center_distance_m = math.inf
        passed_object_line = False
        origin = (
            odometry[-1].pose.pose.position.x,
            odometry[-1].pose.pose.position.y,
        )
        object_direction = (args.x - origin[0], args.y - origin[1])
        object_direction_squared = sum(value * value for value in object_direction)
        start = time.monotonic()
        while time.monotonic() - start < args.duration:
            message = PredictedObjects()
            message.header.frame_id = "map"
            message.header.stamp = node.get_clock().now().to_msg()
            pedestrian = PredictedObject()
            pedestrian.object_id.uuid = [42] + [0] * 15
            pedestrian.existence_probability = 1.0
            classification = ObjectClassification()
            classification.label = ObjectClassification.PEDESTRIAN
            classification.probability = 1.0
            pedestrian.classification = [classification]
            pedestrian.shape.type = Shape.BOUNDING_BOX
            pedestrian.shape.dimensions.x = 0.7
            pedestrian.shape.dimensions.y = 0.7
            pedestrian.shape.dimensions.z = 1.7
            pose = pedestrian.kinematics.initial_pose_with_covariance.pose
            pose.position.x = args.x
            pose.position.y = args.y
            pose.orientation.w = 1.0
            message.objects = [pedestrian]
            publisher.publish(message)
            seen += 1
            rclpy.spin_once(node, timeout_sec=0.1)
            if odometry:
                pose = odometry[-1].pose.pose.position
                displacement = math.hypot(pose.x - origin[0], pose.y - origin[1])
                max_ego_displacement_m = max(max_ego_displacement_m, displacement)
                minimum_center_distance_m = min(
                    minimum_center_distance_m,
                    math.hypot(pose.x - args.x, pose.y - args.y),
                )
                if object_direction_squared > 0.0:
                    passed_object_line |= (
                        (pose.x - args.x) * object_direction[0] +
                        (pose.y - args.y) * object_direction[1]
                    ) > 0.0
            if (max_ego_displacement_m < 0.5 and trajectories and
                    baseline.points and trajectories[-1].points):
                first = baseline.points[-1].pose.position
                last = trajectories[-1].points[-1].pose.position
                max_stationary_endpoint_shift_m = max(
                    max_stationary_endpoint_shift_m,
                    math.hypot(last.x - first.x, last.y - first.y),
                )
            if (max_ego_displacement_m < 0.5 and baseline_path and paths and
                    baseline_path.points and paths[-1].points):
                nearest = lambda path: min(
                    path.points,
                    key=lambda point: math.hypot(
                        point.pose.position.x - args.x,
                        point.pose.position.y - args.y),
                ).pose.position
                first = nearest(baseline_path)
                last = nearest(paths[-1])
                max_stationary_path_shift_m = max(
                    max_stationary_path_shift_m,
                    math.hypot(last.x - first.x, last.y - first.y),
                )
        near = lambda trajectory: [
            (point.pose.position.x, point.pose.position.y,
             point.longitudinal_velocity_mps)
            for point in trajectory.points
            if math.hypot(point.pose.position.x - args.x,
                          point.pose.position.y - args.y) < 12.0
        ]
        def closest(trajectory):
            if not trajectory.points:
                return None
            point = min(trajectory.points,
                        key=lambda item: math.hypot(
                            item.pose.position.x - args.x,
                            item.pose.position.y - args.y))
            return [point.pose.position.x, point.pose.position.y,
                    point.longitudinal_velocity_mps]
        def velocity_at_ego(trajectory):
            if not trajectory.points or not odometry:
                return None
            ego = odometry[-1].pose.pose.position
            point = min(trajectory.points,
                        key=lambda item: math.hypot(
                            item.pose.position.x - ego.x,
                            item.pose.position.y - ego.y))
            return point.longitudinal_velocity_mps
        summary = {
            "published_objects": seen,
            "route_unchanged": bool(routes) and bytes(routes[-1].uuid.uuid) == route_uuid,
            "received_trajectories": len(trajectories),
            "max_stationary_endpoint_shift_m": round(
                max_stationary_endpoint_shift_m, 3),
            "max_stationary_path_shift_m": round(
                max_stationary_path_shift_m, 3),
            "received_behavior_paths": len(paths),
            "max_ego_displacement_m": round(max_ego_displacement_m, 3),
            "minimum_ego_object_center_distance_m": round(
                minimum_center_distance_m, 3),
            "passed_object_line": passed_object_line,
            "baseline_near_count": len(near(baseline)),
            "last_near_count": len(near(trajectories[-1])) if trajectories else 0,
            "baseline_closest": closest(baseline),
            "last_closest": closest(trajectories[-1]) if trajectories else None,
            "baseline_velocity_at_ego_mps": velocity_at_ego(baseline),
            "last_velocity_at_ego_mps": velocity_at_ego(trajectories[-1])
                if trajectories else None,
            "baseline_max_velocity_mps": max(
                (point.longitudinal_velocity_mps for point in baseline.points),
                default=None),
            "last_odometry": [odometry[-1].pose.pose.position.x,
                              odometry[-1].pose.pose.position.y]
                if odometry else None,
        }
        print(json.dumps(summary), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
