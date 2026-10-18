#!/usr/bin/env python3
"""Drive the same stationary start/goal through either planning stack."""

import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from autoware_planning_msgs.msg import LaneletRoute, Trajectory
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tier4_planning_msgs.srv import SetWaypointRoute
from tier4_planning_msgs.msg import RouteState


def spin_until(node, predicate, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def pose_xy(pose):
    return [pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--start-x", type=float, default=3648.0)
    parser.add_argument("--start-y", type=float, default=73505.0)
    parser.add_argument("--goal-x", type=float, default=3632.5)
    parser.add_argument("--goal-y", type=float, default=73628.0)
    parser.add_argument("--output", required=True)
    parser.add_argument("--wait", type=float, default=35.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node("planning_runtime_compare_probe")
    odometry = []
    routes = []
    route_states = []
    trajectories = []
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    init_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose3d", 10)
    node.create_subscription(Odometry, "/localization/kinematic_state",
                             lambda msg: odometry.append(msg), 10)
    node.create_subscription(LaneletRoute, "/planning/mission_planning/route",
                             lambda msg: routes.append(msg), durable)
    node.create_subscription(RouteState,
        "/planning/mission_planning/route_selector/main/state",
        lambda msg: route_states.append(msg.state), durable)
    node.create_subscription(Trajectory, "/planning/scenario_planning/trajectory",
                             lambda msg: trajectories.append(msg), 10)
    client = node.create_client(SetWaypointRoute,
        "/planning/mission_planning/route_selector/main/set_waypoint_route")
    result = {"input": vars(args), "initialization": False,
              "service_ready": False, "route_unset": False,
              "route_states": [], "response": None, "odometry": [],
              "routes": [], "trajectories": []}
    try:
        # The sample map has MGRS-local coordinates. These points lie on the
        # northbound lanelet 102 and its successor 87.
        yaw = math.atan2(73604.9 - 73496.1, 3635.7 - 3649.6)
        init = PoseWithCovarianceStamped()
        init.header.frame_id = "map"
        init.pose.pose.position.x = args.start_x
        init.pose.pose.position.y = args.start_y
        init.pose.pose.position.z = 19.0
        init.pose.pose.orientation.z = math.sin(yaw / 2)
        init.pose.pose.orientation.w = math.cos(yaw / 2)
        for _ in range(15):
            init.header.stamp = node.get_clock().now().to_msg()
            init_pub.publish(init)
            if spin_until(node, lambda: bool(odometry), 1.0):
                break
        result["initialization"] = bool(odometry)
        result["service_ready"] = spin_until(node, client.service_is_ready, 20)
        result["route_unset"] = spin_until(
            node, lambda: bool(route_states) and route_states[-1] == 2, 30)
        if result["initialization"] and result["service_ready"] and result["route_unset"]:
            request = SetWaypointRoute.Request()
            request.header.frame_id = "map"
            request.header.stamp = node.get_clock().now().to_msg()
            request.goal_pose.position.x = args.goal_x
            request.goal_pose.position.y = args.goal_y
            request.goal_pose.position.z = 19.0
            request.goal_pose.orientation.z = init.pose.pose.orientation.z
            request.goal_pose.orientation.w = init.pose.pose.orientation.w
            future = client.call_async(request)
            if spin_until(node, future.done, 20):
                status = future.result().status
                result["response"] = {"success": status.success,
                                      "code": status.code, "message": status.message}
                spin_until(node, lambda: len(trajectories) >= 3, args.wait)
                spin_until(node, lambda: bool(routes), 5)
                spin_until(node, lambda: 4 in route_states, 5)
        result["odometry"] = [pose_xy(msg.pose.pose) for msg in odometry[:3]]
        result["route_states"] = route_states[:10]
        result["routes"] = [{
            "start": pose_xy(msg.start_pose), "goal": pose_xy(msg.goal_pose),
            "segments": [{"preferred": segment.preferred_primitive.id,
                          "primitives": [primitive.id for primitive in segment.primitives]}
                         for segment in msg.segments],
            "allow_modification": msg.allow_modification,
            "uuid": [int(value) for value in msg.uuid.uuid]} for msg in routes[:3]]
        for msg in trajectories[:3]:
            result["trajectories"].append({
                "frame": msg.header.frame_id,
                "points": [{"pose": pose_xy(point.pose),
                            "longitudinal_velocity_mps": point.longitudinal_velocity_mps,
                            "lateral_velocity_mps": point.lateral_velocity_mps,
                            "acceleration_mps2": point.acceleration_mps2,
                            "heading_rate_rps": point.heading_rate_rps,
                            "front_wheel_angle_rad": point.front_wheel_angle_rad,
                            "rear_wheel_angle_rad": point.rear_wheel_angle_rad,
                            "time_from_start_ns": point.time_from_start.sec * 1000000000 +
                                                  point.time_from_start.nanosec}
                           for point in msg.points]})
    finally:
        with open(args.output, "w", encoding="utf-8") as file:
            json.dump(result, file, indent=2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
