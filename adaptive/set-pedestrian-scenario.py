#!/usr/bin/env python3
"""Set the proven RViz dummy pedestrian scenario in a running KATECH stack."""

import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from autoware_perception_msgs.msg import ObjectClassification, PredictedObjects, Shape
from autoware_planning_msgs.msg import LaneletRoute, Trajectory
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tier4_planning_msgs.msg import RouteState
from tier4_planning_msgs.srv import ClearRoute, SetWaypointRoute
from tier4_simulation_msgs.msg import DummyObject


def spin_until(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def call(node, client, request, name):
    if not spin_until(node, client.service_is_ready, 20):
        raise RuntimeError(f"{name} service is unavailable")
    future = client.call_async(request)
    if not spin_until(node, future.done, 20):
        raise RuntimeError(f"{name} service timed out")
    status = future.result().status
    if not status.success:
        raise RuntimeError(f"{name} failed: {status.code} {status.message}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start-x", type=float, default=3646.5)
    parser.add_argument("--start-y", type=float, default=73520.0)
    parser.add_argument("--goal-x", type=float, default=3632.5)
    parser.add_argument("--goal-y", type=float, default=73628.0)
    parser.add_argument("--ped-x", type=float, default=3645.6)
    parser.add_argument("--ped-y", type=float, default=73545.0)
    parser.add_argument("--yaw", type=float,
                        default=math.atan2(73604.9 - 73496.1,
                                           3635.7 - 3649.6),
                        help="initial heading in radians")
    args = parser.parse_args()

    rclpy.init()
    node = Node("set_pedestrian_scenario")
    odometry = []
    routes = []
    route_states = []
    trajectories = []
    objects = []
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    init_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose3d", 10)
    dummy_pub = node.create_publisher(
        DummyObject, "/simulation/dummy_perception_publisher/object_info", 10)
    node.create_subscription(Odometry, "/localization/kinematic_state",
                             odometry.append, 10)
    node.create_subscription(LaneletRoute, "/planning/mission_planning/route",
                             routes.append, durable)
    node.create_subscription(RouteState,
        "/planning/mission_planning/route_selector/main/state",
        lambda msg: route_states.append(msg.state), durable)
    node.create_subscription(Trajectory, "/planning/scenario_planning/trajectory",
                             trajectories.append, 10)
    node.create_subscription(PredictedObjects,
        "/perception/object_recognition/objects", objects.append, 10)
    clear_client = node.create_client(
        ClearRoute, "/planning/mission_planning/route_selector/main/clear_route")
    set_client = node.create_client(
        SetWaypointRoute,
        "/planning/mission_planning/route_selector/main/set_waypoint_route")

    try:
        if not spin_until(node, lambda: dummy_pub.get_subscription_count() > 0, 20):
            raise RuntimeError("RViz dummy perception publisher is unavailable")
        # Match RViz's Delete All Objects tool so repeat runs remain deterministic.
        reset = DummyObject()
        reset.header.frame_id = "map"
        reset.header.stamp = node.get_clock().now().to_msg()
        reset.action = DummyObject.DELETEALL
        dummy_pub.publish(reset)
        rclpy.spin_once(node, timeout_sec=0.2)

        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = "map"
        initial.pose.pose.position.x = args.start_x
        initial.pose.pose.position.y = args.start_y
        initial.pose.pose.position.z = 19.0
        initial.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
        initial.pose.pose.orientation.w = math.cos(args.yaw / 2.0)
        initialized = False
        for _ in range(30):
            initial.header.stamp = node.get_clock().now().to_msg()
            init_pub.publish(initial)
            rclpy.spin_once(node, timeout_sec=0.2)
            if odometry:
                position = odometry[-1].pose.pose.position
                initialized = math.hypot(position.x - args.start_x,
                                         position.y - args.start_y) < 1.0
                if initialized:
                    break
        if not initialized:
            raise RuntimeError("vehicle did not initialize at the requested position")

        if not spin_until(node, lambda: route_states and route_states[-1] in
                          (RouteState.UNSET, RouteState.SET, RouteState.ARRIVED,
                           RouteState.ABORTED, RouteState.INTERRUPTED), 30):
            raise RuntimeError("mission planner is still initializing")
        if route_states[-1] != RouteState.UNSET:
            call(node, clear_client, ClearRoute.Request(), "clear_route")
            if not spin_until(node, lambda: route_states and
                              route_states[-1] == RouteState.UNSET, 10):
                raise RuntimeError("route did not become UNSET after clear_route")

        request = SetWaypointRoute.Request()
        request.header.frame_id = "map"
        request.header.stamp = node.get_clock().now().to_msg()
        request.goal_pose.position.x = args.goal_x
        request.goal_pose.position.y = args.goal_y
        request.goal_pose.position.z = 19.0
        request.goal_pose.orientation.z = initial.pose.pose.orientation.z
        request.goal_pose.orientation.w = initial.pose.pose.orientation.w
        trajectories.clear()
        call(node, set_client, request, "set_waypoint_route")
        if not spin_until(node, lambda: len(trajectories) >= 2, 30):
            raise RuntimeError("route was set but no trajectory was received")

        # RViz 2D Dummy Pedestrian uses a stationary 0.6 m cylinder.
        pedestrian = DummyObject()
        pedestrian.header.frame_id = "map"
        pedestrian.header.stamp = node.get_clock().now().to_msg()
        pedestrian.id.uuid = [53] + [0] * 15
        pedestrian.action = DummyObject.ADD
        pedestrian.classification.label = ObjectClassification.PEDESTRIAN
        pedestrian.classification.probability = 1.0
        pedestrian.shape.type = Shape.CYLINDER
        pedestrian.shape.dimensions.x = 0.6
        pedestrian.shape.dimensions.y = 0.6
        pedestrian.shape.dimensions.z = 2.0
        pedestrian.initial_state.pose_covariance.pose.position.x = args.ped_x
        pedestrian.initial_state.pose_covariance.pose.position.y = args.ped_y
        pedestrian.initial_state.pose_covariance.pose.position.z = 1.0
        pedestrian.initial_state.pose_covariance.pose.orientation.w = 1.0
        pedestrian.initial_state.pose_covariance.covariance[0] = 0.03 ** 2
        pedestrian.initial_state.pose_covariance.covariance[7] = 0.03 ** 2
        pedestrian.initial_state.pose_covariance.covariance[14] = 0.03 ** 2
        pedestrian.initial_state.pose_covariance.covariance[35] = (5 * math.pi / 180) ** 2
        pedestrian.max_velocity = 33.3
        pedestrian.min_velocity = -33.3
        objects.clear()
        dummy_pub.publish(pedestrian)

        def observed_pedestrian():
            for message in objects:
                for obj in message.objects:
                    pose = obj.kinematics.initial_pose_with_covariance.pose.position
                    if (obj.classification and
                            obj.classification[0].label == ObjectClassification.PEDESTRIAN and
                            math.hypot(pose.x - args.ped_x,
                                       pose.y - args.ped_y) < 2.0):
                        return (pose.x, pose.y)
            return None

        if not spin_until(node, observed_pedestrian, 15):
            raise RuntimeError("dummy pedestrian was not received on PredictedObjects")
        spin_until(node, lambda: len(trajectories) >= 10, 5)
        print(json.dumps({
            "start": [args.start_x, args.start_y],
            "goal": [args.goal_x, args.goal_y],
            "pedestrian": [args.ped_x, args.ped_y],
            "observed_pedestrian": observed_pedestrian(),
            "route_segments": len(routes[-1].segments) if routes else 0,
            "received_trajectories": len(trajectories),
            "message": "Scenario set. Enable Autonomous in RViz to drive.",
        }, ensure_ascii=False, indent=2), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
