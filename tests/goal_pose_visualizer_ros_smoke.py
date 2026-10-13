#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Check the goal echo's payload and transient-local delivery over ROS DDS."""

import sys
import time
from pathlib import Path

import rclpy
from autoware_planning_msgs.msg import LaneletRoute
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros"))
sys.dont_write_bytecode = True
from goal_pose_visualizer import GoalPoseVisualizer  # noqa: E402


def main() -> None:
    route_topic = "/planning_aa_smoke/route"
    goal_topic = "/planning_aa_smoke/goal"
    durable = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    rclpy.init()
    visualizer = GoalPoseVisualizer(route_topic, goal_topic)
    source = Node("planning_aa_goal_echo_source")
    late = None
    restarted_late = None
    try:
        route_publisher = source.create_publisher(LaneletRoute, route_topic, durable)
        route = LaneletRoute()
        route.header.frame_id = "map"
        route.header.stamp.sec = 123
        route.goal_pose.position.x = 4.25
        route.goal_pose.position.y = -2.0
        route.goal_pose.orientation.w = 1.0
        deadline = time.monotonic() + 5.0
        while route_publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(source, timeout_sec=0.05)
        assert route_publisher.get_subscription_count() > 0
        route_publisher.publish(route)
        rclpy.spin_once(visualizer, timeout_sec=0.5)

        late = Node("planning_aa_goal_echo_late")
        received = []
        subscription = late.create_subscription(
            PoseStamped, goal_topic, received.append, durable
        )
        deadline = time.monotonic() + 5.0
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(late, timeout_sec=0.05)
        assert subscription is not None
        assert len(received) == 1, f"received {len(received)} goal poses"
        goal = received[0]
        assert goal.header.frame_id == route.header.frame_id
        assert goal.header.stamp == route.header.stamp
        assert goal.pose == route.goal_pose
        visualizer.destroy_node()
        visualizer = GoalPoseVisualizer(route_topic, goal_topic)
        deadline = time.monotonic() + 5.0
        while visualizer._publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(visualizer, timeout_sec=0.05)
        rclpy.spin_once(visualizer, timeout_sec=0.5)
        restarted_late = Node("planning_aa_goal_echo_restarted_late")
        restarted_messages = []
        restarted_late.create_subscription(
            PoseStamped, goal_topic, restarted_messages.append, durable
        )
        deadline = time.monotonic() + 5.0
        while not restarted_messages and time.monotonic() < deadline:
            rclpy.spin_once(restarted_late, timeout_sec=0.05)
        assert len(restarted_messages) == 1
        assert restarted_messages[0].header == route.header
        assert restarted_messages[0].pose == route.goal_pose
        print("goal echo, late subscriber, and restarted relay: passed")
    finally:
        if restarted_late is not None:
            restarted_late.destroy_node()
        if late is not None:
            late.destroy_node()
        source.destroy_node()
        visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
