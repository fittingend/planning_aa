#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Publish the mission route goal with the source node's topic and QoS contract."""

import argparse

import rclpy
from autoware_planning_msgs.msg import LaneletRoute
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


DEFAULT_ROUTE = "/planning/mission_planning/route"
DEFAULT_GOAL = "/planning/mission_planning/echo_back_goal_pose"


class GoalPoseVisualizer(Node):
    def __init__(self, route_topic: str, goal_topic: str) -> None:
        super().__init__("goal_pose_visualizer")
        durable = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.create_publisher(PoseStamped, goal_topic, durable)
        self._subscription = self.create_subscription(
            LaneletRoute, route_topic, self._on_route, durable
        )

    def _on_route(self, route: LaneletRoute) -> None:
        goal = PoseStamped()
        goal.header = route.header
        goal.pose = route.goal_pose
        self._publisher.publish(goal)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route-topic", default=DEFAULT_ROUTE)
    parser.add_argument("--goal-topic", default=DEFAULT_GOAL)
    arguments, ros_arguments = parser.parse_known_args()
    rclpy.init(args=ros_arguments)
    node = GoalPoseVisualizer(arguments.route_topic, arguments.goal_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
