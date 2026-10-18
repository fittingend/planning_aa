#!/usr/bin/env python3
"""Publish Planning AA's full path as an RViz marker, separate from control."""

import math
import socket

import rclpy
from geometry_msgs.msg import Point
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def decode_packet(raw):
    text = raw.decode("ascii")
    if not text.startswith("PLAA_ROUTE1 "):
        raise ValueError("unknown route packet")
    values = text[len("PLAA_ROUTE1 "):].split(";")
    points = []
    for value in values:
        if not value:
            continue
        coordinates = tuple(float(item) for item in value.split(","))
        if len(coordinates) != 3 or not all(map(math.isfinite, coordinates)):
            raise ValueError("invalid route point")
        point = Point()
        point.x, point.y, point.z = coordinates
        points.append(point)
    if not 2 <= len(points) <= 500:
        raise ValueError("invalid route size")
    return points


def markers(node, points):
    line = Marker()
    line.header.frame_id = "map"
    line.header.stamp = node.get_clock().now().to_msg()
    line.ns = "route_lanelets"
    line.id = 0
    line.type = Marker.LINE_STRIP
    line.action = Marker.ADD
    line.scale.x = 0.35
    line.color.r = 0.1
    line.color.g = 0.65
    line.color.b = 1.0
    line.color.a = 0.9
    line.lifetime.sec = 2
    line.points = points

    goal = Marker()
    goal.header = line.header
    goal.ns = "goal_lanelets"
    goal.id = 1
    goal.type = Marker.SPHERE
    goal.action = Marker.ADD
    goal.pose.position = points[-1]
    goal.pose.orientation.w = 1.0
    goal.scale.x = goal.scale.y = goal.scale.z = 0.8
    goal.color.r = 0.1
    goal.color.g = 1.0
    goal.color.b = 0.3
    goal.color.a = 0.9
    goal.lifetime.sec = 2
    return MarkerArray(markers=[line, goal])


def main():
    rclpy.init()
    node = rclpy.create_node("planning_aa_route_visualization")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(
        MarkerArray, "/planning/mission_planning/route_marker", qos)
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("0.0.0.0", 12508))
    receiver.settimeout(1.0)
    print("PlanningAA route visualization listening on UDP 12508", flush=True)
    try:
        while rclpy.ok():
            try:
                raw, _ = receiver.recvfrom(65535)
            except socket.timeout:
                continue
            try:
                points = decode_packet(raw)
            except (UnicodeDecodeError, ValueError) as error:
                node.get_logger().warning(f"route packet dropped: {error}")
                continue
            publisher.publish(markers(node, points))
            print(f"route_marker points={len(points)} goal=({points[-1].x:.3f}, "
                  f"{points[-1].y:.3f})", flush=True)
    finally:
        receiver.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
