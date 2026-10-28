#!/usr/bin/env python3
"""Publish PlanningAA's measured validator results on Autoware diagnostics."""

import os
import socket

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node


CHECKS = (
    "size", "finite", "interval", "relative_angle", "curvature",
    "lateral_acceleration", "acceleration", "deceleration", "steering",
    "steering_rate", "velocity_deviation", "distance_deviation",
    "longitudinal_distance_deviation", "forward_trajectory_length",
)
FAILURES = (
    "invalid trajectory size is found", "infinite value is found",
    "points interval is too long", "relative angle is too large",
    "curvature is too large", "lateral acceleration is too large",
    "acceleration is too large", "deceleration is too large",
    "expected steering is too large", "expected steering rate is too large",
    "velocity deviation is too large", "distance deviation is too large",
    "longitudinal distance deviation is too large",
    "trajectory length is too short",
)


def decode_packet(packet):
    try:
        magic, stamp, levels = packet.decode("ascii").split(" ")
        if magic != "PLAA1" or int(stamp) < 0 or len(levels) != len(CHECKS):
            return None
        if any(level not in "012" for level in levels):
            return None
        return int(stamp), levels
    except (UnicodeDecodeError, ValueError):
        return None


def make_diagnostic_array(levels, stamp):
    message = DiagnosticArray()
    message.header.stamp = stamp
    for name, failure, level in zip(CHECKS, FAILURES, levels):
        status = DiagnosticStatus()
        status.name = "planning_validator: trajectory_validation_" + name
        status.hardware_id = "planning_validator"
        # Humble's generated setter requires one byte for this uint8 field.
        status.level = bytes((int(level),))
        status.message = "validated." if level == "0" else failure
        message.status.append(status)
    return message


def main():
    rclpy.init()
    node = Node("planning_aa_validator_diagnostic_bridge")
    publisher = node.create_publisher(DiagnosticArray, "/diagnostics", 10)
    # Exercise the ROS message setters before announcing readiness. A type
    # mismatch must fail startup rather than the first live trajectory frame.
    make_diagnostic_array("0" * len(CHECKS), node.get_clock().now().to_msg())
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("0.0.0.0", 12507))
    receiver.settimeout(0.5)
    expected_source = os.environ.get("PLANNING_AA_SOURCE_IP", "10.88.0.93")
    print("PlanningAA validator diagnostics listening on UDP 12507", flush=True)
    last_levels = None
    received = 0
    try:
        while rclpy.ok():
            try:
                packet, address = receiver.recvfrom(256)
            except socket.timeout:
                continue
            if address[0] != expected_source:
                continue
            decoded = decode_packet(packet)
            if decoded is None:
                node.get_logger().warning("Invalid PlanningAA diagnostic packet")
                continue
            _, levels = decoded
            publisher.publish(make_diagnostic_array(
                levels, node.get_clock().now().to_msg()))
            received += 1
            if levels != last_levels or received % 100 == 0:
                print(f"published={received} validator_levels={levels}", flush=True)
                last_levels = levels
    finally:
        receiver.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
