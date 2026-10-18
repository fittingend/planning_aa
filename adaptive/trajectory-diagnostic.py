#!/usr/bin/env python3
"""Read-only trajectory and publisher probe run inside the control container."""

import json
import math
import time

import rclpy
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_planning_msgs.msg import LaneletRoute, Trajectory
from diagnostic_msgs.msg import DiagnosticArray
from tier4_system_msgs.msg import (
    DiagGraphStatus, DiagGraphStruct, OperationModeAvailability,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


TOPIC = "/planning/scenario_planning/trajectory"
FIELDS = (
    "pose.position.x", "pose.position.y", "pose.position.z",
    "pose.orientation.x", "pose.orientation.y", "pose.orientation.z",
    "pose.orientation.w", "longitudinal_velocity_mps",
    "lateral_velocity_mps", "acceleration_mps2", "heading_rate_rps",
    "front_wheel_angle_rad", "rear_wheel_angle_rad",
)


def field_value(point, path):
    value = point
    for part in path.split("."):
        value = getattr(value, part)
    return value


def diagnostic_level(status):
    return status.level[0] if isinstance(status.level, bytes) else int(status.level)


class Probe(Node):
    def __init__(self):
        super().__init__("planning_aa_trajectory_diagnostic")
        self.received = 0
        self.last_sample_log = 0.0
        self.last_endpoints = None
        self.last_invalid_fields = []
        self.last_mode = None
        self.last_availability = None
        self.last_diagnostics = {}
        self.last_route = None
        self.graph_structure = None
        self.last_graph_levels = {}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Trajectory, TOPIC, self.on_sample, qos)
        self.create_subscription(LaneletRoute, "/planning/mission_planning/route",
                                 self.on_route, qos)
        self.create_subscription(OperationModeState, "/api/operation_mode/state",
                                 self.on_mode, qos)
        self.create_subscription(OperationModeAvailability,
                                 "/system/operation_mode/availability",
                                 self.on_availability, qos)
        self.create_subscription(DiagnosticArray, "/diagnostics",
                                 self.on_diagnostics, qos)
        structure_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(DiagGraphStruct, "/diagnostics_graph/struct",
                                 self.on_graph_structure, structure_qos)
        self.create_subscription(DiagGraphStatus, "/diagnostics_graph/status",
                                 self.on_graph_status, qos)
        self.create_timer(2.0, self.on_timer)
        self.on_timer()

    def emit(self, kind, **values):
        print(json.dumps({"time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                          "kind": kind, **values}, ensure_ascii=False), flush=True)

    def on_timer(self):
        try:
            publishers = sorted(
                (f"{info.node_namespace.rstrip('/')}/{info.node_name}",
                 str(info.qos_profile.reliability),
                 str(info.qos_profile.durability))
                for info in self.get_publishers_info_by_topic(TOPIC)
            )
            if publishers != self.last_endpoints:
                self.emit("publishers", topic=TOPIC, publishers=publishers)
                self.last_endpoints = publishers
            self.emit("status", received=self.received)
        except Exception as error:
            self.emit("publisher_query_error", error=str(error))

    def on_sample(self, msg):
        self.received += 1
        bad = []
        max_gap = 0.0
        for index, point in enumerate(msg.points):
            for field in FIELDS:
                if not math.isfinite(field_value(point, field)):
                    bad.append(f"points[{index}].{field}")
                    if len(bad) == 12:
                        break
            if len(bad) == 12:
                break
            if index:
                prev = msg.points[index - 1].pose.position
                curr = point.pose.position
                gap = math.hypot(curr.x - prev.x, curr.y - prev.y)
                if math.isfinite(gap):
                    max_gap = max(max_gap, gap)
                    if gap >= 5.0:
                        bad.append(f"points[{index}].position_gap_m={gap:.2f}")
                else:
                    bad.append(f"points[{index}].position_gap_m=nonfinite")
                if len(bad) == 12:
                    break
        if not msg.points:
            bad.append("points.empty")
        now = time.monotonic()
        if (self.received == 1 or bad != self.last_invalid_fields or
                now - self.last_sample_log >= 5.0):
            first = msg.points[0] if msg.points else None
            last = msg.points[-1] if msg.points else None
            self.emit(
                "sample", topic=TOPIC, count=self.received,
                stamp=[msg.header.stamp.sec, msg.header.stamp.nanosec],
                frame_id=msg.header.frame_id, points=len(msg.points),
                first=([first.pose.position.x, first.pose.position.y,
                        first.pose.position.z, first.longitudinal_velocity_mps]
                       if first else None),
                last=([last.pose.position.x, last.pose.position.y,
                       last.pose.position.z, last.longitudinal_velocity_mps]
                      if last else None),
                invalid_fields=bad,
                max_gap_m=round(max_gap, 3),
            )
            self.last_sample_log = now
            self.last_invalid_fields = bad

    def on_mode(self, msg):
        state = (msg.mode, msg.is_autonomous_mode_available,
                 msg.is_autoware_control_enabled, msg.is_in_transition)
        if state != self.last_mode:
            self.emit("operation_mode", mode=msg.mode,
                      autonomous_available=msg.is_autonomous_mode_available,
                      control_enabled=msg.is_autoware_control_enabled,
                      in_transition=msg.is_in_transition)
            self.last_mode = state

    def on_route(self, msg):
        route = (tuple(msg.uuid.uuid), tuple(segment.preferred_primitive.id
                                              for segment in msg.segments))
        if route != self.last_route:
            self.emit("route", start=[msg.start_pose.position.x,
                                       msg.start_pose.position.y],
                      goal=[msg.goal_pose.position.x,
                            msg.goal_pose.position.y],
                      lane_ids=list(route[1]))
            self.last_route = route

    def on_availability(self, msg):
        state = (msg.stop, msg.autonomous, msg.local, msg.remote)
        if state != self.last_availability:
            self.emit("system_availability", stop=msg.stop,
                      autonomous=msg.autonomous, local=msg.local,
                      remote=msg.remote)
            self.last_availability = state

    def on_diagnostics(self, msg):
        # Humble exposes DiagnosticStatus.level as a one-byte bytes value.
        # /diagnostics can contain one status per message, so track each name.
        for status in msg.status:
            level = diagnostic_level(status)
            if self.last_diagnostics.get(status.name) != level:
                self.emit("diagnostic_change", name=status.name,
                          level=level, message=status.message)
                self.last_diagnostics[status.name] = level

    def on_graph_structure(self, msg):
        self.graph_structure = msg
        self.last_graph_levels.clear()
        self.emit("graph_structure", id=msg.id, nodes=len(msg.nodes),
                  leaves=len(msg.diags))

    def on_graph_status(self, msg):
        structure = self.graph_structure
        if structure is None or structure.id != msg.id:
            return
        for item, status in zip(structure.nodes, msg.nodes):
            path = item.path
            if path not in ("/autoware/modes/autonomous", "/autoware/planning",
                            "/autoware/control", "/autoware/map",
                            "/autoware/localization", "/autoware/perception",
                            "/autoware/vehicle", "/autoware/system",
                            "/autoware/planning/trajectory_validation"):
                continue
            level = diagnostic_level(status)
            key = "node:" + path
            if self.last_graph_levels.get(key) != level:
                self.emit("graph_change", path=path, level=level)
                self.last_graph_levels[key] = level
        for item, status in zip(structure.diags, msg.diags):
            if not item.path.startswith("/autoware/planning/"):
                continue
            level = diagnostic_level(status)
            key = "leaf:" + item.path
            if self.last_graph_levels.get(key) != level:
                self.emit("graph_leaf_change", path=item.path, level=level,
                          message=status.message)
                self.last_graph_levels[key] = level


def main():
    rclpy.init()
    node = Probe()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
