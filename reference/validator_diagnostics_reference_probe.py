"""원본 validator의 진단 출력에서 정상·무효·회복 상태를 수집합니다."""

import argparse
import json
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from autoware_planning_validator.msg import PlanningValidatorStatus
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry


def make_trajectory(size):
    message = Trajectory()
    message.header.frame_id = "map"
    for index in range(size):
        point = TrajectoryPoint()
        point.pose.position.x = index * 10.0
        point.pose.orientation.w = 1.0
        point.longitudinal_velocity_mps = 10.0
        message.points.append(point)
    return message


def spin_until(node, predicate, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def collect(diagnostics, expected_level):
    def level(item):
        return item.level[0] if isinstance(item.level, bytes) else item.level

    for message in reversed(diagnostics):
        items = [item for item in message.status
                 if "trajectory_validation_" in item.name]
        if len(items) == 14 and any(
                item.name.endswith("trajectory_validation_size") and
                level(item) == expected_level for item in items):
            return [{"name": item.name, "level": level(item),
                     "message": item.message, "hardware_id": item.hardware_id}
                    for item in items]
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    process = subprocess.Popen(
        ["ros2", "launch", "autoware_planning_validator",
         "planning_validator.launch.xml"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    node = Node("validator_diagnostics_probe")
    statuses = []
    diagnostics = []
    node.create_subscription(
        PlanningValidatorStatus, "/planning_validator/validation_status",
        lambda message: statuses.append(message), 10)
    node.create_subscription(
        DiagnosticArray, "/diagnostics",
        lambda message: diagnostics.append(message), 10)
    pub_odom = node.create_publisher(
        Odometry, "/localization/kinematic_state", 10)
    pub_traj = node.create_publisher(
        Trajectory,
        "/planning/scenario_planning/velocity_smoother/trajectory", 10)
    odom = Odometry()
    odom.header.frame_id = "map"
    odom.pose.pose.orientation.w = 1.0
    odom.twist.twist.linear.x = 10.0
    records = []
    try:
        spin_until(node, lambda: False, 2.0)
        for name, size, count, level in (
                ("valid_before", 3, 0, 0),
                ("invalid_size", 1, 1, 2),
                ("valid_after", 3, 0, 0)):
            previous = len(statuses)
            trajectory = make_trajectory(size)
            for _ in range(20):
                pub_odom.publish(odom)
                pub_traj.publish(trajectory)
                spin_until(node, lambda: False, 0.1)
                if len(statuses) > previous and statuses[-1].invalid_count == count:
                    break
            if len(statuses) == previous:
                raise RuntimeError("validator 상태를 받지 못했습니다")
            diagnostics.clear()
            if not spin_until(
                    node, lambda: collect(diagnostics, level) is not None, 3.0):
                raise RuntimeError(
                    f"validator 진단을 받지 못했습니다: {len(diagnostics)}회, "
                    f"{[(item.name, item.level) for message in diagnostics[-1:] for item in message.status]}")
            items = collect(diagnostics, level)
            records.append({"case": name, "invalid_count": count,
                            "diagnostics": items})
            print(json.dumps({"case": name, "items": len(items)}), flush=True)
        args.output.write_text(
            json.dumps(records, ensure_ascii=False, indent=2) + "\n")
    finally:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
