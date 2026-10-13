"""시나리오 선택 비교 시점의 경로 차로와 목적지 좌표를 rosbag에서 추출합니다."""

import argparse
import json
import math
from pathlib import Path

from autoware_planning_msgs.msg import LaneletRoute
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

from scenario_selector_reference_probe import load


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    reference = json.loads(args.reference.read_text())
    timestamp = reference["bag_timestamp"]
    route = deserialize_message(
        load(args.bag, "/planning/mission_planning/route", near=timestamp)[1],
        LaneletRoute)
    odometry = deserialize_message(
        load(args.bag, "/localization/kinematic_state", near=timestamp)[1],
        Odometry)
    orientation = odometry.pose.pose.orientation
    yaw = math.atan2(
        2 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1 - 2 * (orientation.y**2 + orientation.z**2))
    lane_ids = sorted({primitive.id for segment in route.segments
                       for primitive in segment.primitives})
    result = {
        "bag_timestamp": timestamp,
        "scenario": reference["scenario"],
        "ego": {"x": odometry.pose.pose.position.x,
                "y": odometry.pose.pose.position.y,
                "yaw_rad": yaw},
        "goal": {"x": route.goal_pose.position.x,
                 "y": route.goal_pose.position.y},
        "route_lanelet_ids": lane_ids,
    }
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(f"경로 차로 {len(lane_ids)}개, 원본 시나리오 {result['scenario']}")


if __name__ == "__main__":
    main()
