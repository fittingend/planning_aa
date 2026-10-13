"""기록된 행동 속도 계획 경로에서 정지점이 나타나는 시점을 찾습니다."""

import argparse

from rclpy.serialization import deserialize_message
from autoware_planning_msgs.msg import Path
from tier4_planning_msgs.msg import PathWithLaneId

from scenario_selector_reference_probe import load


def first_zero(points):
    return next((index for index, point in enumerate(points)
                 if abs(point.longitudinal_velocity_mps) < 1e-4), None)


parser = argparse.ArgumentParser()
parser.add_argument("bag")
args = parser.parse_args()
for step in range(1, 20):
    fraction = step / 20
    stamp, data = load(
        args.bag,
        "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id",
        fraction,
    )
    _, output_data = load(
        args.bag, "/planning/scenario_planning/lane_driving/behavior_planning/path",
        near=stamp,
    )
    path = deserialize_message(data, PathWithLaneId)
    output = deserialize_message(output_data, Path)
    source = [point.point for point in path.points]
    print(fraction, len(source), first_zero(source),
          len(output.points), first_zero(output.points), flush=True)
