"""체크포인트 간 차로 경로의 구간과 이웃 차로를 원본과 비교합니다."""

import json
import math
import subprocess
import sys
from pathlib import Path


def main():
    runner, map_file, fixture = map(Path, sys.argv[1:])
    data = json.loads(fixture.read_text())
    start, goal = data["start"], data["goal"]
    payload = [start["x"], start["y"],
               2 * math.atan2(start["yaw_qz"], start["yaw_qw"]),
               goal["x"], goal["y"],
               2 * math.atan2(goal["yaw_qz"], goal["yaw_qw"])]
    waypoints = data.get("waypoints", [])
    payload.append(len(waypoints))
    for waypoint in waypoints:
        payload.extend((waypoint["x"], waypoint["y"],
                        2 * math.atan2(waypoint["yaw_qz"],
                                       waypoint["yaw_qw"])))
    payload.append(int(data.get("consider_no_drivable_lanes", False)))
    payload.append(goal.get("z", 0.0))
    process = subprocess.run(
        [str(runner), str(map_file)], input=" ".join(map(str, payload)),
        text=True, capture_output=True, check=True)
    lines = process.stdout.splitlines()
    assert int(lines[0]) == len(data["segments"])
    for line, expected in zip(lines[1:], data["segments"]):
        preferred, count, *primitives = map(int, line.split())
        assert preferred == expected["preferred"]
        assert count == len(primitives)
        assert primitives == expected["primitives"]
    if data.get("goal_height_m") is not None:
        actual_height = float(lines[len(data["segments"]) + 1])
        assert abs(actual_height - data["goal_height_m"]) < 0.001, (
            actual_height, data["goal_height_m"])
    print(f"목적지 경로 {len(data['segments'])}개 구간 일치")


if __name__ == "__main__":
    main()
