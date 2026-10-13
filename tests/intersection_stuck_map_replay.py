"""교차로 정체 차량의 지도 선택과 통합 정지점을 원본에 비교합니다."""

import json
import math
import subprocess
import sys
from pathlib import Path


FIELDS = ("x", "y", "z", "qx", "qy", "qz", "qw", "velocity",
          "lateral_velocity", "heading_rate")


def run_case(runner, map_file, fixture, expected_lane_id=25001,
             tolerance=0.001):
    data = json.loads(fixture.read_text())
    ego = data["ego"]
    payload = [ego["x"], ego["y"], ego["velocity_mps"],
               ego["acceleration_mps2"], len(data["input"])]
    for point, ids in zip(data["input"], data["input_lane_ids"]):
        payload.extend(point[field] for field in FIELDS)
        payload.extend([len(ids), *ids])
    car = data.get("synthetic_car")
    payload.append(int(car is not None))
    if car:
        payload.extend((car["x"], car["y"], car["speed_mps"],
                        car.get("label", 1)))
    process = subprocess.run(
        [str(runner), str(map_file), "intersection_stuck"],
        input=" ".join(map(str, payload)), text=True,
        capture_output=True, check=True)
    lines = process.stdout.splitlines()
    selected = [list(map(int, line.split()[1:])) for line in lines[:3]]
    actual = [dict(zip(FIELDS, map(float, line.split())))
              for line in lines[4:]]
    assert len(actual) == int(lines[3])
    expected_output = data["output"]
    assert len(actual) == len(expected_output), (
        fixture, selected, len(actual), len(expected_output))
    for index, (actual_point, expected_point) in enumerate(
            zip(actual, expected_output)):
        for field in FIELDS:
            assert abs(actual_point[field] - expected_point[field]) < tolerance, (
                fixture, index, field, actual_point[field], expected_point[field])
    if data["intersection_decisions"][-1].splitlines()[1] == "Safe:":
        assert selected == [[], [], []], selected
        assert data["intersection_decisions"][-1].splitlines()[1] == "Safe:"
        assert next(i for i, point in enumerate(actual)
                    if point["velocity"] == 0) == len(actual) - 1
    else:
        assert selected == [[], [], [expected_lane_id]], selected
        assert data["intersection_decisions"][-1].splitlines()[1] == "StuckStop"
        stop = next(point for point in actual if point["velocity"] == 0)
        reference_stop = next(point for point in data["output"]
                              if point["velocity"] == 0)
        distance = math.hypot(stop["x"] - reference_stop["x"],
                              stop["y"] - reference_stop["y"])
        print(f"교차로 통합 정지점 차이: {distance:.3f}m")
        assert distance < 0.005


def main():
    runner, map_file, *fixtures = map(Path, sys.argv[1:])
    for fixture in fixtures:
        run_case(runner, map_file, fixture)


if __name__ == "__main__":
    main()
