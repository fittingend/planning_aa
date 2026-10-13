"""지도 통합 충돌 판단의 선택 차로와 정지 위치를 원본에 비교합니다."""

import json
import math
import subprocess
import sys
from pathlib import Path


FIELDS = ("x", "y", "z", "qx", "qy", "qz", "qw", "velocity",
          "lateral_velocity", "heading_rate")


def run_case(runner, map_file, fixture):
    data = json.loads(fixture.read_text())
    ego = data["ego"]
    payload = [ego["x"], ego["y"], ego["velocity_mps"],
               ego["acceleration_mps2"], len(data["input"])]
    for point, ids in zip(data["input"], data["input_lane_ids"]):
        payload.extend(point[field] for field in FIELDS)
        payload.extend((len(ids), *ids))
    car = data.get("synthetic_car")
    payload.append(int(car is not None))
    if car:
        payload.extend((car["x"], car["y"], car["yaw_rad"],
                        car["speed_mps"], car["prediction_duration_s"],
                        car.get("label", 1), car.get("confidence", 1.0),
                        car.get("path_y_shift", 0.0),
                        car.get("second_path_confidence") or -1.0))
    signal = data.get("intersection_signal")
    payload.append(int(signal is not None))
    if signal:
        payload.extend((signal["regulatory_id"],
                        {"red": 1, "green": 2, "yellow": 3}[signal["color"]]))
    process = subprocess.run(
        [str(runner), str(map_file), "intersection_collision"],
        input=" ".join(map(str, payload)), text=True,
        capture_output=True, check=True)
    lines = process.stdout.splitlines()
    selected = [list(map(int, line.split()[1:])) for line in lines[:3]]
    actual = [dict(zip(FIELDS, map(float, line.split())))
              for line in lines[4:]]
    assert len(actual) == int(lines[3])
    expected_output = data["output"]
    assert len(actual) == len(expected_output)
    for actual_point, expected_point in zip(actual, expected_output):
        for field in FIELDS:
            assert abs(actual_point[field] - expected_point[field]) < 0.001
    decision = data["intersection_decisions"][-1].splitlines()[1]
    reference_stop_index = next(
        index for index, point in enumerate(expected_output)
        if point["velocity"] == 0)
    if reference_stop_index == len(expected_output) - 1:
        assert decision in ("Safe:", "FullyPrioritized")
        assert selected == [[], [], []], selected
        assert next(i for i, point in enumerate(actual)
                    if point["velocity"] == 0) == len(actual) - 1
    else:
        assert decision in ("NonOccludedCollisionStop:", "FullyPrioritized")
        assert selected == [[], [], [25001]], selected
        stop = next(point for point in actual if point["velocity"] == 0)
        expected = next(point for point in data["output"]
                        if point["velocity"] == 0)
        distance = math.hypot(stop["x"] - expected["x"],
                              stop["y"] - expected["y"])
        print(f"{fixture.name}: 교차로 충돌 정지점 차이 {distance:.3f}m")
        assert distance < 0.005


def main():
    runner, map_file, *fixtures = map(Path, sys.argv[1:])
    for fixture in fixtures:
        run_case(runner, map_file, fixture)


if __name__ == "__main__":
    main()
