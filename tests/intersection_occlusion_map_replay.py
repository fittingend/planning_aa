"""교차로 가림 격자의 정지 판단과 전체 경로를 원본 출력에 대조합니다."""

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
        second_confidence = car.get("second_path_confidence")
        payload.extend((
            car["x"], car["y"], car["yaw_rad"], car["speed_mps"],
            (-1.0 if car.get("prediction_duration_s") is None
             else car["prediction_duration_s"]),
            car.get("label", 1), car.get("confidence", 1.0),
            car.get("path_y_shift", 0.0),
            -1.0 if second_confidence is None else second_confidence,
        ))
    signal = data.get("intersection_signal")
    payload.append(int(signal is not None))
    if signal:
        payload.extend((signal["regulatory_id"],
                        {"red": 1, "green": 2, "yellow": 3}[signal["color"]]))
    grid = data["intersection_occupancy"]
    patch = grid["unknown_patch"] or (0.0, 0.0, 0.0)
    yaw = 2.0 * math.atan2(grid["origin_qz"], grid["origin_qw"])
    payload.extend((grid["width"], grid["height"], grid["resolution"],
                    grid["origin_x"], grid["origin_y"], yaw,
                    1, int(grid["fill_unknown"]), *patch))
    process = subprocess.run(
        [str(runner), str(map_file), "intersection_occlusion"],
        input=" ".join(map(str, payload)), text=True,
        capture_output=True, check=True)
    lines = process.stdout.splitlines()
    selected = [list(map(int, line.split()[1:])) for line in lines[:3]]
    actual = [dict(zip(FIELDS, map(float, line.split())))
              for line in lines[4:]]
    expected = data["output"]
    assert len(actual) == int(lines[3]) == len(expected), fixture
    for actual_point, expected_point in zip(actual, expected):
        for field in FIELDS:
            assert abs(actual_point[field] - expected_point[field]) < 0.001, (
                fixture, field, actual_point[field], expected_point[field])
    decision = data["intersection_decisions"][-1].splitlines()[1]
    assert decision in ("Safe:", "FullyPrioritized", "FirstWaitBeforeOcclusion:")
    expected_ids = [25001] if decision == "FirstWaitBeforeOcclusion:" else []
    assert selected == [[], [], expected_ids], (fixture, selected, decision)
    print(f"{fixture.name}: {decision} 전체 경로 일치")


def main():
    runner, map_file, *fixtures = map(Path, sys.argv[1:])
    for fixture in fixtures:
        run_case(runner, map_file, fixture)


if __name__ == "__main__":
    main()
