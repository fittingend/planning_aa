"""원본 교차로의 안전 통과와 정체 정지 결정을 비교합니다."""

import json
import math
import subprocess
import sys
from pathlib import Path


FIELDS = ("x", "y", "z", "qx", "qy", "qz", "qw", "velocity",
          "lateral_velocity", "heading_rate")


def run_case(runner, map_file, fixture):
    data = json.loads(fixture.read_text())
    path = data["input"]
    payload = [data["ego"]["x"], data["ego"]["y"],
               data["ego"]["velocity_mps"],
               data["ego"]["acceleration_mps2"], len(path)]
    for point in path:
        payload.extend(point[field] for field in FIELDS)
    car = data.get("synthetic_car")
    payload.append(int(car is not None))
    if car:
        payload.extend((car["x"], car["y"], car["speed_mps"],
                        car.get("label", 1)))
    result = subprocess.run(
        [str(runner), str(map_file)], input=" ".join(map(str, payload)),
        text=True, capture_output=True, check=True)
    lines = result.stdout.splitlines()
    detected, conflict_arc, stop_arc, size = map(float, lines[0].split())
    output = [dict(zip(FIELDS, map(float, line.split())))
              for line in lines[1:]]
    assert len(output) == int(size)
    expected_output = data["output"]
    assert len(output) == len(expected_output)
    for actual_point, expected_point in zip(output, expected_output):
        for field in FIELDS:
            assert abs(actual_point[field] - expected_point[field]) < 0.001
    original_state = data["intersection_decisions"][-1]
    if original_state.splitlines()[1] == "Safe:":
        assert detected == 0 and stop_arc == -1
        assert next(i for i, point in enumerate(output)
                    if point["velocity"] == 0) == len(output) - 1
    else:
        assert original_state.splitlines()[1] == "StuckStop"
        assert detected == 1 and 0 < stop_arc < conflict_arc
        actual_stop = next(point for point in output if point["velocity"] == 0)
        reference_stop = next(point for point in data["output"]
                              if point["velocity"] == 0)
        distance = math.hypot(actual_stop["x"] - reference_stop["x"],
                              actual_stop["y"] - reference_stop["y"])
        print(f"교차로 정체 정지 위치 차이: {distance:.3f}m")
        assert distance < 0.005


def main():
    runner, map_file, *fixtures = map(Path, sys.argv[1:])
    for fixture in fixtures:
        run_case(runner, map_file, fixture)


if __name__ == "__main__":
    main()
