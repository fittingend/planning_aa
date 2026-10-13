"""정차 금지 구역의 정지 차량 유무에 따른 전체 경로를 비교합니다."""

import json
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
    vehicle = data.get("synthetic_car")
    payload.append(int(vehicle is not None))
    if vehicle:
        payload.extend((vehicle["x"], vehicle["y"],
                        vehicle["speed_mps"], vehicle["label"],
                        vehicle.get("yaw_rad", 0.0),
                        vehicle.get("length_m", 3.0),
                        vehicle.get("width_m", 3.0)))
    process = subprocess.run(
        [str(runner), str(map_file), "no_stopping_area"],
        input=" ".join(map(str, payload)), text=True,
        capture_output=True, check=True)
    lines = process.stdout.splitlines()
    selected = [list(map(int, line.split()[1:])) for line in lines[:3]]
    actual = [dict(zip(FIELDS, map(float, line.split())))
              for line in lines[4:]]
    expected = data["output"]
    tolerance = data.get("comparison_tolerance", 0.001)
    assert len(actual) == int(lines[3]) == len(expected)
    for index, (got, want) in enumerate(zip(actual, expected)):
        for field in FIELDS:
            assert abs(got[field] - want[field]) < tolerance, (
                fixture, index, field, got[field], want[field])
    first_zero = next(index for index, point in enumerate(expected)
                      if point["velocity"] == 0.0)
    stops = data.get("module_stopped", first_zero < len(expected) - 1)
    assert selected == [[], [], [900000228] if stops else []], selected
    print(f"{fixture.name}: 정차 금지 구역 경로 {len(actual)}점 일치")


def main():
    runner, map_file, *fixtures = map(Path, sys.argv[1:])
    for fixture in fixtures:
        run_case(runner, map_file, fixture)


if __name__ == "__main__":
    main()
