"""감지 영역의 장애물 유무에 따른 전체 정지 경로를 원본과 비교합니다."""

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
    obstacle = data.get("synthetic_point")
    payload.append(int(obstacle is not None))
    if obstacle:
        payload.extend((obstacle["x"], obstacle["y"]))
    process = subprocess.run(
        [str(runner), str(map_file), "detection_area"],
        input=" ".join(map(str, payload)), text=True,
        capture_output=True, check=True)
    lines = process.stdout.splitlines()
    selected = [list(map(int, line.split()[1:])) for line in lines[:3]]
    actual = [dict(zip(FIELDS, map(float, line.split())))
              for line in lines[4:]]
    expected = data["output"]
    assert len(actual) == int(lines[3]) == len(expected)
    for index, (got, want) in enumerate(zip(actual, expected)):
        for field in FIELDS:
            assert abs(got[field] - want[field]) < 0.001, (
                fixture, index, field, got[field], want[field])
    assert selected == [[], [], [900000108] if obstacle else []], selected
    print(f"{fixture.name}: 감지 영역 경로 {len(actual)}점 일치")


def main():
    runner, map_file, *fixtures = map(Path, sys.argv[1:])
    for fixture in fixtures:
        run_case(runner, map_file, fixture)


if __name__ == "__main__":
    main()
