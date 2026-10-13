"""사유 도로 합류의 정지 경로를 원본 전체 출력과 비교합니다."""

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
    payload.extend((0, 0))
    process = subprocess.run(
        [str(runner), str(map_file), "merge_from_private"],
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
    stopping = any(point["velocity"] == 0 for point in expected[:-1])
    assert selected == [[], [], [25001] if stopping else []], selected
    print(f"{fixture.name}: 사유 도로 합류 경로 {len(actual)}점 일치")


def main():
    runner, map_file, fixture = map(Path, sys.argv[1:])
    run_case(runner, map_file, fixture)


if __name__ == "__main__":
    main()
