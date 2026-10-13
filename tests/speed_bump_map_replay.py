"""속도 방지턱 구간의 전체 경로를 원본 플래너와 비교합니다."""

import json
import subprocess
import sys
from pathlib import Path


FIELDS = ("x", "y", "z", "qx", "qy", "qz", "qw", "velocity",
          "lateral_velocity", "heading_rate")


def main():
    runner, map_file, fixture = map(Path, sys.argv[1:])
    data = json.loads(fixture.read_text())
    ego = data["ego"]
    payload = [ego["x"], ego["y"], ego["velocity_mps"],
               ego["acceleration_mps2"], len(data["input"])]
    for point, ids in zip(data["input"], data["input_lane_ids"]):
        payload.extend(point[field] for field in FIELDS)
        payload.extend((len(ids), *ids))
    process = subprocess.run(
        [str(runner), str(map_file), "speed_bump"],
        input=" ".join(map(str, payload)), text=True,
        capture_output=True, check=True)
    lines = process.stdout.splitlines()
    assert [list(map(int, line.split()[1:])) for line in lines[:3]] == [
        [], [], [900000125]]
    actual = [dict(zip(FIELDS, map(float, line.split())))
              for line in lines[4:]]
    expected = data["output"]
    assert len(actual) == int(lines[3]) == len(expected)
    for index, (got, want) in enumerate(zip(actual, expected)):
        for field in FIELDS:
            assert abs(got[field] - want[field]) < 0.001, (
                index, field, got[field], want[field])
    print(f"속도 방지턱 경로 {len(actual)}점 일치")


if __name__ == "__main__":
    main()
