"""주행 금지 차로 모듈의 8프레임 전체 경로를 원본과 비교합니다."""

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
        [str(runner), str(map_file), "no_drivable_sequence"],
        input=" ".join(map(str, payload)), text=True,
        capture_output=True, check=True)
    lines = process.stdout.splitlines()
    cursor = 0
    frames = data["no_drivable_frames"][:8]
    assert len(frames) == 8
    for frame_index, frame in enumerate(frames):
        selected, count = map(int, lines[cursor].split())
        cursor += 1
        assert selected == (0 if frame_index == 0 else 1)
        expected = frame["output"]
        assert count == len(expected)
        for point_index, want in enumerate(expected):
            got = dict(zip(FIELDS, map(float, lines[cursor].split())))
            cursor += 1
            for field in FIELDS:
                assert abs(got[field] - want[field]) < 0.001, (
                    frame_index, point_index, field, got[field], want[field])
    assert cursor == len(lines)
    print("주행 금지 차로 8프레임 전체 경로 일치")


if __name__ == "__main__":
    main()
