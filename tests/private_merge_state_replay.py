"""사유 도로 합류의 정차 후 출발 상태를 원본 연속 출력과 비교합니다."""

import json
import subprocess
import sys
from pathlib import Path


FIELDS = ("x", "y", "z", "qx", "qy", "qz", "qw", "velocity",
          "lateral_velocity", "heading_rate")


def main():
    runner, map_file, fixture = map(Path, sys.argv[1:])
    data = json.loads(fixture.read_text())
    frames = data["frames"]
    payload = [len(frames), len(data["input"])]
    for point, ids in zip(data["input"], data["input_lane_ids"]):
        payload.extend(point[field] for field in FIELDS)
        payload.extend((len(ids), *ids))
    for frame in frames:
        ego = frame["ego"]
        payload.extend((frame["replay_timestamp_ns"] / 1e9,
                        ego["x"], ego["y"], ego["velocity_mps"],
                        ego["acceleration_mps2"]))
    process = subprocess.run(
        [str(runner), str(map_file), "merge_from_private_sequence"],
        input=" ".join(map(str, payload)), text=True,
        capture_output=True, check=True)
    lines = iter(process.stdout.splitlines())
    for index, frame in enumerate(frames):
        stopped, count = map(int, next(lines).split())
        actual = [dict(zip(FIELDS, map(float, next(lines).split())))
                  for _ in range(count)]
        expected = frame["output"]
        assert count == len(expected), (index, count, len(expected))
        expected_stopped = any(point["velocity"] == 0 for point in expected[:-1])
        assert stopped == int(expected_stopped), (index, stopped)
        for point_index, (got, want) in enumerate(zip(actual, expected)):
            for field in FIELDS:
                assert abs(got[field] - want[field]) < 0.001, (
                    index, point_index, field, got[field], want[field])
    print(f"사유 도로 합류 연속 {len(frames)}프레임 일치")


if __name__ == "__main__":
    main()
