"""횡단보도 통합 실행기의 연속 객체 상태와 전체 경로를 원본과 비교합니다."""

import json
import math
import subprocess
import sys
from pathlib import Path


FIELDS = ("x", "y", "z", "qx", "qy", "qz", "qw", "velocity",
          "lateral_velocity", "heading_rate")


def serialize(frames):
    first = frames[0]["replay_timestamp_ns"]
    payload = [len(frames)]
    for frame in frames:
        ego = frame["ego"]
        path = frame["input"]
        payload.extend(((frame["replay_timestamp_ns"] - first) / 1e9,
                        ego["x"], ego["y"], ego["velocity_mps"],
                        ego["acceleration_mps2"], len(path)))
        for point in path:
            payload.extend(point[field] for field in FIELDS)
        payload.append(len(frame["objects"]))
        for obj in frame["objects"]:
            paths = obj.get("paths", [])
            payload.extend((obj["id"], obj["x"], obj["y"], obj["label"],
                            obj["length"], obj["width"], obj["velocity_x"],
                            obj["velocity_y"], len(paths)))
            for predicted in paths:
                poses = predicted["poses"]
                payload.append(len(poses))
                for pose in poses:
                    payload.extend((pose["x"], pose["y"],
                                    2 * math.atan2(pose["qz"], pose["qw"])))
    return " ".join(map(str, payload))


def expected_state(frame):
    matching = [entry for entry in frame["collision_info"]
                if entry.startswith("24783,a9b8,")]
    if not matching:
        return None
    assert len(matching) == 1
    return int(matching[0].split(",")[4])


def main():
    runner, map_file, reference_file = map(Path, sys.argv[1:4])
    frames = json.loads(reference_file.read_text())
    run = subprocess.run(
        [str(runner), str(map_file), "crosswalk_sequence"],
        input=serialize(frames), text=True, capture_output=True, check=True)
    rows = iter(run.stdout.splitlines())
    max_xy = max_other = 0.0
    states = []
    boundary_differences = []
    for index, frame in enumerate(frames):
        state, count = map(int, next(rows).split())
        states.append(state)
        expected = frame["output"]
        actual = [dict(zip(FIELDS, map(float, next(rows).split())))
                  for _ in range(count)]
        reference_state = expected_state(frame)
        if reference_state is not None:
            assert state == reference_state, (index, state, reference_state)
        if len(actual) != len(expected):
            boundary_differences.append((index, len(actual), len(expected)))
            continue
        max_xy = max(max_xy, *(math.hypot(a["x"] - b["x"], a["y"] - b["y"])
                               for a, b in zip(actual, expected)))
        max_other = max(max_other, *(abs(a[field] - b[field])
                                    for a, b in zip(actual, expected)
                                    for field in FIELDS if field not in ("x", "y")))
    print(f"{len(frames)}프레임, 상태 전환 "
          f"{[(i, x) for i, x in enumerate(states) if i == 0 or x != states[i-1]]}, "
          f"최대 위치 차이 {max_xy:.6f}m, 나머지 차이 {max_other:.6f}, "
          f"경계 차이 {boundary_differences}")
    assert not boundary_differences
    assert max_xy < 0.001
    assert max_other < 0.001


if __name__ == "__main__":
    main()
