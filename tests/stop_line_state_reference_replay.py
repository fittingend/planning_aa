"""원본 정지 표지의 연속 정차·출발 출력과 코어 상태를 비교합니다."""

import json
import math
import subprocess
import sys
from pathlib import Path


def main():
    runner = Path(sys.argv[1])
    reference = json.loads(Path(sys.argv[2]).read_text())
    original_log = Path(sys.argv[2]).with_suffix(".log").read_text()
    assert "APPROACH -> STOPPED" in original_log
    assert "STOPPED -> START" in original_log
    line = reference["stop_line"]
    path = reference["input"]
    frames = reference["frames"]
    fields = ("x", "y", "z", "qx", "qy", "qz", "qw", "velocity",
              "lateral_velocity", "heading_rate")
    payload = [line["x1"], line["y1"], line["x2"], line["y2"],
               reference["front_offset_m"], len(path), len(frames)]
    for point in path:
        payload.extend(point[field] for field in fields)
    initial_time_ns = frames[0]["replay_timestamp_ns"]
    for frame in frames:
        ego = frame["ego"]
        payload.extend([(frame["replay_timestamp_ns"] - initial_time_ns) / 1e9,
                        ego["x"], ego["y"], ego["velocity_mps"]])
    output = subprocess.run(
        [str(runner)], input=" ".join(map(str, payload)), text=True,
        capture_output=True, check=True).stdout.splitlines()
    rows = iter(output)
    states = []
    worst_position = 0.0
    worst_other = 0.0
    first_zeros = []
    for frame in frames:
        state, count, _ = next(rows).split()
        states.append(int(state))
        actual = [dict(zip(fields, map(float, next(rows).split())))
                  for _ in range(int(count))]
        expected = frame["output"]
        assert len(actual) == len(expected), (
            f"원본 {len(expected)}점, 코어 {len(actual)}점")
        actual_zero = next(i for i, point in enumerate(actual)
                           if abs(point["velocity"]) < 1e-4)
        expected_zero = next(i for i, point in enumerate(expected)
                             if abs(point["velocity"]) < 1e-4)
        first_zeros.append(expected_zero)
        assert actual_zero == expected_zero
        worst_position = max(worst_position, *(
            math.hypot(a["x"] - b["x"], a["y"] - b["y"])
            for a, b in zip(actual, expected)))
        worst_other = max(worst_other, *(
            abs(a[field] - b[field])
            for a, b in zip(actual, expected)
            for field in fields if field not in ("x", "y")))
    assert states == [0, 1, 1, 2, 2, 2, 2], states
    assert first_zeros == [9, 9, 10, 10, 95, 95, 95], first_zeros
    print(f"상태 {states}, 첫 정지점 {first_zeros}, "
          f"최대 평면 차이 {worst_position:.6f}m, 나머지 최대 차이 {worst_other:.6f}")
    assert worst_position < 0.001
    assert worst_other < 0.001


if __name__ == "__main__":
    main()
