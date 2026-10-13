"""신호등 코어의 첫 정지점을 원본 planner 재생 결과와 비교합니다."""

import json
import math
import subprocess
import sys
from pathlib import Path


def main():
    runner = Path(sys.argv[1])
    reference = json.loads(Path(sys.argv[2]).read_text())
    line = reference["stop_line"]
    ego = reference["ego"]
    path = reference["input"]
    payload = [
        line["x1"], line["y1"], line["x2"], line["y2"],
        reference["front_offset_m"], ego["x"], ego["y"],
        ego["velocity_mps"], ego["acceleration_mps2"], len(path),
    ]
    fields = ("x", "y", "z", "qx", "qy", "qz", "qw", "velocity",
              "lateral_velocity", "heading_rate")
    for point in path:
        payload.extend(point[field] for field in fields)
    completed = subprocess.run(
        [str(runner)], input=" ".join(map(str, payload)), text=True,
        capture_output=True, check=True,
    )
    lines = completed.stdout.splitlines()
    actual = [dict(zip(fields, map(float, row.split())))
              for row in lines[1:]]
    assert len(actual) == int(lines[0])
    expected = reference["output"]
    assert len(actual) == len(expected), (
        f"원본 {len(expected)}점, 코어 {len(actual)}점")
    position_error = max(math.hypot(a["x"] - b["x"], a["y"] - b["y"])
                         for a, b in zip(actual, expected))
    velocity_error = max(abs(a["velocity"] - b["velocity"])
                         for a, b in zip(actual, expected))
    z_error = max(abs(a["z"] - b["z"]) for a, b in zip(actual, expected))
    orientation_error = max(max(abs(a[field] - b[field])
                                for field in ("qx", "qy", "qz", "qw"))
                            for a, b in zip(actual, expected))
    worst = max(range(len(actual)), key=lambda i: math.hypot(
        actual[i]["x"] - expected[i]["x"],
        actual[i]["y"] - expected[i]["y"]))
    print(f"최대 위치 차이: {position_error:.6f}m (인덱스 {worst}), "
          f"높이 차이: {z_error:.6f}m, "
          f"방향 성분 차이: {orientation_error:.6f}, "
          f"최대 속도 차이: {velocity_error:.6f}m/s")
    assert position_error < 0.001, f"최대 위치 차이 {position_error:.6f}m"
    assert z_error < 0.001, f"최대 높이 차이 {z_error:.6f}m"
    assert orientation_error < 0.001, (
        f"최대 방향 성분 차이 {orientation_error:.6f}")
    assert velocity_error < 0.001, f"최대 속도 차이 {velocity_error:.6f}m/s"
    for field in ("lateral_velocity", "heading_rate"):
        difference = max(abs(a[field] - b[field]) for a, b in zip(actual, expected))
        assert difference < 0.001, f"{field} 최대 차이 {difference:.6f}"


if __name__ == "__main__":
    main()
