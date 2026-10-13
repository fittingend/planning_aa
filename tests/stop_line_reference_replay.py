"""합성 정지 표지 지도로 얻은 원본 출력을 독립 코어와 점별 비교합니다."""

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
    payload = [line["x1"], line["y1"], line["x2"], line["y2"],
               reference["front_offset_m"], ego["x"], ego["y"], len(path)]
    fields = ("x", "y", "z", "qx", "qy", "qz", "qw", "velocity",
              "lateral_velocity", "heading_rate")
    for point in path:
        payload.extend(point[field] for field in fields)
    completed = subprocess.run(
        [str(runner)], input=" ".join(map(str, payload)), text=True,
        capture_output=True, check=True)
    rows = completed.stdout.splitlines()
    count, stop_arc = rows[0].split()
    actual = [dict(zip(fields, map(float, row.split()))) for row in rows[1:]]
    expected = reference["output"]
    assert len(actual) == int(count)
    assert len(actual) == len(expected), (
        f"원본 {len(expected)}점, 코어 {len(actual)}점")
    errors = {field: max(abs(a[field] - b[field])
                         for a, b in zip(actual, expected)) for field in fields}
    position_error = max(math.hypot(a["x"] - b["x"], a["y"] - b["y"])
                         for a, b in zip(actual, expected))
    first_zero = next(i for i, p in enumerate(actual) if abs(p["velocity"]) < 1e-4)
    expected_zero = next(i for i, p in enumerate(expected) if abs(p["velocity"]) < 1e-4)
    print(f"정지 호길이 {float(stop_arc):.6f}m, 첫 정지점 {first_zero}/{expected_zero}, "
          f"최대 평면 위치 차이 {position_error:.6f}m, "
          f"높이 차이 {errors['z']:.6f}m, 속도 차이 {errors['velocity']:.6f}m/s")
    assert first_zero == expected_zero
    assert first_zero < len(actual) - 1
    assert position_error < 0.001
    assert all(error < 0.001 for field, error in errors.items()
               if field not in ("x", "y")), errors


if __name__ == "__main__":
    main()
