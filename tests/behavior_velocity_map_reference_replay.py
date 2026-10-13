"""지도에서 모듈을 선택한 뒤 원본 행동 속도 계획 출력과 비교합니다."""

import json
import math
import subprocess
import sys
from pathlib import Path


def main():
    runner = Path(sys.argv[1])
    map_file = Path(sys.argv[2])
    mode = sys.argv[3]
    reference = json.loads(Path(sys.argv[4]).read_text())
    original_log = Path(sys.argv[4]).with_suffix(".log").read_text()
    assert "The scene plugin" not in original_log
    path = reference["input"]
    lane_ids = reference["input_lane_ids"]
    assert len(path) == len(lane_ids)
    ego = reference["ego"]
    fields = ("x", "y", "z", "qx", "qy", "qz", "qw", "velocity",
              "lateral_velocity", "heading_rate")
    payload = [ego["x"], ego["y"], ego["velocity_mps"],
               ego["acceleration_mps2"], len(path)]
    for point, ids in zip(path, lane_ids):
        payload.extend(point[field] for field in fields)
        payload.append(len(ids))
        payload.extend(ids)
    run = subprocess.run(
        [str(runner), str(map_file), mode],
        input=" ".join(map(str, payload)), text=True,
        capture_output=True, check=True)
    rows = run.stdout.splitlines()
    selected = [list(map(int, row.split()[1:])) for row in rows[:3]]
    sizes = [int(row.split()[0]) for row in rows[:3]]
    assert [len(ids) for ids in selected] == sizes
    if mode == "combined":
        assert reference["modules"] == ["stop_line", "traffic_light"]
        assert selected == [[900000007], [25004], []], selected
    elif mode == "crosswalk":
        assert reference["modules"] == ["crosswalk"]
        assert selected == [[], [], [24783]], selected
    elif mode == "walkway":
        assert reference["modules"] == ["walkway"]
        assert selected == [[], [], [24783]], selected
    actual = [dict(zip(fields, map(float, row.split())))
              for row in rows[4:]]
    expected = reference["output"]
    assert len(actual) == int(rows[3])
    assert len(actual) == len(expected), (
        f"원본 {len(expected)}점, 코어 {len(actual)}점")
    position_error = max(math.hypot(a["x"] - b["x"], a["y"] - b["y"])
                         for a, b in zip(actual, expected))
    other_error = max(abs(a[field] - b[field])
                      for a, b in zip(actual, expected)
                      for field in fields if field not in ("x", "y"))
    print(f"{mode}: 선택 {selected}, {len(actual)}점, "
          f"위치 차이 {position_error:.6f}m, 나머지 차이 {other_error:.6f}")
    assert position_error < 0.001
    assert other_error < 0.001


if __name__ == "__main__":
    main()
