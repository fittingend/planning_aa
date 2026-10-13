"""이진 지도 통합 실행기의 가림 감속을 원본 횡단보도 출력과 비교합니다."""

import json
import math
import subprocess
import sys
from pathlib import Path


FIELDS = ("x", "y", "z", "qx", "qy", "qz", "qw", "velocity",
          "lateral_velocity", "heading_rate")


def run_case(runner, map_file, lane_ids, reference):
    ego = reference["ego"]
    path = reference["input"]
    grid = reference["occupancy_grid"]
    assert len(path) == len(lane_ids)
    payload = [ego["x"], ego["y"], ego["velocity_mps"],
               ego["acceleration_mps2"], len(path)]
    for point, ids in zip(path, lane_ids):
        payload.extend(point[field] for field in FIELDS)
        payload.append(len(ids))
        payload.extend(ids)
    patch = grid.get("unknown_patch") or (0.0, 0.0, 0.0)
    mask = grid.get("mask_object") or (0.0, 0.0, -1.0)
    payload.extend((grid["width"], grid["height"], grid["resolution"],
                    grid["origin_x"], grid["origin_y"],
                    2 * math.atan2(grid["origin_qz"], grid["origin_qw"]),
                    int(grid["occlusion_enabled"]), int(grid["fill_unknown"]),
                    *patch, *mask))
    run = subprocess.run(
        [str(runner), str(map_file), "crosswalk_occlusion"],
        input=" ".join(map(str, payload)), text=True,
        capture_output=True, check=True)
    rows = run.stdout.splitlines()
    selected = list(map(int, rows[2].split()[1:]))
    actual = [dict(zip(FIELDS, map(float, row.split()))) for row in rows[4:]]
    expected = reference["output"]
    assert len(actual) == int(rows[3]) == len(expected)
    max_xy = max(math.hypot(a["x"] - b["x"], a["y"] - b["y"])
                 for a, b in zip(actual, expected))
    max_other = max(abs(a[field] - b[field])
                    for a, b in zip(actual, expected)
                    for field in FIELDS if field not in ("x", "y"))
    print(f"{Path(reference['name']).stem}: 선택 {selected}, {len(actual)}점, "
          f"위치 차이 {max_xy:.6f}m, 나머지 차이 {max_other:.6f}")
    assert max_xy < 0.001
    assert max_other < 0.001


def main():
    runner, map_file, lane_file = map(Path, sys.argv[1:4])
    lane_ids = json.loads(lane_file.read_text())["input_lane_ids"]
    for filename in sys.argv[4:]:
        reference = json.loads(Path(filename).read_text())
        reference["name"] = filename
        run_case(runner, map_file, lane_ids, reference)


if __name__ == "__main__":
    main()
