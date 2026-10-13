"""이진 지도 통합 실행기의 정체 차량 분기를 원본 경로와 비교합니다."""

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
    obj = reference["objects"][0]
    assert len(path) == len(lane_ids)
    payload = [ego["x"], ego["y"], ego["velocity_mps"],
               ego["acceleration_mps2"], len(path)]
    for point, ids in zip(path, lane_ids):
        payload.extend(point[field] for field in FIELDS)
        payload.append(len(ids))
        payload.extend(ids)
    payload.extend((obj["x"], obj["y"], obj["length"],
                    math.hypot(obj["velocity_x"], obj["velocity_y"]),
                    int(reference["occupancy_grid"].get("stuck_in_intersection", False))))
    run = subprocess.run(
        [str(runner), str(map_file), "crosswalk_stuck"],
        input=" ".join(map(str, payload)), text=True,
        capture_output=True, check=True)
    rows = run.stdout.splitlines()
    actual = [dict(zip(FIELDS, map(float, row.split()))) for row in rows[4:]]
    expected = reference["output"]
    assert len(actual) == int(rows[3]) == len(expected)
    max_xy = max(math.hypot(a["x"] - b["x"], a["y"] - b["y"])
                 for a, b in zip(actual, expected))
    max_other = max(abs(a[field] - b[field])
                    for a, b in zip(actual, expected)
                    for field in FIELDS if field not in ("x", "y"))
    print(f"{Path(reference['name']).stem}: {len(actual)}점, "
          f"위치 차이 {max_xy:.6f}m, 나머지 차이 {max_other:.6f}")
    assert max_xy < (0.02 if len(expected) == 101 else 0.001)
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
