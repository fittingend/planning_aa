"""이진 지도 기반 횡단보도 객체·보행자 신호 출력을 원본과 비교합니다."""

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
    assert len(path) == len(lane_ids)
    payload = [ego["x"], ego["y"], ego["velocity_mps"],
               ego["acceleration_mps2"], len(path)]
    for point, ids in zip(path, lane_ids):
        payload.extend(point[field] for field in FIELDS)
        payload.append(len(ids))
        payload.extend(ids)
    signal = reference["pedestrian_signal"]
    payload.extend((signal["regulatory_id"],
                    1 if signal["color"] == "red" else 2,
                    len(reference["objects"])))
    for obj in reference["objects"]:
        payload.extend((obj["id"], obj["x"], obj["y"], obj["label"],
                        obj["length"], obj["width"],
                        obj["velocity_x"], obj["velocity_y"],
                        len(obj["paths"])))
        for predicted in obj["paths"]:
            poses = predicted["poses"]
            payload.append(len(poses))
            for pose in poses:
                payload.extend((pose["x"], pose["y"],
                                2 * math.atan2(pose["qz"], pose["qw"])))
    run = subprocess.run(
        [str(runner), str(map_file), "crosswalk_signal"],
        input=" ".join(map(str, payload)), text=True,
        capture_output=True, check=True)
    rows = run.stdout.splitlines()
    selected = list(map(int, rows[2].split()[1:]))
    actual = [dict(zip(FIELDS, map(float, row.split()))) for row in rows[4:]]
    expected = reference["output"]
    assert len(actual) == int(rows[3])
    assert len(actual) == len(expected), (len(actual), len(expected))
    max_xy = max(math.hypot(a["x"] - b["x"], a["y"] - b["y"])
                 for a, b in zip(actual, expected))
    max_other = max(abs(a[field] - b[field])
                    for a, b in zip(actual, expected)
                    for field in FIELDS if field not in ("x", "y"))
    print(f"{signal['color']}: 선택 {selected}, {len(actual)}점, "
          f"위치 차이 {max_xy:.6f}m, 나머지 차이 {max_other:.6f}")
    assert selected == ([24783] if signal["color"] == "green" else [])
    assert max_xy < (0.02 if signal["color"] == "green" else 0.001)
    assert max_other < 0.001


def main():
    runner, map_file, lane_file, green_file, red_file = map(Path, sys.argv[1:6])
    lane_ids = json.loads(lane_file.read_text())["input_lane_ids"]
    green = json.loads(green_file.read_text())
    red = json.loads(red_file.read_text())
    assert green["input"] == red["input"]
    for case in (green, red):
        run_case(runner, map_file, lane_ids, case)


if __name__ == "__main__":
    main()
