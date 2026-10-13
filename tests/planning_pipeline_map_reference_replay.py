"""행동 속도 원본 사례를 지도 플러그인과 후속 독립 계산에 전달합니다."""

import json
import math
import subprocess
import sys
from pathlib import Path


def main():
    runner = Path(sys.argv[1])
    map_file = Path(sys.argv[2])
    reference = json.loads(Path(sys.argv[3]).read_text())
    fields = ("x", "y", "z", "qx", "qy", "qz", "qw", "velocity",
              "lateral_velocity", "heading_rate")
    ego = reference["ego"]
    source = reference["input"]
    lane_ids = reference["input_lane_ids"]
    assert len(source) == len(lane_ids)
    rows = [f"{ego['x']} {ego['y']} {ego['velocity_mps']} "
            f"{ego['acceleration_mps2']} {len(source)}"]
    for point, ids in zip(source, lane_ids):
        rows.append(" ".join(str(point[field]) for field in fields) +
                    " " + str(len(ids)) +
                    (" " + " ".join(map(str, ids)) if ids else ""))
    run = subprocess.run(
        [str(runner), str(map_file)], input="\n".join(rows) + "\n",
        text=True, capture_output=True, check=True)
    output = run.stdout.splitlines()
    status, behavior_count, smooth_count, optimized_count, motion_count, final_count, validated = (
        map(int, output[0].split()))
    actual = [tuple(map(float, row.split())) for row in output[1:]]
    expected = reference["output"]
    assert len(actual) == behavior_count == len(expected), (
        len(actual), behavior_count, len(expected), output[0])
    position_error = max(
        math.hypot(point[0] - target["x"], point[1] - target["y"])
        for point, target in zip(actual, expected))
    velocity_error = max(
        abs(point[7] - target["velocity"])
        for point, target in zip(actual, expected))
    print(f"행동 속도 {behavior_count}점, 위치 차이 {position_error:.6f}m, "
          f"속도 차이 {velocity_error:.6f}m/s, "
          f"후속 평활화·최적화·동작 속도·최종 "
          f"{smooth_count}/{optimized_count}/{motion_count}/{final_count}점, "
          f"검증 {validated}, 상태 {status}")
    assert position_error < 0.001 and velocity_error < 0.001
    assert status == 0 and all((smooth_count, optimized_count,
                                motion_count, final_count, validated))


if __name__ == "__main__":
    main()
