"""원본 횡단보도 가림 감속 출력과 독립 코어의 경로를 비교합니다."""

import json
import math
import subprocess
import sys


def main():
    runner, map_path, clear_path, unknown_path = sys.argv[1:5]
    clear = json.load(open(clear_path, encoding="utf-8"))
    unknown = json.load(open(unknown_path, encoding="utf-8"))
    if clear["output"] == unknown["output"]:
        raise AssertionError("원본 가림 감속이 적용되지 않았습니다")
    meta = unknown["occupancy_grid"]
    if not meta["fill_unknown"] or not meta["occlusion_enabled"]:
        raise AssertionError("원본 가림 영역 재생 조건이 다릅니다")
    source = unknown["input"]
    ego = unknown["ego"]
    data = f"{len(source)}\n" + "\n".join(
        f"{p['x']} {p['y']} {p['z']} {p['velocity']}" for p in source
    ) + "\n"
    data += (f"{ego['x']} {ego['y']} {ego['velocity_mps']} "
             f"{ego['acceleration_mps2']}\n")
    data += (f"{meta['width']} {meta['height']} {meta['resolution']} "
             f"{meta['origin_x']} {meta['origin_y']} "
             f"{2.0 * math.atan2(meta['origin_qz'], meta['origin_qw'])}\n")
    completed = subprocess.run(
        [runner, map_path, "15984"], input=data,
        text=True, capture_output=True, check=True,
    )
    lines = completed.stdout.splitlines()
    output = [tuple(map(float, line.split())) for line in lines[1:]]
    reference = unknown["output"]
    if len(output) != int(lines[0]) or len(output) != len(reference):
        raise AssertionError("원본과 독립 코어의 출력 점 개수가 다릅니다")
    max_xy = max(math.hypot(p[0] - q["x"], p[1] - q["y"])
                 for p, q in zip(output, reference))
    max_z = max(abs(p[2] - q["z"])
                for p, q in zip(output, reference))
    max_vel = max(abs(p[3] - q["velocity"])
                  for p, q in zip(output, reference))
    max_orientation = max(
        abs(p[i] - q[key]) for p, q in zip(output, reference)
        for i, key in enumerate(("qx", "qy", "qz", "qw"), 4)
    )
    if (max_xy >= 0.001 or max_z >= 0.001 or
            max_vel >= 0.001 or max_orientation >= 0.001):
        raise AssertionError(
            f"가림 감속 경로 차이: 위치 {max_xy:.6f}m, 속도 {max_vel:.6f}m/s")
    print(f"가림 감속 경로 최대 차이: 위치 {max_xy:.6f}m, "
          f"높이 {max_z:.6f}m, 속도 {max_vel:.6f}m/s, "
          f"방향 {max_orientation:.6f}")


if __name__ == "__main__":
    main()
