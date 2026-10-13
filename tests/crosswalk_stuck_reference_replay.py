"""원본 횡단보도 정체 차량 정지 경로와 독립 코어를 비교합니다."""

import json
import math
import subprocess
import sys


def run_case(runner, map_path, case):
    source = case["input"]
    ego = case["ego"]
    obj = case["objects"][0]
    meta = case["occupancy_grid"]
    data = f"{len(source)}\n"
    data += "\n".join(
        f"{p['x']} {p['y']} {p['z']} {p['velocity']}" for p in source
    ) + "\n"
    data += (f"{ego['x']} {ego['y']} {ego['velocity_mps']} "
             f"{ego['acceleration_mps2']}\n")
    data += (f"{obj['x']} {obj['y']} {obj['length']} "
             f"{obj['velocity_x']}\n")
    data += f"{int(meta.get('stuck_in_intersection', False))}\n"
    completed = subprocess.run(
        [runner, map_path, "15984"], input=data, text=True,
        capture_output=True, check=True,
    )
    lines = completed.stdout.splitlines()
    output = [tuple(map(float, line.split())) for line in lines[1:]]
    if len(output) != int(lines[0]):
        raise AssertionError("독립 코어 출력 점 개수가 잘못되었습니다")
    return output


def compare(output, reference, tolerance):
    if len(output) != len(reference):
        raise AssertionError(
            f"원본 {len(reference)}점, 독립 코어 {len(output)}점")
    max_xy = max(math.hypot(p[0] - q["x"], p[1] - q["y"])
                 for p, q in zip(output, reference))
    max_other = max(
        abs(p[i] - q[key]) for p, q in zip(output, reference)
        for i, key in enumerate(("z", "velocity", "qx", "qy", "qz", "qw"), 2)
    )
    if max_xy >= tolerance or max_other >= 0.001:
        raise AssertionError(
            f"정체 차량 경로 차이: 위치 {max_xy:.6f}m, 기타 {max_other:.6f}")
    return max_xy


def main():
    runner, map_path, disabled_path, stopped_path, fast_path, clear_path = sys.argv[1:7]
    cases = []
    for path in (disabled_path, stopped_path, fast_path, clear_path):
        with open(path, encoding="utf-8") as stream:
            cases.append(json.load(stream))
    disabled, stopped, fast, clear = cases
    if stopped["output"] == clear["output"]:
        raise AssertionError("원본 정체 차량 정지가 적용되지 않았습니다")
    if disabled["output"] != clear["output"]:
        raise AssertionError("원본 교차로 검사 비활성 경로가 다릅니다")
    if fast["output"] != clear["output"]:
        raise AssertionError("원본 속도 기준 초과 경로가 다릅니다")
    if not (len(stopped["output"]) == 101 and
            len(disabled["output"]) == len(fast["output"]) == 100):
        raise AssertionError("원본 경로 점 개수가 예상과 다릅니다")
    errors = [
        compare(run_case(runner, map_path, case), case["output"],
                0.02 if case is stopped else 0.001)
        for case in (disabled, stopped, fast)
    ]
    print("정체 차량 원본 비교 최대 평면 위치 차이: " +
          ", ".join(f"{error:.6f}m" for error in errors))


if __name__ == "__main__":
    main()
