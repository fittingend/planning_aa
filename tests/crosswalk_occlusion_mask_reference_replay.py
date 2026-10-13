"""실측 K-City 경로에 합성 가림·이동 객체를 주입한 원본 출력과 비교합니다."""

import json
import math
import subprocess
import sys


def run_core(runner, map_path, case, mask_speed):
    source = case["input"]
    ego = case["ego"]
    grid = case["occupancy_grid"]
    patch = grid["unknown_patch"]
    text = f"{len(source)}\n"
    text += "\n".join(
        f"{p['x']} {p['y']} {p['z']} {p['velocity']}" for p in source
    ) + "\n"
    text += (f"{ego['x']} {ego['y']} {ego['velocity_mps']} "
             f"{ego['acceleration_mps2']}\n")
    text += (f"{grid['width']} {grid['height']} {grid['resolution']} "
             f"{grid['origin_x']} {grid['origin_y']} "
             f"{2 * math.atan2(grid['origin_qz'], grid['origin_qw'])}\n")
    text += f"1\n{patch[0]} {patch[1]} {patch[2]}\n"
    text += f"2597 24102 {mask_speed}\n"
    completed = subprocess.run(
        [runner, map_path, "15984"], input=text, text=True,
        capture_output=True, check=True,
    )
    lines = completed.stdout.splitlines()
    result = [tuple(map(float, line.split())) for line in lines[1:]]
    if len(result) != int(lines[0]):
        raise AssertionError("독립 코어 출력 개수가 잘못되었습니다")
    return result


def compare(output, reference):
    if len(output) != len(reference):
        raise AssertionError("원본과 독립 코어의 출력 점 개수가 다릅니다")
    maximum = max(
        max(abs(point[i] - expected[key]) for i, key in enumerate(
            ("x", "y", "z", "velocity", "qx", "qy", "qz", "qw")))
        for point, expected in zip(output, reference)
    )
    if maximum >= 0.001:
        raise AssertionError(f"원본과 독립 코어의 경로 성분 차이: {maximum:.6f}")
    return maximum


def main():
    runner, map_path, patch_path, mask_path, slow_path, clear_path = sys.argv[1:7]
    with open(patch_path, encoding="utf-8") as stream:
        patch = json.load(stream)
    with open(mask_path, encoding="utf-8") as stream:
        masked = json.load(stream)
    with open(slow_path, encoding="utf-8") as stream:
        slow_object = json.load(stream)
    with open(clear_path, encoding="utf-8") as stream:
        clear = json.load(stream)
    if patch["output"] == masked["output"]:
        raise AssertionError("원본 이동 객체가 가림 영역을 제거하지 않았습니다")
    if masked["output"] != clear["output"]:
        raise AssertionError("원본 차폐 경로가 빈 격자 경로와 다릅니다")
    if slow_object["output"] != patch["output"]:
        raise AssertionError("속도 기준 미만 객체가 원본 가림 영역을 제거했습니다")
    if patch["input"] != masked["input"]:
        raise AssertionError("비교 입력 경로가 서로 다릅니다")
    patch_error = compare(run_core(runner, map_path, patch, -1.0), patch["output"])
    mask_error = compare(run_core(runner, map_path, masked, 1.0), masked["output"])
    slow_error = compare(
        run_core(runner, map_path, slow_object, 0.1), slow_object["output"])
    print(f"합성 가림 경로 최대 성분 차이 {patch_error:.6f}, "
          f"이동 객체 차폐 경로 최대 성분 차이 {mask_error:.6f}, "
          f"저속 객체 경로 최대 성분 차이 {slow_error:.6f}")


if __name__ == "__main__":
    main()
