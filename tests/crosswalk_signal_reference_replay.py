"""원본 횡단보도 보행자 신호 색에 따른 전체 경로와 독립 코어를 비교합니다."""

import json
import math
import subprocess
import sys

from crosswalk_reference_replay import object_input, path_input


def core_path(runner, map_path, reference, color, object_reference):
    data = path_input(reference) + object_input(object_reference)
    data += f"{1 if color == 'red' else 2}\n"
    result = subprocess.run(
        [runner, map_path, "15984"], input=data, text=True,
        capture_output=True, check=True,
    )
    lines = result.stdout.splitlines()
    index = next(i for i, line in enumerate(lines)
                 if line.startswith("신호경로 "))
    count = int(lines[index].split()[1])
    return [tuple(map(float, line.split()))
            for line in lines[index + 1:index + 1 + count]]


def compare(core, original, tolerance):
    if len(core) != len(original):
        raise AssertionError("원본과 독립 코어의 경로 점 개수가 다릅니다")
    max_xy = max(math.hypot(point[0] - expected["x"],
                            point[1] - expected["y"])
                 for point, expected in zip(core, original))
    max_other = max(
        abs(point[i] - expected[key])
        for point, expected in zip(core, original)
        for i, key in enumerate(("z", "velocity", "qx", "qy", "qz", "qw"), 2)
    )
    if max_xy >= tolerance or max_other >= 0.001:
        raise AssertionError(
            f"원본 경로 차이: 평면 {max_xy:.6f}m, 그 외 {max_other:.6f}")
    return max_xy, max_other


def main():
    runner, map_path, green_path, red_path, clear_path, shifted_path = sys.argv[1:7]
    with open(green_path, encoding="utf-8") as stream:
        green = json.load(stream)
    with open(red_path, encoding="utf-8") as stream:
        red = json.load(stream)
    with open(clear_path, encoding="utf-8") as stream:
        clear = json.load(stream)
    with open(shifted_path, encoding="utf-8") as stream:
        shifted = json.load(stream)
    if green["input"] != red["input"] or green["objects"] != red["objects"]:
        raise AssertionError("신호 색 비교의 경로 또는 객체가 다릅니다")
    if green["pedestrian_signal"]["color"] != "green":
        raise AssertionError("녹색 신호 조건이 다릅니다")
    if red["pedestrian_signal"]["color"] != "red":
        raise AssertionError("적색 신호 조건이 다릅니다")
    if green["output"] != shifted["output"]:
        raise AssertionError("녹색 신호의 객체 정지 경로가 기존 원본과 다릅니다")
    if red["output"] != clear["output"]:
        raise AssertionError("적색 신호의 통과 경로가 빈 격자 원본과 다릅니다")
    if not any(",0," in item for item in green["collision_info"]):
        raise AssertionError("녹색 신호에서 객체 충돌 진단을 찾지 못했습니다")
    if any(red["collision_info"]):
        raise AssertionError("적색 신호에서 객체 충돌 진단이 남았습니다")
    green_error = compare(core_path(runner, map_path, green, "green", green),
                          green["output"], 0.02)
    red_error = compare(core_path(runner, map_path, red, "red", green),
                        red["output"], 0.001)
    print(f"녹색 정지 경로 최대 차이 {green_error[0]:.6f}m, "
          f"적색 통과 경로 최대 차이 {red_error[0]:.6f}m")


if __name__ == "__main__":
    main()
