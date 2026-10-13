"""K-City 보행자 기록의 횡단보도 기본 정지점을 원본과 비교합니다."""

import json
import math
import subprocess
import sys


def object_input(reference):
    object_id = next(item.split(",")[1] for item in reference["collision_info"]
                     if ",0," in item)
    obj = next(o for o in reference["objects"] if o["id"].startswith(object_id))
    ego = reference["ego"]
    data = (f"{ego['x']} {ego['y']} {ego['velocity_mps']} "
            f"{ego['acceleration_mps2']}\n")
    data += (f"{obj['length']} {obj['width']} {obj['velocity_x']} "
             f"{obj['velocity_y']} {len(obj['paths'])}\n")
    for predicted in obj["paths"]:
        data += f"{len(predicted['poses'])}\n"
        data += "\n".join(
            f"{pose['x']} {pose['y']} "
            f"{2.0 * math.atan2(pose['qz'], pose['qw'])}"
            for pose in predicted["poses"]
        ) + "\n"
    return data


def path_input(reference):
    source = reference["input"]
    return f"{len(source)}\n" + "\n".join(
        f"{p['x']} {p['y']} {p['z']} {p['velocity']}" for p in source
    ) + "\n"


def main():
    runner, map_path, reference_path, empty_reference_path = sys.argv[1:5]
    detailed = json.load(open(sys.argv[5], encoding="utf-8")) if len(sys.argv) > 5 else None
    reference = json.load(open(reference_path, encoding="utf-8"))
    empty_reference = json.load(open(empty_reference_path, encoding="utf-8"))
    if not any(",0," in item for item in reference["collision_info"]):
        raise AssertionError("원본의 양보 충돌 진단이 없습니다")
    if any(empty_reference["collision_info"]):
        raise AssertionError("객체 제거 재생에 충돌 진단이 남았습니다")
    if reference["output"] != empty_reference["output"]:
        raise AssertionError("객체 제거 전후 원본 경로가 달라졌습니다")
    data = path_input(reference)
    if detailed:
        data += object_input(detailed)
    completed = subprocess.run(
        [runner, map_path, "15984"], input=data, text=True,
        capture_output=True, check=True,
    )
    lines = completed.stdout.splitlines()
    _, _, x, y = map(float, lines[0].split())
    output = reference["output"]
    first_zero = next(p for p in output if p["velocity"] == 0.0)
    distance = math.hypot(x - first_zero["x"], y - first_zero["y"])
    if distance >= 0.001:
        raise AssertionError(f"원본 정지점과 위치 차이: {distance:.9f}m")
    print(f"K-City 횡단보도 기본 정지점 차이: {distance:.9f}m")
    print("객체 제거 전후 원본 출력은 동일하며 충돌 진단만 달라집니다")
    core_path = [tuple(map(float, line.split()))
                 for line in lines[2:2 + int(lines[1])]]
    if len(core_path) != int(lines[1]):
        raise AssertionError("독립 코어의 출력 점 개수가 잘못되었습니다")
    if len(core_path) != len(output):
        raise AssertionError(
            f"전체 경로 점 개수: 독립 코어 {len(core_path)}, 원본 {len(output)}")
    max_xy = max(math.hypot(p[0] - q["x"], p[1] - q["y"])
                 for p, q in zip(core_path, output))
    max_z = max(abs(p[2] - q["z"]) for p, q in zip(core_path, output))
    max_vel = max(abs(p[3] - q["velocity"])
                  for p, q in zip(core_path, output))
    max_orientation = max(
        abs(p[i] - q[key]) for p, q in zip(core_path, output)
        for i, key in enumerate(("qx", "qy", "qz", "qw"), 4)
    )
    if max_xy >= 0.001 or max_z >= 0.001 or max_vel >= 0.001 or max_orientation >= 0.001:
        raise AssertionError("원본 전체 경로와의 차이가 허용 범위를 넘었습니다")
    print(f"전체 경로 최대 차이: 위치 {max_xy:.6f}m, 높이 {max_z:.6f}m, "
          f"속도 {max_vel:.6f}m/s, 방향 {max_orientation:.6f}")
    if detailed:
        collision = lines[len(core_path) + 2].split()
        if collision[0] != "충돌":
            raise AssertionError("실측 보행자 충돌점을 찾지 못했습니다")
        _, _, expected_ttc, expected_ttv, _, _ = next(
            item.split(",") for item in detailed["collision_info"] if ",0," in item)
        if (abs(float(collision[5]) - float(expected_ttc)) >= 0.03 or
                abs(float(collision[6]) - float(expected_ttv)) >= 0.03):
            raise AssertionError("실측 보행자의 TTC·TTV 차이가 큽니다")
        print(f"실측 보행자 TTC·TTV: {collision[5]}초, {collision[6]}초")

    if len(sys.argv) > 6:
        shifted = json.load(open(sys.argv[6], encoding="utf-8"))
        if shifted.get("object_adjustment") != {
                "id_prefix": "a9b8", "shift_y_m": -2.0,
                "only_selected_object": True}:
            raise AssertionError("합성 객체의 이동 조건이 다릅니다")
        if shifted["output"] == empty_reference["output"]:
            raise AssertionError("객체 이동 후 원본 정지 경로가 바뀌지 않았습니다")
        shifted_run = subprocess.run(
            [runner, map_path, "15984"],
            input=path_input(shifted) + object_input(shifted),
            text=True, capture_output=True, check=True,
        )
        shifted_lines = shifted_run.stdout.splitlines()
        count = int(shifted_lines[1])
        collision = shifted_lines[count + 2].split()
        stop = shifted_lines[count + 3].split()
        if collision[0] != "충돌" or stop[0] != "객체정지":
            raise AssertionError("이동 객체의 충돌 또는 정지점을 찾지 못했습니다")
        shifted_path = [tuple(map(float, line.split())) for line in
                        shifted_lines[count + 4:]]
        original_path = shifted["output"]
        if len(shifted_path) != len(original_path):
            raise AssertionError("이동 객체의 출력 점 개수가 다릅니다")
        distance = max(math.hypot(p[0] - q["x"], p[1] - q["y"])
                       for p, q in zip(shifted_path, original_path))
        velocity = max(abs(p[3] - q["velocity"])
                       for p, q in zip(shifted_path, original_path))
        height = max(abs(p[2] - q["z"])
                     for p, q in zip(shifted_path, original_path))
        orientation = max(
            abs(p[i] - q[key]) for p, q in zip(shifted_path, original_path)
            for i, key in enumerate(("qx", "qy", "qz", "qw"), 4)
        )
        if (distance >= 0.02 or velocity >= 0.001 or
                height >= 0.001 or orientation >= 0.001):
            raise AssertionError("이동 객체 정지 경로가 원본과 다릅니다")
        print(f"이동 객체 정지 경로 최대 차이: 위치 {distance:.6f}m, "
              f"높이 {height:.6f}m, 속도 {velocity:.6f}m/s, "
              f"방향 {orientation:.6f}")


if __name__ == "__main__":
    main()
