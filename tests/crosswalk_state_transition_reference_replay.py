"""합성 객체 위치 변경으로 발생한 원본 양보·무시 상태 전환을 비교합니다."""

import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET

from crosswalk_reference_replay import path_input


def crosswalk_polygon(map_path):
    root = ET.parse(map_path).getroot()
    relation = next(r for r in root.findall("relation") if r.get("id") == "15984")
    bounds = {m.get("role"): m.get("ref") for m in relation.findall("member")
              if m.get("role") in ("left", "right")}
    ways = {way.get("id"): [item.get("ref") for item in way.findall("nd")]
            for way in root.findall("way")
            if way.get("id") in bounds.values()}
    ids = set(ways[bounds["left"]] + ways[bounds["right"]])
    nodes = {node.get("id"): {
        tag.get("k"): float(tag.get("v")) for tag in node.findall("tag")
        if tag.get("k") in ("local_x", "local_y")
    } for node in root.findall("node") if node.get("id") in ids}
    ordered = ways[bounds["left"]] + list(reversed(ways[bounds["right"]]))
    return [(nodes[identifier]["local_x"], nodes[identifier]["local_y"])
            for identifier in ordered]


def polygon_distance(polygon, x, y):
    distances = []
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        dx, dy = x2 - x1, y2 - y1
        ratio = max(0.0, min(1.0,
            ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)))
        distances.append(math.hypot(x - x1 - ratio * dx,
                                    y - y1 - ratio * dy))
    return min(distances)


def target(frame):
    return next(obj for obj in frame["objects"] if obj["id"].startswith("a9b8"))


def collision(frame):
    matching = [item for item in frame["collision_info"]
                if item.startswith("24783,a9b8,")]
    if not matching:
        return None
    if len(matching) != 1:
        raise AssertionError("대상 객체 충돌 진단이 중복됐습니다")
    _, _, ttc, ttv, state, _ = matching[0].split(",")
    return float(ttc), float(ttv), int(state)


def ego_yielding(frames, index):
    now = frames[index]["replay_timestamp_ns"]
    speeds = []
    for frame in reversed(frames[:index + 1]):
        speeds.append(frame["ego"]["velocity_mps"])
        if (now - frame["replay_timestamp_ns"]) / 1e9 >= 1.0:
            break
    return all(speed < 0.001 for speed in speeds)


def core_states(runner, polygon, frames):
    comparable = [frame for frame in frames if collision(frame) is not None]
    if comparable != frames[:len(comparable)]:
        raise AssertionError("대상 객체의 충돌 진단이 중간에 끊겼습니다")
    first = frames[0]["replay_timestamp_ns"]
    lines = [str(len(comparable))]
    reference = []
    for index, frame in enumerate(comparable):
        obj = target(frame)
        ttc, ttv, state = collision(frame)
        shifted = frame["object_pose_shift_x"] != 0.0
        distance = polygon_distance(polygon, obj["x"], obj["y"]) if shifted else 0.0
        now = (frame["replay_timestamp_ns"] - first) / 1e9
        lines.append(
            f"{obj['id']} {now} {obj['velocity_x']} {obj['velocity_y']} "
            f"{ttc} {ttv} {int(ego_yielding(frames, index))} "
            f"{int(shifted)} {distance}")
        reference.append(state)
    process = subprocess.run(
        [runner], input="\n".join(lines) + "\n",
        text=True, capture_output=True, check=True,
    )
    core = [int(line) for line in process.stdout.splitlines()]
    if core != reference:
        raise AssertionError(f"원본 상태 {reference}, 독립 코어 상태 {core}")
    changes = [state for index, state in enumerate(reference)
               if index == 0 or state != reference[index - 1]]
    if changes != [0, 3, 0]:
        raise AssertionError(f"양보·무시·재양보 전환이 없습니다: {changes}")
    return core


def object_input(frame):
    obj = target(frame)
    ego = frame["ego"]
    data = (f"{ego['x']} {ego['y']} {ego['velocity_mps']} "
            f"{ego['acceleration_mps2']}\n")
    data += (f"{obj['length']} {obj['width']} {obj['velocity_x']} "
             f"{obj['velocity_y']} {len(obj['paths'])}\n")
    for predicted in obj["paths"]:
        data += f"{len(predicted['poses'])}\n"
        data += "\n".join(
            f"{pose['x']} {pose['y']} "
            f"{2 * math.atan2(pose['qz'], pose['qw'])}"
            for pose in predicted["poses"]
        ) + "\n"
    return data


def core_path(runner, map_path, frame, state):
    mode = 2 if state == 0 else 1
    process = subprocess.run(
        [runner, map_path, "15984"],
        input=path_input(frame) + object_input(frame) + f"{mode}\n",
        text=True, capture_output=True, check=True,
    )
    lines = process.stdout.splitlines()
    index = next(i for i, line in enumerate(lines)
                 if line.startswith("신호경로 "))
    count = int(lines[index].split()[1])
    output = [tuple(map(float, line.split())) for line in lines[index + 1:]]
    if len(output) != count:
        raise AssertionError("독립 코어의 전체 경로 점 개수가 잘못됐습니다")
    return output


def main():
    path_runner, state_runner, map_path, reference_path = sys.argv[1:5]
    with open(reference_path, encoding="utf-8") as stream:
        frames = json.load(stream)
    if len(frames) != 50:
        raise AssertionError("연속 재생 프레임 수가 다릅니다")
    if (frames[-1]["bag_timestamp"] - frames[0]["bag_timestamp"]) / 1e9 < 4.8:
        raise AssertionError("원본 입력 구간이 너무 짧습니다")
    if [frame["object_pose_shift_x"] for frame in frames[:10]] != [0.0] * 10:
        raise AssertionError("합성 위치 이동 시작점이 다릅니다")
    if any(frame["object_pose_shift_x"] != 15.0 for frame in frames[10:]):
        raise AssertionError("합성 위치 이동량이 다릅니다")
    states = core_states(state_runner, crosswalk_polygon(map_path), frames)
    maximum_xy = maximum_other = 0.0
    for index, frame in enumerate(frames):
        output = core_path(path_runner, map_path, frame,
                           states[index] if index < len(states) else -1)
        reference = frame["output"]
        if len(output) != len(reference):
            raise AssertionError(
                f"{index}번 프레임 경로 점 수: {len(output)}, 원본 {len(reference)}")
        maximum_xy = max(maximum_xy, *(
            math.hypot(point[0] - expected["x"],
                       point[1] - expected["y"])
            for point, expected in zip(output, reference)))
        maximum_other = max(maximum_other, *(
            abs(point[i] - expected[key])
            for point, expected in zip(output, reference)
            for i, key in enumerate(
                ("z", "velocity", "qx", "qy", "qz", "qw"), 2)))
    if maximum_xy >= 0.001 or maximum_other >= 0.001:
        raise AssertionError(
            f"전환 구간 경로 차이: 평면 {maximum_xy:.6f}m, 기타 {maximum_other:.6f}")
    print(f"50프레임 양보→무시→양보 전환 일치, "
          f"경로 최대 평면 차이 {maximum_xy:.6f}m, 기타 {maximum_other:.6f}")


if __name__ == "__main__":
    main()
