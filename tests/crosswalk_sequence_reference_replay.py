"""원본 횡단보도 20프레임의 객체 상태와 전체 경로를 독립 코어와 비교합니다."""

import json
import math
import subprocess
import sys

from crosswalk_reference_replay import object_input, path_input


def core_path(runner, map_path, frame):
    process = subprocess.run(
        [runner, map_path, "15984"],
        input=path_input(frame) + object_input(frame),
        text=True, capture_output=True, check=True,
    )
    lines = process.stdout.splitlines()
    index = next(i for i, line in enumerate(lines)
                 if line.startswith("객체정지 "))
    return [tuple(map(float, line.split())) for line in lines[index + 1:]]


def main():
    path_runner, state_runner, map_path, reference_path = sys.argv[1:5]
    with open(reference_path, encoding="utf-8") as stream:
        frames = json.load(stream)
    if len(frames) != 20:
        raise AssertionError("연속 재생 프레임 수가 다릅니다")
    timestamps = [frame["bag_timestamp"] for frame in frames]
    if any(b <= a for a, b in zip(timestamps, timestamps[1:])):
        raise AssertionError("rosbag 프레임 시각이 증가하지 않습니다")
    if (timestamps[-1] - timestamps[0]) / 1e9 < 1.8:
        raise AssertionError("연속 재생 시간이 너무 짧습니다")
    maximum_xy = maximum_other = 0.0
    states = []
    for frame in frames:
        diagnostics = [item for item in frame["collision_info"]
                       if item.startswith("24783,a9b8,")]
        if len(diagnostics) != 1:
            raise AssertionError("대상 보행자의 충돌 진단이 누락됐습니다")
        _, prefix, ttc, ttv, state, _ = diagnostics[0].split(",")
        obj = next(item for item in frame["objects"]
                   if item["id"].startswith(prefix))
        states.append((obj["id"], obj["velocity_x"], obj["velocity_y"],
                       float(ttc), float(ttv), int(state)))
        output = core_path(path_runner, map_path, frame)
        reference = frame["output"]
        if len(output) != len(reference):
            raise AssertionError("연속 프레임의 경로 점 개수가 다릅니다")
        maximum_xy = max(maximum_xy, *(
            math.hypot(point[0] - expected["x"],
                       point[1] - expected["y"])
            for point, expected in zip(output, reference)))
        maximum_other = max(maximum_other, *(
            abs(point[i] - expected[key])
            for point, expected in zip(output, reference)
            for i, key in enumerate(
                ("z", "velocity", "qx", "qy", "qz", "qw"), 2)))
    data = f"{len(states)}\n"
    data += "\n".join(
        f"{identifier} {(timestamp - timestamps[0]) / 1e9} "
        f"{speed_x} {speed_y} {ttc} {ttv}"
        for timestamp, (identifier, speed_x, speed_y, ttc, ttv, _)
        in zip(timestamps, states)
    ) + "\n"
    process = subprocess.run(
        [state_runner], input=data, text=True, capture_output=True, check=True)
    core_states = [int(line) for line in process.stdout.splitlines()]
    reference_states = [state for _, _, _, _, _, state in states]
    if core_states != reference_states:
        raise AssertionError(
            f"객체 상태 차이: 독립 코어 {core_states}, 원본 {reference_states}")
    if maximum_xy >= 0.001 or maximum_other >= 0.001:
        raise AssertionError(
            f"연속 경로 차이: 위치 {maximum_xy:.6f}m, 기타 {maximum_other:.6f}")
    print(f"20프레임 상태 일치, 경로 최대 평면 차이 {maximum_xy:.6f}m, "
          f"기타 성분 차이 {maximum_other:.6f}")


if __name__ == "__main__":
    main()
