"""K-City 예측 객체를 동적 장애물 정지 코어에 연속 재생합니다."""

import argparse
import bisect
import math
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import (
    decode_acceleration, decode_odometry, decode_predicted_objects,
    decode_trajectory, messages)

PREFIX = "/planning/scenario_planning/lane_driving/motion_planning/"
INPUT = PREFIX + "path_optimizer/trajectory"
OUTPUT = PREFIX + "motion_velocity_planner/trajectory"
OBJECTS = "/perception/object_recognition/objects"
ODOMETRY = "/localization/kinematic_state"
ACCELERATION = "/localization/acceleration"


def yaw(pose):
    x, y, z, w = pose[3:7]
    return math.atan2(2 * (w * z + x * y),
                      1 - 2 * (y * y + z * z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frames", type=int, default=10000)
    args = parser.parse_args()
    objects = list(messages(args.bag, OBJECTS))
    odometry = list(messages(args.bag, ODOMETRY))
    acceleration = list(messages(args.bag, ACCELERATION))
    object_times = [time for time, _ in objects]
    odometry_times = [time for time, _ in odometry]
    acceleration_times = [time for time, _ in acceleration]
    outputs = {
        item["stamp_ns"]: item["points"]
        for _, payload in messages(args.bag, OUTPUT)
        if (item := decode_trajectory(payload))}
    frames = []
    for time, payload in messages(args.bag, INPUT):
        if len(frames) >= args.frames:
            break
        trajectory = decode_trajectory(payload)
        if len(trajectory["points"]) < 2 or trajectory["stamp_ns"] not in outputs:
            continue
        object_index = bisect.bisect_right(object_times, time) - 1
        odometry_index = bisect.bisect_right(odometry_times, time) - 1
        acceleration_index = bisect.bisect_right(acceleration_times, time) - 1
        if object_index < 0 or odometry_index < 0:
            continue
        predicted = decode_predicted_objects(objects[object_index][1])
        ego = decode_odometry(odometry[odometry_index][1])
        ego_acceleration = (
            decode_acceleration(acceleration[acceleration_index][1])
            if acceleration_index >= 0 else {"acceleration_mps2": 0.0})
        frames.append((time, trajectory["points"],
                       outputs[trajectory["stamp_ns"]], ego,
                       predicted["objects"], ego_acceleration))
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory) / "입력.txt"
        target = Path(directory) / "출력.txt"
        with source.open("w") as stream:
            stream.write(f"{len(frames)}\n")
            for time, points, _, ego, observed, ego_acceleration in frames:
                stream.write(f"{len(points)} {len(observed)} {time * 1e-9} "
                             f"{ego['x']} {ego['y']} "
                             f"{ego['orientation_z']} {ego['orientation_w']} "
                             f"{ego['velocity_mps']} "
                             f"{ego_acceleration['acceleration_mps2']}\n")
                for point in points:
                    stream.write(" ".join(str(point[name]) for name in (
                        "x", "y", "orientation_z", "orientation_w",
                        "velocity")) + "\n")
                for item in observed:
                    pose = item["pose"]
                    label = (item["classification"][0][0]
                             if item["classification"] else 0)
                    stream.write(f"{item['uuid']} {label} {pose[0]} {pose[1]} "
                                 f"{yaw(pose)} {item['twist'][0]} "
                                 f"{item['dimensions'][0]} "
                                 f"{item['dimensions'][1]}\n")
        subprocess.run((str(args.runner), str(source), str(target)), check=True)
        rows = [[int(value) for value in line.split()]
                for line in target.read_text().splitlines()]
    assert len(rows) == len(frames)
    totals = [sum(row[i] for row in rows) for i in range(4)]
    active = [sum(row[i] > 0 for row in rows) for i in range(1, 4)]
    missing_acceleration = sum(
        not acceleration_times or time < acceleration_times[0]
        for time, *_ in frames)
    changed = sum(
        len(points) != len(reference) or any(
            abs(point["x"] - recorded["x"]) > 1e-5 or
            abs(point["y"] - recorded["y"]) > 1e-5 or
            abs(point["velocity"] - recorded["velocity"]) > 1e-5
            for point, recorded in zip(points, reference))
        for _, points, reference, _, _, _ in frames)
    print(f"{args.bag}: 비교 {len(frames)}프레임, 객체 {totals[0]}, "
          f"필터 통과 {totals[1]}({active[0]}프레임), "
          f"충돌 교점 {totals[2]}({active[1]}프레임), "
          f"독립 정지 요청 {totals[3]}({active[2]}프레임), "
          f"원본 출력 변경 {changed}프레임, "
          f"초기 가속도 대체 {missing_acceleration}프레임")


if __name__ == "__main__":
    main()
