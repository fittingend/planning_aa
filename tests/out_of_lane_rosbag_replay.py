"""K-City 경로와 실제 예측 객체로 차로 밖 주행 후보 수를 진단합니다."""

import argparse
import bisect
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import (
    decode_odometry, decode_predicted_objects, decode_traffic_light_groups,
    decode_trajectory, messages)

PREFIX = "/planning/scenario_planning/lane_driving/motion_planning/"
INPUT = PREFIX + "path_optimizer/trajectory"
OUTPUT = PREFIX + "motion_velocity_planner/trajectory"
OBJECTS = "/perception/object_recognition/objects"
ODOMETRY = "/localization/kinematic_state"
SIGNALS = "/perception/traffic_light_recognition/traffic_signals"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("map", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frames", type=int, default=10000)
    args = parser.parse_args()
    objects = list(messages(args.bag, OBJECTS))
    odometry = list(messages(args.bag, ODOMETRY))
    signals = list(messages(args.bag, SIGNALS))
    object_times = [time for time, _ in objects]
    odometry_times = [time for time, _ in odometry]
    signal_times = [time for time, _ in signals]
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
        if object_index < 0 or odometry_index < 0:
            continue
        predicted = decode_predicted_objects(objects[object_index][1])
        ego = decode_odometry(odometry[odometry_index][1])
        signal_index = bisect.bisect_right(signal_times, time) - 1
        red_ids = []
        if signal_index >= 0:
            observed = decode_traffic_light_groups(signals[signal_index][1])
            red_ids = [group["id"] for group in observed["groups"]
                       if any(color == 1 and status == 2
                              for color, _, status, _ in group["elements"])]
        frames.append((trajectory["points"], outputs[trajectory["stamp_ns"]],
                       ego, predicted["objects"], red_ids))
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory) / "입력.txt"
        target = Path(directory) / "출력.txt"
        with source.open("w") as stream:
            stream.write(f"{len(frames)}\n")
            for points, _, ego, detected, red_ids in frames:
                stream.write(f"{len(points)} {len(detected)} {len(red_ids)} "
                             f"{ego['x']} {ego['y']}\n")
                stream.write(" ".join(str(value) for value in red_ids) + "\n")
                for point in points:
                    stream.write(" ".join(str(point[name]) for name in (
                        "x", "y", "orientation_z", "orientation_w",
                        "velocity", "time_from_start")) + "\n")
                for item in detected:
                    pose = item["pose"]
                    paths = item["predicted_paths"]
                    labels = [label for label, _ in item["classification"]]
                    stream.write(f"{pose[0]} {pose[1]} {item['dimensions'][1]} "
                                 f"{int(7 in labels)} {len(paths)}\n")
                    for path in paths:
                        stream.write(f"{path['confidence']} {path['time_step_s']} "
                                     f"{item['dimensions'][0]} {item['dimensions'][1]} "
                                     f"{len(path['poses'])}\n")
                        for predicted_pose in path["poses"]:
                            stream.write(f"{predicted_pose[0]} {predicted_pose[1]} "
                                         f"{predicted_pose[5]} {predicted_pose[6]}\n")
        subprocess.run((str(args.runner), str(args.map), str(source),
                        str(target)), check=True)
        requested = [tuple(int(value) for value in line.split())
                     for line in target.read_text().splitlines()]
    assert len(requested) == len(frames)
    active = sum(candidate for candidate, _ in requested)
    red_frames = sum(bool(red_ids) for _, _, _, _, red_ids in frames)
    red_line_frames = sum(lines > 0 for _, lines in requested)
    changed = sum(
        len(points) != len(reference) or any(
            abs(point["x"] - recorded["x"]) > 1e-5 or
            abs(point["y"] - recorded["y"]) > 1e-5 or
            abs(point["velocity"] - recorded["velocity"]) > 1e-5
            for point, recorded in zip(points, reference))
        for points, reference, _, _, _ in frames)
    print(f"비교 {len(frames)}프레임, 적색 신호 입력 {red_frames}프레임, "
          f"주변 지도 정지선 연결 {red_line_frames}프레임, "
          f"독립 차로 밖 주행 후보 {active}프레임, "
          f"원본 동작 계획 출력 변경 {changed}프레임")


if __name__ == "__main__":
    main()
