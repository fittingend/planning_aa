"""K-City 지도 가드레일 속도 제한 요청과 원본 동작 계획 출력을 비교합니다."""

import argparse
import bisect
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import decode_odometry, decode_trajectory, messages

PREFIX = "/planning/scenario_planning/lane_driving/motion_planning/"
INPUT = PREFIX + "path_optimizer/trajectory"
OUTPUT = PREFIX + "motion_velocity_planner/trajectory"
ODOMETRY = "/localization/kinematic_state"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("map", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frames", type=int, default=10000)
    parser.add_argument("--bicycle", action="store_true",
                        help="자전거 투영 모형의 수치 안정성을 진단합니다")
    args = parser.parse_args()
    odometry = list(messages(args.bag, ODOMETRY))
    odometry_times = [time for time, _ in odometry]
    recorded_outputs = {
        item["stamp_ns"]: item["points"]
        for _, payload in messages(args.bag, OUTPUT)
        if (item := decode_trajectory(payload))
    }
    frames = []
    for time, payload in messages(args.bag, INPUT):
        if len(frames) >= args.frames:
            break
        path = decode_trajectory(payload)
        if len(path["points"]) < 2 or path["stamp_ns"] not in recorded_outputs:
            continue
        index = bisect.bisect_right(odometry_times, time) - 1
        if index < 0:
            continue
        ego = decode_odometry(odometry[index][1])
        ego_index = min(range(len(path["points"])), key=lambda i:
                        (path["points"][i]["x"] - ego["x"]) ** 2 +
                        (path["points"][i]["y"] - ego["y"]) ** 2)
        frames.append((path["points"], recorded_outputs[path["stamp_ns"]],
                       ego_index, ego["velocity_mps"]))
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory) / "입력.txt"
        target = Path(directory) / "출력.txt"
        with source.open("w") as stream:
            stream.write(f"{len(frames)}\n")
            for points, _, ego_index, ego_velocity in frames:
                stream.write(f"{len(points)} {ego_index} {ego_velocity}\n")
                for point in points:
                    stream.write(" ".join(str(point[name]) for name in (
                        "x", "y", "orientation_z", "orientation_w",
                        "velocity", "time_from_start",
                        "front_wheel_angle")) + "\n")
        command = [str(args.runner), str(args.map), str(source), str(target)]
        if args.bicycle:
            command.append("--bicycle")
        subprocess.run(command, check=True)
        with target.open() as stream:
            requested = [int(stream.readline()) for _ in frames]
    changed = sum(any(abs(a["velocity"] - b["velocity"]) > 1e-5 or
                      abs(a["x"] - b["x"]) > 1e-5 or
                      abs(a["y"] - b["y"]) > 1e-5
                      for a, b in zip(source, output)) or len(source) != len(output)
                  for source, output, _, _ in frames)
    active = sum(count > 0 for count in requested)
    if args.bicycle:
        print(f"자전거 모형 수치 재생 {len(frames)}프레임, "
              f"지도 정적 장애물 감속 요청 {active}프레임")
    else:
        print(f"비교 {len(frames)}프레임, 지도 정적 장애물 감속 요청 {active}프레임, "
              f"원본 동작 계획 출력 변경 {changed}프레임")


if __name__ == "__main__":
    main()
