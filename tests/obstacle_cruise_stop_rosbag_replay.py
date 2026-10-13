"""기록된 정지점 위치를 역산해 C++ 정지점 삽입 결과를 대조합니다."""

import argparse
import math
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import decode_trajectory, messages

PREFIX = "/planning/scenario_planning/lane_driving/"
INPUT = PREFIX + "motion_planning/motion_velocity_planner/trajectory"
OUTPUT = PREFIX + "trajectory"


def stop_arc(points, target):
    arc = 0.0
    best = (math.inf, 0.0)
    for front, back in zip(points, points[1:]):
        dx = back["x"] - front["x"]
        dy = back["y"] - front["y"]
        length = math.hypot(dx, dy)
        if length < 1e-9:
            continue
        ratio = max(0.0, min(1.0,
            ((target["x"] - front["x"]) * dx +
             (target["y"] - front["y"]) * dy) / length ** 2))
        error = math.hypot(front["x"] + ratio * dx - target["x"],
                           front["y"] + ratio * dy - target["y"])
        if error < best[0]:
            best = error, arc + ratio * length
        arc += length
    return best


def inserted_index(source, target):
    for i in range(len(source)):
        if math.hypot(source[i]["x"] - target[i]["x"],
                      source[i]["y"] - target[i]["y"]) > 1e-4:
            return i
    return len(source)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    args = parser.parse_args()
    inputs = {message["stamp_ns"]: message["points"]
              for _, payload in messages(args.bag, INPUT)
              if (message := decode_trajectory(payload))}
    frames = []
    other_insertions = 0
    for _, payload in messages(args.bag, OUTPUT):
        message = decode_trajectory(payload)
        source = inputs.get(message["stamp_ns"])
        target = message["points"]
        if source is None or len(target) != len(source) + 1:
            continue
        index = inserted_index(source, target)
        if index >= len(target) or abs(target[index]["velocity"]) >= 1e-4:
            other_insertions += 1
            continue
        stop = target[index]
        error, arc = stop_arc(source, stop)
        if error > 0.05:
            continue
        frames.append((source, target, arc))
    with tempfile.TemporaryDirectory() as directory:
        input_path = Path(directory) / "입력.txt"
        output_path = Path(directory) / "출력.txt"
        with input_path.open("w") as stream:
            stream.write(f"{len(frames)}\n")
            for source, _, arc in frames:
                stream.write(f"{len(source)} {arc:.17g}\n")
                for point in source:
                    stream.write(" ".join(str(point[key]) for key in (
                        "x", "y", "z", "orientation_x", "orientation_y",
                        "orientation_z", "orientation_w", "velocity")) + "\n")
        subprocess.run([str(args.runner), str(input_path), str(output_path)], check=True)
        position_error = speed_error = 0.0
        count_difference = 0
        with output_path.open() as stream:
            for _, expected, _ in frames:
                count = int(stream.readline())
                actual = [tuple(float(v) for v in stream.readline().split())
                          for _ in range(count)]
                if count != len(expected):
                    count_difference += 1
                for ours, theirs in zip(actual, expected):
                    position_error = max(position_error, math.hypot(
                        ours[0] - theirs["x"], ours[1] - theirs["y"]))
                    speed_error = max(speed_error, abs(
                        ours[2] - theirs["velocity"]))
    print(f"새 정지점 삽입 {len(frames)}프레임, 다른 삽입 제외 "
          f"{other_insertions}프레임, 점 수 차이 {count_difference}, "
          f"최대 위치 차이 {position_error:.6f}m, "
          f"최대 속도 차이 {speed_error:.6f}m/s")


if __name__ == "__main__":
    main()
