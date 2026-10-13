"""원본 스무더의 여러 연속 프레임과 독립 계산 결과를 비교합니다."""

import csv
import gzip
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

from smoother_reference_replay import project_motion, read_result


def write_points(path, points):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        for point in points:
            writer.writerow((
                point["x"], point["y"], point["velocity"],
                point["acceleration"], point.get("z", 0.0),
                point.get("orientation_x", 0.0),
                point.get("orientation_y", 0.0),
                point.get("orientation_z", 0.0),
                point.get("orientation_w", 1.0),
            ))


def compare(observed, expected, frame, stage):
    assert len(observed) == len(expected), (
        frame, stage, len(observed), len(expected),
        observed[18:26], [(p["x"], p["velocity"]) for p in expected[18:26]])
    largest = 0.0
    largest_field = ""
    largest_index = 0
    for index, (actual, reference) in enumerate(zip(observed, expected)):
        for field, value in zip(
                ("x", "y", "velocity", "acceleration"), actual):
            difference = abs(value - reference[field])
            if difference > largest:
                largest = difference
                largest_field = field
                largest_index = index
    assert largest < 0.001, (
        frame, stage, largest_field, largest, largest_index,
        observed[largest_index], expected[largest_index])
    return largest


def main():
    runner, fixture = map(Path, sys.argv[1:])
    if fixture.suffix == ".gz":
        with gzip.open(fixture, "rt") as stream:
            records = json.load(stream)["sequence_records"]
    else:
        records = json.loads(fixture.read_text())["sequence_records"]
    assert len(records) >= 3
    with tempfile.TemporaryDirectory() as directory:
        directory = Path(directory)
        previous_internal = None
        for index, record in enumerate(records):
            current = record["trajectories"]
            points = current["resampled"]
            pose = {"x": record["ego_x"], "y": record["ego_y"]}
            closest = min(
                range(len(points)),
                key=lambda i: math.hypot(
                    points[i]["x"] - pose["x"],
                    points[i]["y"] - pose["y"]),
            )
            if index == 0:
                initial_velocity = abs(record["ego_velocity"])
                initial_acceleration = 0.0
                previous_file = ""
                keep_closest = False
            else:
                previous = previous_internal
                initial_velocity, initial_acceleration = project_motion(
                    previous, pose)
                initial_velocity = abs(initial_velocity)
                keep_closest = (
                    abs(abs(record["ego_velocity"]) - initial_velocity) > 5.53)
                if keep_closest:
                    initial_velocity = abs(record["ego_velocity"])
                previous_file = str(directory / "previous.csv")
                write_points(Path(previous_file), previous)
            input_file = directory / "input.csv"
            output_file = directory / "output.csv"
            write_points(input_file, points)
            subprocess.run((
                str(runner), str(input_file), str(output_file),
                str(record["ego_velocity"]), str(initial_acceleration),
                str(closest), str(pose["x"]), str(pose["y"]),
                "0", "0", "0", "0", "1",
                str(initial_velocity), str(initial_acceleration), previous_file,
                "1" if keep_closest else "0",
            ), check=True)
            actual = read_result(output_file)
            final = compare(actual["output"], current["output"], index, "output")
            previous_internal = [
                {"x": x, "y": y, "velocity": velocity,
                 "acceleration": acceleration}
                for x, y, velocity, acceleration in actual["internal"]
            ]
            print(f"{index}프레임: 최종 출력 일치, 최대 차이 {final:.9f}")


if __name__ == "__main__":
    main()
