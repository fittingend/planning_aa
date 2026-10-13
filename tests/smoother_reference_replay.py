"""원본 decision 이미지의 기준 궤적과 독립 계산 코어를 비교한다."""

import csv
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path


def read_result(path):
    result = {"merged": [], "internal": [], "output": []}
    with path.open(newline="") as stream:
        for stage, x, y, velocity, acceleration in csv.reader(stream):
            if stage not in result:
                continue
            result[stage].append(
                (float(x), float(y), float(velocity), float(acceleration))
            )
    return result


def project_motion(trajectory, pose):
    best = None
    for start, end in zip(trajectory, trajectory[1:]):
        dx = end["x"] - start["x"]
        dy = end["y"] - start["y"]
        length_squared = dx * dx + dy * dy
        if length_squared <= 1.0e-12:
            continue
        ratio = max(0.0, min(1.0, (
            (pose["x"] - start["x"]) * dx +
            (pose["y"] - start["y"]) * dy
        ) / length_squared))
        x = start["x"] + ratio * dx
        y = start["y"] + ratio * dy
        distance_squared = (x - pose["x"]) ** 2 + (y - pose["y"]) ** 2
        if best is None or distance_squared < best[0]:
            best = (distance_squared, start, end, ratio)
    _, start, end, ratio = best
    return (
        start["velocity"] + (end["velocity"] - start["velocity"]) * ratio,
        start["acceleration"] +
        (end["acceleration"] - start["acceleration"]) * ratio,
    )


def main():
    runner = Path(sys.argv[1])
    reference = json.loads(Path(sys.argv[2]).read_text())
    baseline = reference["trajectories"]
    closest = 0
    if reference.get("ego_pose") is not None:
        pose = reference["ego_pose"]
        closest = min(
            range(len(baseline["resampled"])),
            key=lambda index: math.hypot(
                baseline["resampled"][index]["x"] - pose["x"],
                baseline["resampled"][index]["y"] - pose["y"],
            ),
        )
    initial_velocity = abs(reference["ego_velocity"])
    initial_acceleration = (
        0.0 if reference.get("bag_timestamp") is not None
        else reference["ego_acceleration"]
    )
    if reference.get("first_trajectories"):
        pose = reference["ego_pose"]
        previous_velocity, previous_acceleration = project_motion(
            reference["first_trajectories"]["output"], pose
        )
        initial_velocity = abs(previous_velocity)
        initial_acceleration = previous_acceleration
    with tempfile.TemporaryDirectory() as directory:
        input_path = Path(directory) / "input.csv"
        output_path = Path(directory) / "output.csv"
        previous_path = Path(directory) / "previous.csv"
        with input_path.open("w", newline="") as stream:
            writer = csv.writer(stream)
            for point in baseline["resampled"]:
                writer.writerow(
                    (
                        point["x"], point["y"], point["velocity"], point["acceleration"],
                        point.get("z", 0.0), point.get("orientation_x", 0.0),
                        point.get("orientation_y", 0.0), point.get("orientation_z", 0.0),
                        point.get("orientation_w", 1.0),
                    )
                )
        ego = reference.get("ego_pose", baseline["resampled"][0])
        if reference.get("first_trajectories"):
            with previous_path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                for point in reference["first_trajectories"]["output"]:
                    writer.writerow(
                        (
                            point["x"], point["y"], point["velocity"],
                            point["acceleration"], point.get("z", 0.0),
                            point.get("orientation_x", 0.0),
                            point.get("orientation_y", 0.0),
                            point.get("orientation_z", 0.0),
                            point.get("orientation_w", 1.0),
                        )
                    )
        subprocess.run(
            (
                str(runner), str(input_path), str(output_path),
                str(reference["ego_velocity"]), str(initial_acceleration),
                str(closest), str(ego["x"]), str(ego["y"]), str(ego.get("z", 0.0)),
                str(ego.get("orientation_x", 0.0)), str(ego.get("orientation_y", 0.0)),
                str(ego.get("orientation_z", 0.0)), str(ego.get("orientation_w", 1.0)),
                str(initial_velocity), str(initial_acceleration),
                str(previous_path) if reference.get("first_trajectories") else "",
            ),
            check=True,
        )
        actual = read_result(output_path)

    if reference.get("bag_timestamp") is not None:
        print(
            "실측 최종 출력 점 수:", len(actual["output"]),
            "원본 점 수:", len(baseline["output"]),
        )

    for stage in ("merged", "output"):
        expected = baseline[stage][closest:] if stage == "merged" else baseline[stage]
        observed = actual[stage]
        assert len(observed) == len(expected), (stage, len(observed), len(expected))
        max_difference = {field: 0.0 for field in ("x", "y", "velocity", "acceleration")}
        max_index = {field: 0 for field in max_difference}
        for index, (got, want) in enumerate(zip(observed, expected)):
            for value, field in zip(got, ("x", "y", "velocity", "acceleration")):
                difference = abs(value - want[field])
                if difference > max_difference[field]:
                    max_difference[field] = difference
                    max_index[field] = index
        print(f"{stage}: {len(observed)}점, 최대 차이 {max_difference}, 위치 {max_index}")
        for field, difference in max_difference.items():
            assert difference < 1.0e-3, (stage, field, difference)


if __name__ == "__main__":
    main()
