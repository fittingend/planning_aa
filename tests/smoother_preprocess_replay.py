"""원본 노드의 횡가속도·조향 제한 디버그 궤적을 비교한다."""

import csv
import json
import subprocess
import sys
import tempfile
from pathlib import Path

from smoother_reference_replay import project_motion


def main():
    reference = json.loads(Path(sys.argv[2]).read_text())
    baseline = reference["trajectories"]
    with tempfile.TemporaryDirectory() as directory:
        input_path = Path(directory) / "input.csv"
        output_path = Path(directory) / "output.csv"
        with input_path.open("w", newline="") as stream:
            writer = csv.writer(stream)
            for point in baseline["raw"]:
                writer.writerow(
                    (
                        point["x"], point["y"], point["velocity"], point["acceleration"],
                        point.get("z", 0.0), point.get("orientation_x", 0.0),
                        point.get("orientation_y", 0.0), point.get("orientation_z", 0.0),
                        point.get("orientation_w", 1.0),
                    )
                )
        ego = reference.get("ego_pose", baseline["limited"][0])
        initial_velocity = abs(reference["ego_velocity"])
        initial_acceleration = 0.0
        extra = ()
        if reference.get("first_trajectories"):
            previous_velocity, previous_acceleration = project_motion(
                reference["first_trajectories"]["output"], ego
            )
            initial_velocity = abs(previous_velocity)
            initial_acceleration = previous_acceleration
            if reference["case"].endswith("_external_limit"):
                extra = (str(previous_velocity), str(previous_acceleration), "2.0")
        subprocess.run(
            (str(sys.argv[1]), str(input_path), str(output_path),
             str(initial_velocity), str(initial_acceleration),
             str(ego["x"]), str(ego["y"]), str(ego.get("z", 0.0)),
             str(ego.get("orientation_x", 0.0)), str(ego.get("orientation_y", 0.0)),
             str(ego.get("orientation_z", 0.0)), str(ego.get("orientation_w", 1.0)),
             *extra),
            check=True,
        )
        actual = {"limited": [], "lateral": [], "steering": [], "resampled": []}
        with output_path.open(newline="") as stream:
            for stage, x, y, velocity, acceleration in csv.reader(stream):
                actual[stage].append(
                    (float(x), float(y), float(velocity), float(acceleration))
                )

    for stage in ("limited", "lateral", "steering", "resampled"):
        baseline_stage = stage
        expected = baseline[baseline_stage]
        observed = actual[stage]
        assert len(observed) == len(expected), (stage, len(observed), len(expected))
        maximum = {field: 0.0 for field in ("x", "y", "velocity", "acceleration")}
        maximum_index = {field: 0 for field in maximum}
        for index, (got, want) in enumerate(zip(observed, expected)):
            for value, field in zip(got, maximum):
                difference = abs(value - want[field])
                if difference > maximum[field]:
                    maximum[field] = difference
                    maximum_index[field] = index
        print(f"{stage}: {len(observed)}점, 최대 차이 {maximum}, 위치 {maximum_index}")
        for field, difference in maximum.items():
            assert difference < 1.0e-3, (stage, field, difference)


if __name__ == "__main__":
    main()
