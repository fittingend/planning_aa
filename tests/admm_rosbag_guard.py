"""실측 경로에서 느슨한 상대 잔차만으로 ADMM 해를 승인하지 않는지 검사합니다."""

import csv
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path


def run_case(benchmark, reference, penalty, expected_success):
    data = json.loads(reference.read_text())
    path = data["trajectories"]["resampled"]
    ego = data["ego_pose"]
    closest = min(range(len(path)), key=lambda index: math.hypot(
        path[index]["x"] - ego["x"], path[index]["y"] - ego["y"]))
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory) / "input.csv"
        output = Path(directory) / "output.csv"
        with source.open("w", newline="") as stream:
            writer = csv.writer(stream)
            for point in path[closest:]:
                writer.writerow((point["x"], point["y"],
                                 point["velocity"], point["acceleration"]))
        process = subprocess.run((str(benchmark), "1", str(source),
                                  str(abs(data["ego_velocity"])), str(penalty),
                                  str(output)), capture_output=True, text=True)
        if (process.returncode == 0) != expected_success:
            raise AssertionError(
                f"{reference.name}, 벌점 {penalty}: 예상한 수렴 결과와 다릅니다\n"
                f"{process.stdout}{process.stderr}")
        if expected_success:
            assert output.exists() and output.stat().st_size > 0
        else:
            assert "풀이 실패" in process.stderr


def main():
    benchmark = Path(sys.argv[1])
    reference_dir = Path(sys.argv[2])
    run_case(benchmark, reference_dir / "smoother_reference_pedestrian.json",
             1.0, True)
    for name in ("stopline", "cutinout"):
        run_case(benchmark, reference_dir / f"smoother_reference_{name}.json",
                 30000.0, False)
    print("실측 ADMM 수렴 보호 검사 통과")


if __name__ == "__main__":
    main()
