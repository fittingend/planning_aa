"""원본 재생 자료에서 스무더 시간 측정용 경로를 추출합니다."""

import argparse
import csv
import json
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(args.reference.read_text())
    path = data["trajectories"]["resampled"]
    ego = data["ego_pose"]
    closest = min(
        range(len(path)),
        key=lambda index: math.hypot(
            path[index]["x"] - ego["x"],
            path[index]["y"] - ego["y"]),
    )
    with args.output.open("w", newline="") as stream:
        writer = csv.writer(stream)
        for point in path[closest:]:
            writer.writerow((point["x"], point["y"], point["velocity"],
                             point["acceleration"]))
    print(f"입력 {len(path) - closest}점, 초기 속도 {abs(data['ego_velocity'])}m/s")


if __name__ == "__main__":
    main()
