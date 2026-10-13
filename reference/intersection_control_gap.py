"""교차로 원본 경로에서 독립 출력과 다른 제어점 위치를 찾습니다."""

import json
import math
import subprocess
import sys
from pathlib import Path


FIELDS = ("x", "y", "z", "qx", "qy", "qz", "qw", "velocity",
          "lateral_velocity", "heading_rate")


def cumulative_arcs(points):
    arcs = [0.0]
    for previous, current in zip(points, points[1:]):
        arcs.append(arcs[-1] + math.hypot(
            current["x"] - previous["x"],
            current["y"] - previous["y"]))
    return arcs


def main():
    if len(sys.argv) not in (4, 5) or (len(sys.argv) == 5 and sys.argv[4] != "--verify"):
        raise SystemExit("사용법: 실행기 이진지도 원본기준.json [--verify]")
    runner, map_file, fixture = map(Path, sys.argv[1:4])
    reference = json.loads(fixture.read_text())
    ego = reference["ego"]
    payload = [ego["x"], ego["y"], ego["velocity_mps"],
               ego["acceleration_mps2"], len(reference["input"])]
    for point, ids in zip(reference["input"], reference["input_lane_ids"]):
        payload.extend(point[field] for field in FIELDS)
        payload.extend((len(ids), *ids))
    payload.extend((0, 0))
    process = subprocess.run(
        (str(runner), str(map_file), "intersection_collision"),
        input=" ".join(map(str, payload)), text=True,
        capture_output=True, check=True)
    lines = process.stdout.splitlines()
    observed = [dict(zip(FIELDS, map(float, line.split())))
                for line in lines[4:]]
    expected = reference["output"]
    assert len(observed) == int(lines[3])
    observed_arcs = cumulative_arcs(observed)
    expected_arcs = cumulative_arcs(expected)
    print(f"원본 {len(expected)}점, 독립 출력 {len(observed)}점")
    if len(sys.argv) == 5:
        assert len(observed) == len(expected), "교차로 출력점 수가 원본과 다릅니다"
        reference_index = min(
            range(len(expected_arcs)),
            key=lambda index: abs(expected_arcs[index] - 58.9))
        reference_point = expected[reference_index]
        assert abs(expected_arcs[reference_index] - 58.9) < 0.01
        distance = min(math.hypot(
            reference_point["x"] - point["x"],
            reference_point["y"] - point["y"]) for point in observed)
        assert distance < 0.005, "주목 차로 중심선 기준점이 원본과 다릅니다"
    for index, point in enumerate(expected):
        distance, nearest = min(
            (math.hypot(point["x"] - candidate["x"],
                        point["y"] - candidate["y"]), candidate_index)
            for candidate_index, candidate in enumerate(observed))
        if distance > 0.05:
            print(
                f"원본 {index}번 호길이 {expected_arcs[index]:.3f}m, "
                f"독립 최근접 호길이 {observed_arcs[nearest]:.3f}m, "
                f"위치 차이 {distance:.3f}m")


if __name__ == "__main__":
    main()
