"""가림 확인 정지선에서 정적 가림 해제 시간을 연속 프레임으로 확인합니다."""

import json
import math
import subprocess
import sys
from pathlib import Path


FIELDS = ("x", "y", "z", "qx", "qy", "qz", "qw", "velocity",
          "lateral_velocity", "heading_rate")


def projected_arc(points, location):
    best_distance = float("inf")
    best_arc = 0.0
    arc = 0.0
    for first, second in zip(points, points[1:]):
        dx = second["x"] - first["x"]
        dy = second["y"] - first["y"]
        length2 = dx * dx + dy * dy
        if length2 == 0:
            continue
        ratio = max(0.0, min(1.0,
            ((location["x"] - first["x"]) * dx +
             (location["y"] - first["y"]) * dy) / length2))
        distance = math.hypot(location["x"] - first["x"] - ratio * dx,
                              location["y"] - first["y"] - ratio * dy)
        length = math.sqrt(length2)
        if distance < best_distance:
            best_distance = distance
            best_arc = arc + ratio * length
        arc += length
    return best_arc


def main():
    runner, map_file, fixture = map(Path, sys.argv[1:])
    data = json.loads(fixture.read_text())
    stop = next(point for point in data["output"]
                if point["velocity"] == 0.0)
    peeking_arc = projected_arc(data["input"], stop)
    payload = [stop["x"], stop["y"], 0.0, 0.0, len(data["input"])]
    for point, ids in zip(data["input"], data["input_lane_ids"]):
        payload.extend(point[field] for field in FIELDS)
        payload.extend((len(ids), *ids))
    signal = data["intersection_signal"]
    payload.extend((0, 1, signal["regulatory_id"], 2))
    grid = data["intersection_occupancy"]
    yaw = 2.0 * math.atan2(grid["origin_qz"], grid["origin_qw"])
    frames = (0.0, 0.1, 0.3, 0.6, 0.71, 0.8)
    payload.extend((grid["width"], grid["height"], grid["resolution"],
                    grid["origin_x"], grid["origin_y"], yaw, len(frames),
                    peeking_arc - 0.2, peeking_arc))
    for now in frames:
        payload.extend((now, 1, 2))
    process = subprocess.run(
        [str(runner), str(map_file), "intersection_occlusion_release_sequence"],
        input=" ".join(map(str, payload)), text=True,
        capture_output=True, check=True)
    actual = [tuple(map(int, line.split()))
              for line in process.stdout.splitlines()]
    assert [result[0] for result in actual] == [1, 1, 1, 1, 0, 0], actual
    assert len({result[1] for result in actual[:4]}) == 1, actual
    assert actual[4][1] > actual[3][1], actual
    assert actual[4][1] == actual[5][1], actual
    print("교차로 정적 가림 정지 후 0.5초 초과 해제 6프레임 일치")


if __name__ == "__main__":
    main()
