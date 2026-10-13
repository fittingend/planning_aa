"""미확인 격자가 사라진 뒤 교차로 정지 유지 시간을 원본 연속 출력에 비교합니다."""

import json
import math
import subprocess
import sys
from pathlib import Path


FIELDS = ("x", "y", "z", "qx", "qy", "qz", "qw", "velocity",
          "lateral_velocity", "heading_rate")


def main():
    runner, map_file, fixture = map(Path, sys.argv[1:])
    data = json.loads(fixture.read_text())
    ego = data["ego"]
    payload = [ego["x"], ego["y"], ego["velocity_mps"],
               ego["acceleration_mps2"], len(data["input"])]
    for point, ids in zip(data["input"], data["input_lane_ids"]):
        payload.extend(point[field] for field in FIELDS)
        payload.extend((len(ids), *ids))
    signal = data["intersection_signal"]
    payload.extend((0, 1, signal["regulatory_id"], 2))
    grid = data["intersection_occupancy"]
    yaw = 2.0 * math.atan2(grid["origin_qz"], grid["origin_qw"])
    frames = data["intersection_frames"]
    payload.extend((grid["width"], grid["height"], grid["resolution"],
                    grid["origin_x"], grid["origin_y"], yaw, len(frames)))
    for index, frame in enumerate(frames):
        previous = frames[max(0, index - 1)]
        payload.extend((frame["time_s"], int(previous["unknown"]),
                        {"red": 1, "green": 2, "yellow": 3}.get(
                            previous.get("signal_color", "green"), 0)))
    process = subprocess.run(
        [str(runner), str(map_file), "intersection_occlusion_sequence"],
        input=" ".join(map(str, payload)), text=True,
        capture_output=True, check=True)
    actual = [tuple(map(int, line.split()))
              for line in process.stdout.splitlines()]
    expected = []
    for frame in frames:
        decision = frame["decision"].splitlines()[1]
        assert decision in ("FirstWaitBeforeOcclusion:", "Safe:")
        zero = next(index for index, point in enumerate(frame["output"])
                    if point["velocity"] == 0)
        expected.append((int(decision == "FirstWaitBeforeOcclusion:"), zero))
    assert actual == expected, list(enumerate(zip(actual, expected)))
    print(f"교차로 가림·신호 전환 연속 {len(frames)}프레임 일치")


if __name__ == "__main__":
    main()
