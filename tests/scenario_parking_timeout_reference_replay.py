"""주차장 안에서 정차한 뒤 5초 초과 전환을 원본 실행에 대조합니다."""

import json
import subprocess
import sys
from pathlib import Path


def main():
    runner, map_file, position_file, reference_file = map(Path, sys.argv[1:])
    position = json.loads(position_file.read_text())
    reference = json.loads(reference_file.read_text())
    ego = position["ego"]
    ids = position["route_lanelet_ids"]
    payload = [ego["x"], ego["y"], ego["yaw_rad"], len(ids), *ids]
    process = subprocess.run(
        [str(runner), str(map_file), "timeout"],
        input=" ".join(map(str, payload)),
        text=True, capture_output=True, check=True)
    location, transitions = (
        list(map(int, line.split())) for line in process.stdout.splitlines())
    assert location == [1, 1, 1], location
    assert transitions == [1, 1, 2], transitions
    source = reference["synthetic_parking_timeout"]
    assert source["initial_scenario"] == "LaneDriving"
    assert source["completed_scenario"] == "Parking"
    assert 5.0 < source["transition_delay_s"] < 6.0
    print(f"원본 주차 전환 {source['transition_delay_s']:.2f}초, 독립 코어 5초 경계 일치")


if __name__ == "__main__":
    main()
