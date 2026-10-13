"""합성 주차장 지도에서 원본 시나리오 선택 전환을 독립 코어에 비교합니다."""

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
        [str(runner), str(map_file)],
        input=" ".join(map(str, payload)),
        text=True, capture_output=True, check=True)
    location, transitions = (
        list(map(int, line.split())) for line in process.stdout.splitlines())
    assert location == [1, 0, 1, 1], location
    assert transitions == [2, 1, 1, 1], transitions
    source = reference["synthetic_parking"]
    assert source["initial_scenario"] == "Parking"
    assert source["completed_scenario"] == "LaneDriving"
    assert source["parking_path_published"] == 1
    assert reference["scenario"] == "LaneDriving"
    assert reference["output_points"] == reference["input_points"]
    print("주차 선택·주차 완료 후 차로 복귀가 원본과 일치")


if __name__ == "__main__":
    main()
