"""주차 궤적 한 점이 3초 넘게 유지될 때 차로 복귀를 원본과 비교합니다."""

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
        [str(runner), str(map_file), "empty_timeout"],
        input=" ".join(map(str, payload)),
        text=True, capture_output=True, check=True)
    actual = list(map(int, process.stdout.split()))
    assert actual == [2, 2, 1, 1], actual
    source = reference["synthetic_parking_empty_timeout"]
    assert source["initial_scenario"] == "Parking"
    assert source["completed_scenario"] == "LaneDriving"
    assert source["parking_path_published"] == 1
    assert 2.5 < source["transition_delay_s"] < 3.5
    print("빈 주차 궤적 유지 후 차로 복귀가 원본과 일치")


if __name__ == "__main__":
    main()
