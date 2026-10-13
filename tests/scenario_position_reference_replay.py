"""기록된 경로와 이진 지도에서 시나리오 선택 위치 입력을 계산합니다."""

import json
import subprocess
import sys
from pathlib import Path


def main():
    runner, map_file, reference_file = map(Path, sys.argv[1:4])
    reference = json.loads(reference_file.read_text())
    ego = reference["ego"]
    goal = reference["goal"]
    ids = reference["route_lanelet_ids"]
    payload = [ego["x"], ego["y"], ego["yaw_rad"],
               goal["x"], goal["y"], len(ids), *ids]
    run = subprocess.run(
        [str(runner), str(map_file)], input=" ".join(map(str, payload)),
        text=True, capture_output=True, check=True)
    fields = list(map(int, run.stdout.split()))
    assert len(fields) == 4
    print(f"원본 시나리오 {reference['scenario']}, "
          f"차로·목적지·주차장·경로 정렬 판정 {fields}")
    assert reference["scenario"] == "LaneDriving"
    assert fields == [1, 1, 0, 1]


if __name__ == "__main__":
    main()
