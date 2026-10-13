"""원본 교차로 예측 차량의 안전·충돌 결정을 비교합니다."""

import json
import subprocess
import sys
from pathlib import Path


FIELDS = ("x", "y", "z", "qx", "qy", "qz", "qw", "velocity",
          "lateral_velocity", "heading_rate")


def run_case(runner, map_file, fixture):
    data = json.loads(fixture.read_text())
    entry = next(index for index, ids in enumerate(data["input_lane_ids"])
                 if 25001 in ids)
    payload = [data["ego"]["x"], data["ego"]["y"],
               len(data["input"]), entry]
    for point in data["input"]:
        payload.extend(point[field] for field in FIELDS)
    car = data.get("synthetic_car")
    payload.append(int(car is not None))
    if car:
        payload.extend((car["x"], car["y"], car["yaw_rad"],
                        car["speed_mps"], car["prediction_duration_s"],
                        car.get("label", 1), car.get("confidence", 1.0),
                        car.get("path_y_shift", 0.0),
                        car.get("second_path_confidence") or -1.0))
    process = subprocess.run([str(runner), str(map_file)],
                             input=" ".join(map(str, payload)),
                             text=True, capture_output=True, check=True)
    collision, time, arc = map(float, process.stdout.split())
    decision = data["intersection_decisions"][-1].splitlines()[1]
    expected = decision == "NonOccludedCollisionStop:"
    print(f"{fixture.name}: 원본 {decision}, 코어 충돌 {bool(collision)}, "
          f"시각 {time:.2f}초, 경로 {arc:.2f}m")
    assert bool(collision) == expected


def main():
    runner, map_file, *fixtures = map(Path, sys.argv[1:])
    for fixture in fixtures:
        run_case(runner, map_file, fixture)


if __name__ == "__main__":
    main()
