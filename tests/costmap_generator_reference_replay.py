"""원본 costmap generator의 합성 주차 활성 격자와 독립 지도를 셀별 비교합니다."""

import argparse
import gzip
import json
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import decode_lanelet_map, messages


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("reference", type=Path)
    args = parser.parse_args()
    with gzip.open(args.reference, "rt") as stream:
        reference = json.load(stream)
    map_messages = messages(args.bag, "/map/vector_map")
    try:
        _, payload = next(map_messages)
    except StopIteration as error:
        raise ValueError("기록에 지도 메시지가 없습니다") from error
    original = reference["output"]
    ego = reference["input"]
    with tempfile.TemporaryDirectory() as directory:
        map_path = Path(directory) / "map.bin"
        map_path.write_bytes(decode_lanelet_map(payload))
        command = [str(args.runner), str(map_path), str(ego["ego_x"]),
                   str(ego["ego_y"]), str(ego["ego_z"])]
        if isinstance(ego["objects"], dict):
            obj = ego["objects"]
            command.extend(str(obj[field]) for field in
                           ("x", "y", "length_m", "width_m", "probability"))
        if isinstance(ego["points"], dict):
            point = ego["points"]
            command.extend(str(point[field]) for field in ("x", "y", "z"))
            if point.get("count") == 2:
                command.append("--second-point")
                command.extend(str(point[field]) for field in
                               ("x2", "y2", "z2"))
        if isinstance(ego["objects"], dict):
            shape = ego["objects"].get("shape")
            yaw = ego["objects"].get("yaw_rad", 0.0)
            if shape in ("CYLINDER", "POLYGON") or yaw:
                shape_code = {"CYLINDER": 1, "POLYGON": 2}.get(shape, 0)
                if shape == "POLYGON" and ego["objects"].get("polygon_asymmetric"):
                    shape_code = 3
                command.append(str(shape_code))
            if yaw:
                command.append(str(yaw))
        lines = subprocess.check_output(command, text=True).splitlines()
    width, height, resolution, origin_x, origin_y = lines[0].split()
    actual = list(map(int, lines[1].split()))
    metadata = (int(width), int(height), float(resolution),
                float(origin_x), float(origin_y))
    expected_metadata = (original["width"], original["height"],
                         original["resolution_m"], original["origin_x"],
                         original["origin_y"])
    if metadata[:2] != expected_metadata[:2] or any(
            abs(a - b) > 1e-5 for a, b in zip(metadata[2:],
                                               expected_metadata[2:])):
        raise AssertionError((metadata, expected_metadata))
    if len(actual) != len(original["data"]):
        raise AssertionError("독립/원본 격자 셀 수가 다릅니다")
    mismatches = [(index, independent, source)
                  for index, (independent, source) in enumerate(
                      zip(actual, original["data"]))
                  if independent != source]
    print(f"합성 주차 활성 원본/독립 격자: {metadata[0]}x{metadata[1]}, "
          f"셀 차이 {len(mismatches)}, 첫 사례 {mismatches[:5]}")
    if mismatches:
        raise AssertionError("원본 점유 격자와 차이가 있습니다")
    grid_map = reference.get("grid_map")
    if grid_map:
        if grid_map["outer_start_index"] or grid_map["inner_start_index"]:
            raise AssertionError("시작 인덱스가 있는 원본 layer 배열의 배치를 확인해야 합니다")
        for index, name in enumerate(("points", "objects", "primitives", "combined")):
            independent = list(map(float, lines[2 + index].split()))
            # grid_map 메시지는 행렬 원시 순서이며, OccupancyGrid 직렬화와 역순이다.
            source = list(reversed(grid_map["layers"][name]))
            if len(independent) != len(source):
                raise AssertionError(f"{name} layer 셀 수가 다릅니다")
            differences = sum(abs(a - b) > 1e-6 for a, b in
                              zip(independent, source))
            print(f"  {name} layer 셀 차이 {differences}")
            if differences:
                raise AssertionError(f"원본 {name} layer와 차이가 있습니다")


if __name__ == "__main__":
    main()
