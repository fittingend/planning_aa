"""Compare current-image evaluator lane metrics with the ROS-free map adapter."""

import argparse
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
    source = json.loads(args.reference.read_text())
    source_metrics = {name: float(value)
                      for name, value in source["first_lane_metrics"]}
    if set(source_metrics) != {"ego_lane_info/lane_id", "ego_lane_info/s",
                               "ego_lane_info/t"}:
        raise AssertionError("원본 차로 지표 세 항목을 모두 수집하지 못했습니다")
    _, map_payload = next(messages(args.bag, "/map/vector_map"))
    with tempfile.TemporaryDirectory() as directory:
        map_path = Path(directory) / "map.bin"
        map_path.write_bytes(decode_lanelet_map(map_payload))
        lane_ids = [lane_id for section in source["bag_route_primitive_ids"]
                    for lane_id in section]
        command = [str(args.runner), str(map_path),
                   *map(str, source["bag_ego_xy"]), *map(str, lane_ids)]
        output = subprocess.check_output(command, text=True)
    actual_id, actual_s, actual_t = map(float, output.split())
    expected_id = source_metrics["ego_lane_info/lane_id"]
    expected_s = source_metrics["ego_lane_info/s"]
    expected_t = source_metrics["ego_lane_info/t"]
    print(f"차로 ID {actual_id}/{expected_id}, s {actual_s:.6f}/{expected_s:.6f}m, "
          f"t {actual_t:.6f}/{expected_t:.6f}m")
    if actual_id != expected_id or abs(actual_s - expected_s) > 1e-3 or \
            abs(actual_t - expected_t) > 1e-3:
        raise AssertionError("원본/독립 route-map 차로 지표가 다릅니다")


if __name__ == "__main__":
    main()
