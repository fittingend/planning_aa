"""Report the known original/independent freespace difference on a generated grid."""

import argparse
import gzip
import json
import math
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("freespace_reference", type=Path)
    parser.add_argument("costmap_reference", type=Path)
    parser.add_argument("--expect-match", action="store_true")
    parser.add_argument("--expect-terminal-match", action="store_true")
    parser.add_argument("--maximum-position-error", type=float)
    args = parser.parse_args()
    freespace = json.loads(args.freespace_reference.read_text())
    with gzip.open(args.costmap_reference, "rt") as stream:
        occupancy = json.load(stream)["output"]
    with tempfile.TemporaryDirectory() as directory:
        grid_path = Path(directory) / "grid.txt"
        grid_path.write_text(" ".join(map(str, [
            occupancy["width"], occupancy["height"],
            occupancy["resolution_m"], occupancy["origin_x"],
            occupancy["origin_y"], *occupancy["data"]])))
        command = [str(args.runner), *map(str, freespace["input"]["ego"]),
                   *map(str, freespace["input"]["goal"]), str(grid_path)]
        if freespace["input"].get("adapt_expansion_distance") is False:
            command.append("no-adapt")
        output = subprocess.check_output(command, text=True,
                                         stderr=subprocess.DEVNULL)
    independent = [[float(value) for value in line.split()]
                   for line in output.splitlines()]
    source = freespace["output"]["points"]
    aligned = [math.dist(a[:2], b[:2])
               for a, b in zip(source, independent)]
    speed_differences = [abs(a[4] - b[2])
                         for a, b in zip(source, independent)]
    result = {
        "original_points": len(source),
        "independent_points": len(independent),
        "first_over_1cm": next((i for i, value in enumerate(aligned)
                                if value > 0.01), None),
        "maximum_aligned_position_error_m": max(aligned),
        "endpoint_error_m": math.dist(source[-1][:2], independent[-1][:2]),
        "maximum_aligned_speed_error_mps": max(speed_differences),
    }
    print(json.dumps(result, indent=2))
    if args.expect_match and (len(source) != len(independent) or
                              result["maximum_aligned_position_error_m"] > 0.001 or
                              result["maximum_aligned_speed_error_mps"] > 1e-6):
        raise AssertionError("원본/독립 생성 격자 경로가 일치하지 않습니다")
    if args.expect_terminal_match and (
            len(source) != len(independent) or
            result["endpoint_error_m"] > 0.001 or
            result["maximum_aligned_speed_error_mps"] > 1e-6):
        raise AssertionError("원본/독립 생성 격자 경로의 점 수·종점·속도가 다릅니다")
    if (args.maximum_position_error is not None and
            result["maximum_aligned_position_error_m"] >
            args.maximum_position_error):
        raise AssertionError("생성 격자 경로의 위치 차이가 허용값을 넘었습니다")


if __name__ == "__main__":
    main()
