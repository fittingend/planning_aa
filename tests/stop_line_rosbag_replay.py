"""K-City 정지선 기록을 지도 계획기에 연속 재생하고 요인 위치를 대조합니다."""

import argparse
import bisect
import collections
import math
import sqlite3
import statistics
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import (decode_lanelet_map, decode_odometry,
                                decode_path, decode_velocity_factors)


PATH = ("/planning/scenario_planning/lane_driving/behavior_planning/"
        "path_with_lane_id")
ODOMETRY = "/localization/kinematic_state"
FACTOR = "/planning/velocity_factors/stop_line"


def messages(database, topic):
    row = database.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
    if row is None:
        return []
    return list(database.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? "
        "ORDER BY timestamp", (row[0],)))


def yaw(point):
    x, y, z, w = point[3:7]
    return math.atan2(2 * (w * z + x * y),
                      1 - 2 * (y * y + z * z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frames", type=int, default=10000)
    parser.add_argument("--diagnose", action="store_true",
                        help="가장 큰 정지점 위치 차이를 출력합니다")
    args = parser.parse_args()
    with sqlite3.connect(f"file:{args.bag}?mode=ro", uri=True) as database:
        paths = messages(database, PATH)[:args.frames]
        odometry = messages(database, ODOMETRY)
        factors = messages(database, FACTOR)
        map_messages = messages(database, "/map/vector_map")
    if not map_messages or not paths or not odometry:
        raise ValueError("지도·경로·차량 상태 중 일부가 없습니다")
    odometry_times = [time for time, _ in odometry]
    frames = []
    for time, payload in paths:
        state_index = bisect.bisect_right(odometry_times, time) - 1
        if state_index < 0:
            continue
        path = decode_path(payload, with_lane_ids=True)
        if len(path["points"]) < 2:
            continue
        frames.append((time, path, decode_odometry(odometry[state_index][1])))
    with tempfile.TemporaryDirectory() as directory:
        directory = Path(directory)
        map_file = directory / "지도.bin"
        source = directory / "입력.txt"
        result_file = directory / "결과.txt"
        map_file.write_bytes(decode_lanelet_map(map_messages[0][1]))
        with source.open("w") as stream:
            stream.write(f"{len(frames)}\n")
            for time, path, ego in frames:
                stream.write(f"{time * 1e-9:.9f} {ego['x']:.17g} "
                             f"{ego['y']:.17g} {ego['velocity_mps']:.17g} "
                             f"{len(path['points'])}\n")
                for point, lane_ids in zip(path["points"], path["lane_ids"]):
                    stream.write(f"{point[0]:.17g} {point[1]:.17g} "
                                 f"{point[2]:.17g} {yaw(point):.17g} "
                                 f"{point[7]:.17g} {len(lane_ids)} " +
                                 " ".join(map(str, lane_ids)) + "\n")
        subprocess.run([str(args.runner), str(map_file),
                        str(source), str(result_file)], check=True)
        results = [line.split() for line in result_file.read_text().splitlines()]
    frame_times = [time for time, _, _ in frames]
    original = {}
    statuses = collections.Counter()
    for time, payload in factors:
        decoded = decode_velocity_factors(payload)
        index = bisect.bisect_right(frame_times, time) - 1
        if index < 0 or time - frame_times[index] > 50_000_000:
            continue
        original[index] = decoded["factors"]
        statuses.update(factor["status"] for factor in decoded["factors"])
    comparison = 0
    original_active = independent_active = both = 0
    errors = []
    errors_by_status = collections.defaultdict(list)
    stopped_distance_residuals = []
    detailed_errors = []
    disagreement = []
    for index, row in enumerate(results):
        if index not in original:
            continue
        comparison += 1
        reference = original[index]
        observed = row[2] == "1"
        original_active += bool(reference)
        independent_active += observed
        both += bool(reference) and observed
        if reference and observed:
            error, item = min(
                ((math.hypot(float(row[3]) - item["pose"][0],
                             float(row[4]) - item["pose"][1]), item)
                 for item in reference), key=lambda pair: pair[0])
            errors.append(error)
            errors_by_status[item["status"]].append(error)
            if item["status"] == 2:
                stopped_distance_residuals.append(
                    abs(error - abs(item["distance"])))
            detailed_errors.append((error, index, item["status"],
                                    item["distance"],
                                    (float(row[3]), float(row[4])),
                                    item["pose"][:2]))
        if bool(reference) != observed:
            disagreement.append((index, bool(reference), observed,
                                 int(row[1])))
    print(f"입력 {len(frames)}프레임, 요인 대응 {comparison}프레임, "
          f"원본 요인 {original_active}, 독립 정지 {independent_active}, "
          f"동시 {both}, 독립만 {independent_active - both}, "
          f"원본만 {original_active - both}")
    print(f"원본 상태 {dict(sorted(statuses.items()))}")
    if errors:
        print(f"정지점 위치 차이 {len(errors)}프레임: "
              f"중앙값 {statistics.median(errors):.6f}m, "
              f"최대 {max(errors):.6f}m")
        for status, values in sorted(errors_by_status.items()):
            print(f"  상태 {status}: {len(values)}프레임, "
                  f"중앙값 {statistics.median(values):.6f}m, "
                  f"최대 {max(values):.6f}m")
        if stopped_distance_residuals:
            print("  정차 중 위치 차이와 원본 음수 거리의 절대값 차이: "
                  f"최대 {max(stopped_distance_residuals):.6f}m")
    if disagreement:
        print(f"활성 불일치 첫 사례 {disagreement[:12]}")
    if args.diagnose:
        print("위치 차이 상위 사례", sorted(
            detailed_errors, reverse=True)[:12])


if __name__ == "__main__":
    main()
