"""K-City 횡단보도 기록의 경로·예측 객체를 독립 계획기에 연속 재생합니다."""

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
from offline_rosbag_cdr import (decode_acceleration, decode_lanelet_map,
                                decode_odometry, decode_path,
                                decode_predicted_objects,
                                decode_velocity_factors)


PATH = ("/planning/scenario_planning/lane_driving/behavior_planning/"
        "path_with_lane_id")
OBJECTS = "/perception/object_recognition/objects"
FACTOR = "/planning/velocity_factors/crosswalk"


def messages(database, topic):
    row = database.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
    if row is None:
        return []
    return list(database.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (row[0],)))


def yaw(pose):
    x, y, z, w = pose[3:7]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frames", type=int, default=10000)
    parser.add_argument("--rtc", action="store_true")
    parser.add_argument("--diagnose", action="store_true")
    args = parser.parse_args()
    with sqlite3.connect(f"file:{args.bag}?mode=ro", uri=True) as database:
        paths = messages(database, PATH)[:args.frames]
        odometry = messages(database, "/localization/kinematic_state")
        accelerations = messages(database, "/localization/acceleration")
        objects = messages(database, OBJECTS)
        factors = messages(database, FACTOR)
        maps = messages(database, "/map/vector_map")
    if not maps or not paths or not odometry:
        raise ValueError("지도·경로·차량 상태 중 일부가 없습니다")
    odometry_times = [time for time, _ in odometry]
    acceleration_times = [time for time, _ in accelerations]
    object_times = [time for time, _ in objects]
    frames = []
    object_cache = {}
    for time, payload in paths:
        state_index = bisect.bisect_right(odometry_times, time) - 1
        if state_index < 0:
            continue
        path = decode_path(payload, with_lane_ids=True)
        if len(path["points"]) < 2:
            continue
        acceleration_index = bisect.bisect_right(acceleration_times, time) - 1
        acceleration = (decode_acceleration(
            accelerations[acceleration_index][1])["acceleration_mps2"]
            if acceleration_index >= 0 else 0.0)
        object_index = bisect.bisect_right(object_times, time) - 1
        if object_index >= 0 and object_index not in object_cache:
            object_cache[object_index] = decode_predicted_objects(
                objects[object_index][1])["objects"]
        frame_objects = object_cache.get(object_index, [])
        frames.append((time, path,
                       decode_odometry(odometry[state_index][1]),
                       acceleration, frame_objects))
    with tempfile.TemporaryDirectory() as directory:
        directory = Path(directory)
        map_file = directory / "지도.bin"
        source = directory / "입력.txt"
        result_file = directory / "결과.txt"
        map_file.write_bytes(decode_lanelet_map(maps[0][1]))
        with source.open("w") as stream:
            stream.write(f"{len(frames)}\n")
            for time, path, ego, acceleration, frame_objects in frames:
                stream.write(f"{time * 1e-9:.9f} {ego['x']:.17g} "
                             f"{ego['y']:.17g} {ego['velocity_mps']:.17g} "
                             f"{acceleration:.17g} {len(path['points'])}\n")
                for point, lane_ids in zip(path["points"], path["lane_ids"]):
                    stream.write(f"{point[0]:.17g} {point[1]:.17g} "
                                 f"{point[2]:.17g} {yaw(point):.17g} "
                                 f"{point[7]:.17g} {len(lane_ids)} " +
                                 " ".join(map(str, lane_ids)) + "\n")
                relevant = [item for item in frame_objects if
                            item["classification"] and
                            item["classification"][0][0] in (5, 6, 7)]
                stream.write(f"{len(relevant)}\n")
                for item in relevant:
                    pose = item["pose"]
                    # 현재 위치와 예측 자세·속도는 CDR의 원본 필드를 그대로 사용합니다.
                    stream.write(
                        f"{item['uuid']} {pose[0]:.17g} {pose[1]:.17g} "
                        f"{item['classification'][0][0]} "
                        f"{item['dimensions'][0]:.17g} "
                        f"{item['dimensions'][1]:.17g} "
                        f"{item['twist'][0]:.17g} {item['twist'][1]:.17g} "
                        f"{len(item['predicted_paths'])}\n")
                    for predicted in item["predicted_paths"]:
                        stream.write(f"{len(predicted['poses'])}\n")
                        for predicted_pose in predicted["poses"]:
                            stream.write(
                                f"{predicted_pose[0]:.17g} "
                                f"{predicted_pose[1]:.17g} "
                                f"{yaw(predicted_pose):.17g}\n")
        subprocess.run([str(args.runner), str(map_file), str(source),
                        str(result_file), "1" if args.rtc else "0"],
                       check=True)
        results = [line.split() for line in result_file.read_text().splitlines()]
    frame_times = [time for time, *_ in frames]
    original = {}
    status_counts = collections.Counter()
    for time, payload in factors:
        index = bisect.bisect_right(frame_times, time) - 1
        if index < 0 or time - frame_times[index] > 50_000_000:
            continue
        decoded = decode_velocity_factors(payload)["factors"]
        original[index] = decoded
        status_counts.update(item["status"] for item in decoded)
    comparable = original_active = independent_active = both = 0
    errors = []
    detailed_errors = []
    mismatches = []
    approaching_only = 0
    unknown_active = unknown_both = 0
    negative_distance_residuals = []
    for index, row in enumerate(results):
        if index not in original:
            continue
        comparable += 1
        reference = original[index]
        observed = row[2] == "1"
        original_active += bool(reference)
        independent_active += observed
        both += bool(reference) and observed
        unknown = any(item["status"] == 0 for item in reference)
        unknown_active += unknown
        unknown_both += unknown and observed
        approaching_only += bool(reference) and not unknown
        if reference and observed:
            error, nearest = min(
                ((math.hypot(float(row[3]) - item["pose"][0],
                             float(row[4]) - item["pose"][1]), item)
                 for item in reference), key=lambda value: value[0])
            errors.append(error)
            detailed_errors.append((error, index, nearest["status"],
                                    nearest["distance"],
                                    (float(row[3]), float(row[4])),
                                    nearest["pose"][:2]))
            if nearest["distance"] < 0:
                negative_distance_residuals.append(
                    abs(error - abs(nearest["distance"])))
        if bool(reference) != observed:
            mismatches.append((index, bool(reference), observed, int(row[1]),
                               len(frames[index][4])))
    print(f"입력 {len(frames)}프레임, 요인 대응 {comparable}프레임, "
          f"원본 요인 {original_active}, 독립 정지 {independent_active}, "
          f"동시 {both}, 독립만 {independent_active - both}, "
          f"원본만 {original_active - both}")
    print(f"원본 상태 {dict(sorted(status_counts.items()))}")
    print(f"원본 UNKNOWN 포함 {unknown_active}프레임 중 독립 정지 "
          f"{unknown_both}프레임, APPROACHING만 {approaching_only}프레임")
    if errors:
        print(f"동시 활성 위치 차이 {len(errors)}프레임: "
              f"중앙값 {statistics.median(errors):.6f}m, "
              f"최대 {max(errors):.6f}m")
    if negative_distance_residuals:
        print(f"음수 거리 요인의 위치 차이와 거리 절대값 잔차: "
              f"중앙값 {statistics.median(negative_distance_residuals):.6f}m, "
              f"최대 {max(negative_distance_residuals):.6f}m")
    if args.diagnose:
        print(f"활성 불일치 첫 사례 {mismatches[:25]}")
        print(f"위치 차이 상위 사례 {sorted(detailed_errors, reverse=True)[:12]}")


if __name__ == "__main__":
    main()
