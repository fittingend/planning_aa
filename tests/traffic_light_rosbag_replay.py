"""K-City 신호등 기록을 지도 계획기에 연속 재생하고 요인을 대조합니다."""

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
                                decode_traffic_light_groups,
                                decode_velocity_factors)


PATH = ("/planning/scenario_planning/lane_driving/behavior_planning/"
        "path_with_lane_id")
ODOMETRY = "/localization/kinematic_state"
ACCELERATION = "/localization/acceleration"
SIGNAL = "/perception/traffic_light_recognition/traffic_signals"
FACTOR = "/planning/velocity_factors/traffic_light"


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
    parser.add_argument("--diagnose", action="store_true")
    args = parser.parse_args()
    with sqlite3.connect(f"file:{args.bag}?mode=ro", uri=True) as database:
        paths = messages(database, PATH)[:args.frames]
        odometry = messages(database, ODOMETRY)
        accelerations = messages(database, ACCELERATION)
        signals = messages(database, SIGNAL)
        factors = messages(database, FACTOR)
        map_messages = messages(database, "/map/vector_map")
    if not map_messages or not paths or not odometry:
        raise ValueError("지도·경로·차량 상태 중 일부가 없습니다")
    odometry_times = [time for time, _ in odometry]
    acceleration_times = [time for time, _ in accelerations]
    signal_times = [time for time, _ in signals]
    frames = []
    colors = collections.Counter()
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
        signal_index = bisect.bisect_right(signal_times, time) - 1
        signal = (decode_traffic_light_groups(signals[signal_index][1])
                  if signal_index >= 0 else None)
        elements = [element for group in signal["groups"]
                    for element in group["elements"]] if signal else []
        color = elements[0][0] if elements else 0
        colors[color] += 1
        stop = not any(element[0] == 3 and element[1] == 1
                       and element[2] != 1 for element in elements)
        frames.append((time, path,
                       decode_odometry(odometry[state_index][1]),
                       acceleration, signal, stop, color))
    with tempfile.TemporaryDirectory() as directory:
        directory = Path(directory)
        map_file = directory / "지도.bin"
        source = directory / "입력.txt"
        result_file = directory / "결과.txt"
        map_file.write_bytes(decode_lanelet_map(map_messages[0][1]))
        with source.open("w") as stream:
            stream.write(f"{len(frames)}\n")
            for time, path, ego, acceleration, signal, stop, _ in frames:
                stream.write(f"{time * 1e-9:.9f} {ego['x']:.17g} "
                             f"{ego['y']:.17g} {ego['velocity_mps']:.17g} "
                             f"{acceleration:.17g} {int(signal is not None)} "
                             f"{int(stop)} "
                             f"{signal['stamp_ns'] * 1e-9 if signal else 0:.9f} "
                             f"{len(path['points'])}\n")
                for point, lane_ids in zip(path["points"], path["lane_ids"]):
                    stream.write(f"{point[0]:.17g} {point[1]:.17g} "
                                 f"{point[2]:.17g} {yaw(point):.17g} "
                                 f"{point[7]:.17g} {len(lane_ids)} " +
                                 " ".join(map(str, lane_ids)) + "\n")
        subprocess.run([str(args.runner), str(map_file),
                        str(source), str(result_file)], check=True)
        results = [line.split() for line in result_file.read_text().splitlines()]
    frame_times = [time for time, *_ in frames]
    original = {}
    for time, payload in factors:
        decoded = decode_velocity_factors(payload)
        index = bisect.bisect_right(frame_times, time) - 1
        if index < 0 or time - frame_times[index] > 50_000_000:
            continue
        original[index] = decoded["factors"]
    comparison = original_factor = independent_stop = both = 0
    known_comparison = known_original = known_independent = known_both = 0
    warm_comparison = warm_original = warm_independent = warm_both = 0
    first_signal_index = next((index for index, frame in enumerate(frames)
                               if frame[4] is not None), len(frames))
    errors = []
    mismatches = []
    for index, row in enumerate(results):
        if index not in original:
            continue
        comparison += 1
        reference = original[index]
        observed = row[2] == "1"
        original_factor += bool(reference)
        independent_stop += observed
        both += bool(reference) and observed
        if frames[index][4] is not None:
            known_comparison += 1
            known_original += bool(reference)
            known_independent += observed
            known_both += bool(reference) and observed
            if index >= first_signal_index + 2:
                warm_comparison += 1
                warm_original += bool(reference)
                warm_independent += observed
                warm_both += bool(reference) and observed
        if reference and observed:
            errors.append(min(math.hypot(float(row[3]) - factor["pose"][0],
                                         float(row[4]) - factor["pose"][1])
                              for factor in reference))
        if bool(reference) != observed:
            mismatches.append((index, bool(reference), observed,
                               frames[index][6], int(row[1])))
    print(f"입력 {len(frames)}프레임, 요인 대응 {comparison}프레임, "
          f"원본 요인 {original_factor}, 독립 정지 {independent_stop}, "
          f"동시 {both}, 독립만 {independent_stop - both}, "
          f"원본만 {original_factor - both}")
    print(f"입력 신호 색 {dict(sorted(colors.items()))}")
    print(f"신호가 기록된 요인 대응 {known_comparison}프레임: "
          f"원본 요인 {known_original}, 독립 정지 {known_independent}, "
          f"동시 {known_both}, 독립만 {known_independent - known_both}, "
          f"원본만 {known_original - known_both}")
    print(f"최초 신호 뒤 두 프레임을 상태 준비에 쓴 {warm_comparison}프레임: "
          f"원본 요인 {warm_original}, 독립 정지 {warm_independent}, "
          f"동시 {warm_both}, 독립만 {warm_independent - warm_both}, "
          f"원본만 {warm_original - warm_both}")
    if errors:
        print(f"요인 자세와 독립 정지점 위치 차이 {len(errors)}프레임: "
              f"중앙값 {statistics.median(errors):.6f}m, "
              f"최대 {max(errors):.6f}m")
    if args.diagnose:
        print(f"불일치 첫 사례 {mismatches[:20]}")
        print("신호가 있는 불일치", [item for item in mismatches
                            if frames[item[0]][4] is not None][:20])


if __name__ == "__main__":
    main()
