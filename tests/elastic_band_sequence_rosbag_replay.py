"""기록의 직전 내부 상태에서 시작해 Elastic Band를 연속 재생합니다."""

import argparse
import bisect
import csv
import math
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import decode_odometry, decode_path, decode_trajectory, messages
from elastic_band_rosbag_replay import INPUT, INTERNAL, FINAL, write_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--start", type=int, default=30)
    parser.add_argument("--frames", type=int, default=10)
    parser.add_argument("--allow-count-differences", action="store_true")
    args = parser.parse_args()
    inputs = list(messages(args.bag, INPUT))
    internals = list(messages(args.bag, INTERNAL))
    outputs = list(messages(args.bag, FINAL))
    odometry = list(messages(args.bag, "/localization/kinematic_state"))
    internal_times = [timestamp for timestamp, _ in internals]
    def matching_internal(output_time):
        insertion = bisect.bisect_left(internal_times, output_time)
        candidates = [index for index in (insertion - 1, insertion)
                      if 0 <= index < len(internals)]
        if not candidates:
            raise ValueError("대응할 내부 경로가 없습니다")
        index = min(candidates,
                    key=lambda item: abs(internal_times[item] - output_time))
        if abs(internal_times[index] - output_time) > 20_000_000:
            raise ValueError("내부 경로와 최종 출력 시각이 너무 멉니다")
        return index
    outputs_by_header = {
        decode_trajectory(payload)["stamp_ns"]: (timestamp, payload)
        for timestamp, payload in outputs}
    odometry_times = [timestamp for timestamp, _ in odometry]
    if args.start < 1 or args.frames < 1 or args.start + args.frames > len(inputs):
        raise ValueError("재생할 입력 범위가 잘못되었습니다")

    def ego_at(timestamp):
        index = bisect.bisect_right(odometry_times, timestamp) - 1
        if index < 0:
            raise ValueError("입력 이전의 차량 상태가 없습니다")
        return decode_odometry(odometry[index][1])

    references = []
    internal_references = []
    frame_ids = []
    with tempfile.TemporaryDirectory() as directory:
        directory = Path(directory)
        manifest = directory / "목록.csv"
        output_file = directory / "출력.csv"
        with manifest.open("w", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            previous_time, previous_payload = inputs[args.start - 1]
            previous_header = decode_path(previous_payload)["stamp_ns"]
            if previous_header not in outputs_by_header:
                raise ValueError("직전 입력에 대응하는 출력이 없습니다")
            previous_output_time = outputs_by_header[previous_header][0]
            previous_index = matching_internal(previous_output_time)
            previous_ego = ego_at(previous_time)
            previous_path_file = directory / "이전입력.csv"
            previous_internal_file = directory / "이전내부.csv"
            write_path(previous_path_file, decode_path(previous_payload)["points"])
            write_path(previous_internal_file,
                       decode_trajectory(internals[previous_index][1])["points"],
                       recorded=True)
            writer.writerow(("seed", previous_time / 1e9, previous_ego["x"],
                             previous_ego["y"], previous_ego["velocity_mps"],
                             previous_path_file, previous_internal_file))
            for frame in range(args.start, args.start + args.frames):
                timestamp, payload = inputs[frame]
                path = decode_path(payload)
                if path["stamp_ns"] not in outputs_by_header:
                    continue
                output_time, output_payload = outputs_by_header[path["stamp_ns"]]
                internal_index = matching_internal(output_time)
                ego = ego_at(timestamp)
                path_file = directory / f"입력{frame}.csv"
                write_path(path_file, path["points"])
                writer.writerow(("frame", timestamp / 1e9, ego["x"],
                                 ego["y"], ego["velocity_mps"], path_file))
                frame_ids.append(frame)
                references.append(decode_trajectory(output_payload)["points"])
                internal_references.append(
                    decode_trajectory(internals[internal_index][1])["points"])
        process = subprocess.run((str(args.runner), str(manifest), str(output_file)),
                                 capture_output=True, text=True)
        if process.returncode:
            raise RuntimeError(f"연속 재생 실행 실패: {process.stderr}; "
                               f"목록 첫 두 줄: {manifest.read_text().splitlines()[:2]}")
        actual = [[] for _ in references]
        actual_internal = [[] for _ in references]
        replans = 0
        resets = 0
        with output_file.open(newline="") as stream:
            for row in csv.reader(stream):
                index = int(row[0])
                if row[1] == "state":
                    resets += int(row[2])
                    replans += int(row[3])
                elif row[1] == "output":
                    actual[index].append((float(row[2]), float(row[3]),
                                          float(row[4])))
                elif row[1] == "optimized":
                    actual_internal[index].append((float(row[2]), float(row[3])))
    maximum_position = 0.0
    worst_position = None
    maximum_velocity = 0.0
    worst_velocity = None
    velocity_difference_points = 0
    velocity_difference_frames = set()
    velocity_difference_examples = []
    different_counts = []
    maximum_internal = 0.0
    worst_internal = None
    internal_outliers = []
    for offset, (computed, reference) in enumerate(
            zip(actual_internal, internal_references)):
        if len(computed) != len(reference):
            continue
        error = max(math.hypot(x - point["x"], y - point["y"])
                    for (x, y), point in zip(computed, reference))
        if error > maximum_internal:
            maximum_internal = error
            worst_internal = frame_ids[offset]
        if error > 0.1 and len(internal_outliers) < 5:
            internal_outliers.append((frame_ids[offset], round(error, 3),
                                      round(math.hypot(
                                          computed[0][0] - reference[0]["x"],
                                          computed[0][1] - reference[0]["y"]), 3)))
    for offset, (computed, reference) in enumerate(zip(actual, references)):
        if len(computed) != len(reference):
            different_counts.append((frame_ids[offset],
                                     len(computed), len(reference)))
            continue
        for point_index, ((x, y, _), point) in enumerate(
                zip(computed, reference)):
            error = math.hypot(x - point["x"], y - point["y"])
            if error > maximum_position:
                maximum_position = error
                worst_position = (frame_ids[offset], point_index)
        for point_index, ((_, _, velocity), point) in enumerate(
                zip(computed, reference)):
            error = abs(velocity - point["velocity"])
            if error > 0.01:
                velocity_difference_points += 1
                velocity_difference_frames.add(frame_ids[offset])
                if len(velocity_difference_examples) < 10:
                    velocity_difference_examples.append((
                        frame_ids[offset], point_index,
                        round(velocity, 3), round(point["velocity"], 3)))
            if error > maximum_velocity:
                maximum_velocity = error
                worst_velocity = (frame_ids[offset], point_index,
                                  round(velocity, 3),
                                  round(point["velocity"], 3))
    print(f"연속 {len(references)}프레임: 재계획 {replans}회, 초기화 {resets}회, "
          f"점 수 차이 {len(different_counts)}프레임 {different_counts[:5]}, "
          f"최대 위치 차이 "
          f"{maximum_position:.6f}m ({worst_position}), "
          f"속도 차이 {len(velocity_difference_frames)}프레임/"
          f"{velocity_difference_points}점, "
          f"최대 속도 차이 {maximum_velocity:.6f}m/s, "
          f"내부 최대 차이 {maximum_internal:.6f}m ({worst_internal}번)")
    if worst_velocity and maximum_velocity > 0.01:
        print(f"속도 최대 차이 프레임·점·독립·원본: {worst_velocity}")
        print(f"속도 차이 첫 사례: {velocity_difference_examples}")
    if internal_outliers:
        print(f"내부 큰 차이 첫 프레임(번호, 최대, 첫 점): {internal_outliers}")
    if different_counts and not args.allow_count_differences:
        raise AssertionError("최종 출력 점 수가 다른 프레임이 있습니다")


if __name__ == "__main__":
    main()
