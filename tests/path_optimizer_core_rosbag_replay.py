"""기록된 Path 경계로 독립 경로 최적화기를 연속 실행하고 출력을 비교합니다."""

import argparse
import bisect
import math
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import decode_odometry, decode_path, decode_trajectory, messages

PREFIX = "/planning/scenario_planning/lane_driving/motion_planning/"
INPUT = PREFIX + "path_smoother/path"
REFERENCE = PREFIX + "path_optimizer/debug/mpt_ref_traj"
INTERNAL = PREFIX + "path_optimizer/debug/mpt_traj"
FINAL = PREFIX + "path_optimizer/trajectory"
ODOMETRY = "/localization/kinematic_state"


def write_point(stream, point):
    stream.write(" ".join(str(value) for value in point) + "\n")


def read_trajectory(stream):
    count = int(stream.readline())
    return [tuple(float(value) for value in stream.readline().split())
            for _ in range(count)]


def deviation(actual, recorded):
    if not actual or not recorded:
        return math.inf
    return max(math.hypot(ours[0] - theirs["x"], ours[1] - theirs["y"])
               for ours, theirs in zip(actual, recorded))


def small_alignment(actual, recorded, maximum_shift):
    choices = []
    for shift in range(-maximum_shift, maximum_shift + 1):
        pairs = [(actual[i], recorded[i + shift])
                 for i in range(len(actual))
                 if 0 <= i + shift < len(recorded)]
        if len(pairs) < 3:
            continue
        choices.append((max(math.hypot(a[0] - b["x"], a[1] - b["y"])
                            for a, b in pairs), shift))
    return min(choices)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frames", type=int, default=50)
    parser.add_argument("--compare-after", type=int, default=0,
                        help="앞 프레임은 상태만 갱신하고 비교에서는 제외합니다")
    parser.add_argument("--odometry-at-final", action="store_true",
                        help="진단용으로 최종 출력 기록 시각 직전 차량 상태를 사용합니다")
    parser.add_argument("--reset-policy", action="store_true",
                        help="원본의 이전 최적화 상태 초기화 조건을 적용합니다")
    parser.add_argument("--inspect-frame", type=int,
                        help="지정 프레임의 입력·원본·독립 시작점을 표시합니다")
    parser.add_argument("--assert-ipc-stopline-reference", action="store_true",
                        help="IPC 정지선 기록의 연속 최적화 기준을 검사합니다")
    args = parser.parse_args()
    paths = list(messages(args.bag, INPUT))[:args.frames]
    odometry = list(messages(args.bag, ODOMETRY))
    odometry_times = [time for time, _ in odometry]
    recorded = {}
    final_times = {}
    for topic in (REFERENCE, INTERNAL, FINAL):
        recorded[topic] = {}
        for time, payload in messages(args.bag, topic):
            item = decode_trajectory(payload)
            if item:
                recorded[topic][item["stamp_ns"]] = item["points"]
                if topic == FINAL:
                    final_times[item["stamp_ns"]] = time
    frames = []
    for time, payload in paths:
        path = decode_path(payload)
        odometry_time = (final_times.get(path["stamp_ns"], time)
                         if args.odometry_at_final else time)
        index = bisect.bisect_right(odometry_times, odometry_time) - 1
        if index < 0:
            continue
        ego = decode_odometry(odometry[index][1])
        frames.append((path, ego))
    with tempfile.TemporaryDirectory() as directory:
        input_path = Path(directory) / "입력.txt"
        output_path = Path(directory) / "출력.txt"
        with input_path.open("w") as stream:
            stream.write(f"{len(frames)}\n")
            for path, ego in frames:
                points = path["points"]
                stream.write(f"{len(points)} {len(path['left_bound'])} "
                             f"{len(path['right_bound'])} {int(points[-1][10])}\n")
                write_point(stream, (ego["x"], ego["y"], ego["z"],
                                     ego["orientation_x"], ego["orientation_y"],
                                     ego["orientation_z"], ego["orientation_w"],
                                     ego["velocity_mps"]))
                stream.write(f"{ego['velocity_mps']}\n")
                for point in points:
                    write_point(stream, point[:8])
                for bound in path["left_bound"]:
                    write_point(stream, bound[:2])
                for bound in path["right_bound"]:
                    write_point(stream, bound[:2])
        command = [str(args.runner), str(input_path), str(output_path)]
        if args.reset_policy:
            command.append("--reset-policy")
        subprocess.run(command,
                       check=True)
        maxima = {topic: 0.0 for topic in (REFERENCE, INTERNAL, FINAL)}
        maximum_locations = {topic: None for topic in maxima}
        counts = {topic: 0 for topic in maxima}
        count_mismatches = {topic: 0 for topic in maxima}
        first_large = {topic: None for topic in maxima}
        aligned_maxima = {topic: 0.0 for topic in maxima}
        speed_maxima = {topic: 0.0 for topic in maxima}
        shifted = {topic: 0 for topic in maxima}
        failed = 0
        with output_path.open() as stream:
            for frame_index, (path, _) in enumerate(frames):
                solved = int(stream.readline())
                if solved == 0:
                    failed += 1
                    continue
                for topic in (REFERENCE, INTERNAL, FINAL):
                    actual = read_trajectory(stream)
                    if frame_index == args.inspect_frame and actual:
                        original = recorded[topic].get(path["stamp_ns"], [])
                        print(f"{frame_index} {topic}: 입력 첫 점 "
                              f"{path['points'][0][:2]}, 독립 첫 점 "
                              f"{actual[0][:2]}, 원본 첫 점 "
                              f"{(original[0]['x'], original[0]['y']) if original else '없음'}")
                    if frame_index < args.compare_after:
                        continue
                    expected = recorded[topic].get(path["stamp_ns"])
                    if expected is None:
                        continue
                    if len(actual) != len(expected):
                        count_mismatches[topic] += 1
                    error = deviation(actual, expected)
                    if error > maxima[topic]:
                        maxima[topic] = error
                        maximum_locations[topic] = frame_index
                    aligned_error, shift = small_alignment(
                        actual, expected, 3 if topic == FINAL else 1)
                    aligned_maxima[topic] = max(aligned_maxima[topic], aligned_error)
                    speed_maxima[topic] = max(
                        speed_maxima[topic],
                        max((abs(actual[i][5] - expected[i + shift]["velocity"])
                             for i in range(len(actual))
                             if 0 <= i + shift < len(expected)), default=0.0))
                    if shift:
                        shifted[topic] += 1
                    if error > 0.05 and first_large[topic] is None:
                        first_large[topic] = frame_index
                    counts[topic] += 1
        print(f"입력 {len(frames)}프레임, 풀이 실패 {failed}프레임")
        for topic in maxima:
            print(f"{topic}: 비교 {counts[topic]}프레임, 점 수 차이 "
                  f"{count_mismatches[topic]}프레임, 최대 공통 점 차이 "
                  f"{maxima[topic]:.6f}m ({maximum_locations[topic]}번), "
                  f"첫 0.05m 초과 "
                  f"{first_large[topic]}, 작은 시작점 이동 정렬 최대 "
                  f"{aligned_maxima[topic]:.6f}m, 속도 {speed_maxima[topic]:.6f}m/s, "
                  f"이동 {shifted[topic]}프레임")
        if args.assert_ipc_stopline_reference:
            if (args.compare_after != 30 or len(frames) != 571 or
                    failed != 0 or counts[FINAL] != 541 or
                    count_mismatches[FINAL] != 0 or
                    maxima[FINAL] >= 0.05 or speed_maxima[FINAL] > 1e-6):
                raise AssertionError(
                    "IPC 정지선 최적화 기준 불일치: "
                    f"입력 {len(frames)}, 실패 {failed}, 비교 {counts[FINAL]}, "
                    f"점 수 차이 {count_mismatches[FINAL]}, "
                    f"위치 {maxima[FINAL]:.6f}m, "
                    f"속도 {speed_maxima[FINAL]:.6f}m/s")


if __name__ == "__main__":
    main()
