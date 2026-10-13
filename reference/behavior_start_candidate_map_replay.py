"""K-City 경로 시작점과 지도에서 후방 출발 자세를 재계산해 원본 표식과 대조합니다."""

import argparse
import math
import subprocess
import tempfile
from pathlib import Path

from offline_rosbag_cdr import (decode_lanelet_map, decode_lanelet_route,
                                decode_marker_array, messages)


MARKERS = ("/planning/scenario_planning/lane_driving/behavior_planning/"
           "behavior_path_planner/debug/start_planner")


def angle_error(left, right):
    return abs(math.remainder(left - right, 2.0 * math.pi))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--resolution", type=float, default=2.0)
    args = parser.parse_args()
    if not math.isfinite(args.resolution) or args.resolution <= 0.0:
        parser.error("후방 표본 간격은 양수여야 합니다")
    routes = list(messages(args.bag, "/planning/mission_planning/route"))
    maps = list(messages(args.bag, "/map/vector_map"))
    frames = []
    for _, payload in messages(args.bag, MARKERS):
        poses = [item["pose"] for item in decode_marker_array(payload)
                 if item["namespace"] == "start_pose_candidates_idx"]
        if poses:
            frames.append(poses)
    if not routes or not maps or not frames:
        raise ValueError("경로·지도·출발 후보 표식이 모두 필요합니다")
    start = decode_lanelet_route(routes[0][1])["start"]
    count = max(map(len, frames))
    distances = [index * args.resolution for index in range(count)]
    with tempfile.TemporaryDirectory() as directory:
        map_path = Path(directory) / "지도.bin"
        map_path.write_bytes(decode_lanelet_map(maps[0][1]))
        result = subprocess.run(
            [str(args.runner), str(map_path), f"{start[0]:.17g}",
             f"{start[1]:.17g}", *(f"{distance:.17g}" for distance in distances)],
            check=True, text=True, capture_output=True)
    lines = result.stdout.splitlines()
    lane_id, centerline_count = map(int, lines[0].split())
    index = centerline_count + 1
    if lines[index] != f"S {count}":
        raise ValueError("C++ 표본 출력 개수가 예상과 다릅니다")
    samples = [tuple(map(float, line.split())) if line != "N" else None
               for line in lines[index + 1:index + 1 + count]]
    if len(samples) != count or any(sample is None for sample in samples):
        raise ValueError("후방 출발 자세를 모두 표본화하지 못했습니다")
    errors = []
    yaw_errors = []
    for poses in frames:
        for candidate, sample in zip(poses, samples):
            errors.append(math.dist(candidate[:2], sample[:2]))
            qx, qy, qz, qw = candidate[3:7]
            yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                             1.0 - 2.0 * (qy * qy + qz * qz))
            yaw_errors.append(angle_error(yaw, sample[2]))
    print(f"원본 프레임 {len(frames)}개, 후보 대조 {len(errors)}개, "
          f"지도 차로 {lane_id}, 중심선 점 {centerline_count}개")
    print(f"최대 위치 오차 {max(errors):.9f}m, "
          f"최대 방향 오차 {max(yaw_errors):.6f}rad")
    print("표본 수와 2m 간격은 원본 표식·실행 설정에서 지정했습니다")
    print("후보별 첫 프레임 위치 오차 " +
          ", ".join(f"{value:.9f}m" for value in errors[:len(frames[0])]))
    if (not all(math.isfinite(value) for value in errors + yaw_errors) or
            max(errors) > 1.0e-6 or max(yaw_errors) > 1.0e-6):
        raise AssertionError("원본 시작 후보의 위치 또는 방향이 독립 계산과 다릅니다")


if __name__ == "__main__":
    main()
