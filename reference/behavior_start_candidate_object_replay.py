"""기록된 인식 객체의 선택 시각을 바꿔 출발 후보 수를 원본과 대조합니다."""

import argparse
import bisect
import collections
import math
import subprocess
import tempfile
from pathlib import Path

from offline_rosbag_cdr import (decode_lanelet_map, decode_lanelet_route,
                                decode_marker_array, decode_predicted_objects,
                                messages)


MARKERS = ("/planning/scenario_planning/lane_driving/behavior_planning/"
           "behavior_path_planner/debug/start_planner")
OBJECTS = "/perception/object_recognition/objects"


def yaw_of(pose):
    qx, qy, qz, qw = pose[3:7]
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--max-back-distance", type=float)
    parser.add_argument("--perception-lag-s", type=float, default=0.0)
    parser.add_argument("--same-callback-goal", action="store_true",
                        help="같은 인식 묶음으로 출발·목표 후보를 함께 계산합니다")
    args = parser.parse_args()
    if (args.max_back_distance is not None and
            (not math.isfinite(args.max_back_distance) or
             args.max_back_distance < 0.0)):
        parser.error("최대 후진 거리는 0 이상이어야 합니다")
    if not math.isfinite(args.perception_lag_s) or args.perception_lag_s < 0.0:
        parser.error("인식 입력 지연은 0 이상이어야 합니다")
    route_raw = next(messages(args.bag, "/planning/mission_planning/route"), None)
    map_raw = next(messages(args.bag, "/map/vector_map"), None)
    perception = list(messages(args.bag, OBJECTS))
    frames = []
    for stamp, payload in messages(args.bag, MARKERS):
        count = sum(marker["namespace"] == "start_pose_candidates_idx"
                    for marker in decode_marker_array(payload))
        if count:
            frames.append((stamp, count))
    if route_raw is None or map_raw is None or not perception or not frames:
        raise ValueError("경로·지도·인식 객체·출발 후보가 모두 필요합니다")
    route = decode_lanelet_route(route_raw[1])
    start = route["start"]
    goal = route["goal"]
    goal_yaw = yaw_of(goal)
    lane_id = route["segments"][0]["preferred"][0]
    times = [stamp for stamp, _ in perception]
    paired = []
    for stamp, count in frames:
        index = bisect.bisect_right(
            times, stamp - round(args.perception_lag_s * 1e9)) - 1
        if index < 0:
            continue
        paired.append((stamp, count, perception[index]))
    if not paired:
        raise ValueError("출발 표식보다 앞선 인식 객체 메시지가 없습니다")
    with tempfile.TemporaryDirectory() as directory:
        map_path = Path(directory) / "지도.bin"
        input_path = Path(directory) / "입력.txt"
        output_path = Path(directory) / "결과.txt"
        map_path.write_bytes(decode_lanelet_map(map_raw[1]))
        with input_path.open("w") as stream:
            header = f"{start[0]:.17g} {start[1]:.17g} "
            header += f"{lane_id} {len(paired)}"
            if args.same_callback_goal:
                header += (f" {goal[0]:.17g} {goal[1]:.17g} "
                           f"{goal_yaw:.17g}")
            stream.write(header + "\n")
            for callback_stamp, _, (perception_stamp, raw) in paired:
                objects = decode_predicted_objects(raw)["objects"]
                frame_header = str(len(objects))
                if args.same_callback_goal:
                    frame_header += f" {callback_stamp} {perception_stamp}"
                stream.write(frame_header + "\n")
                for obj in objects:
                    pose = obj["pose"]
                    twist = obj["twist"]
                    footprint = obj["footprint"]
                    stream.write(" ".join([
                        f"{pose[0]:.17g}", f"{pose[1]:.17g}",
                        f"{yaw_of(pose):.17g}", f"{twist[0]:.17g}",
                        f"{twist[1]:.17g}", str(obj["shape_type"]),
                        f"{obj['dimensions'][0]:.17g}",
                        f"{obj['dimensions'][1]:.17g}", "1",
                        str(len(footprint)),
                        *(f"{value:.17g}" for point in footprint
                          for value in point[:2])]) + "\n")
        command = [str(args.runner), str(map_path), str(input_path),
                   str(output_path)]
        if args.max_back_distance is not None:
            command.append(f"{args.max_back_distance:.17g}")
        subprocess.run(command, check=True)
        results = [tuple(map(int, line.split()))
                   for line in output_path.read_text().splitlines()]
        independent = [result[0] for result in results]
        selected_objects = [result[1] for result in results]
        goal_reasons = ([result[2] for result in results]
                        if args.same_callback_goal else [])
    if len(independent) != len(paired):
        raise ValueError("출발 후보 재생 결과 개수가 다릅니다")
    matching = sum(predicted == original for predicted, (_, original, _) in
                   zip(independent, paired))
    delays = [(stamp - object_message[0]) * 1e-6
              for stamp, _, object_message in paired]
    print(f"원본 표식 {len(frames)}프레임, 이전 인식 객체와 짝 {len(paired)}프레임")
    print(f"가정한 최대 후진 거리 "
          f"{args.max_back_distance if args.max_back_distance is not None else 20.0:g}m")
    print(f"가정한 인식 입력 지연 {args.perception_lag_s:g}초")
    print(f"원본 후보 수 {dict(sorted(collections.Counter(c for _, c in frames).items()))}")
    print(f"독립 후보 수 {dict(sorted(collections.Counter(independent).items()))}")
    print("독립 정지 객체 선별 수 " +
          str(dict(sorted(collections.Counter(selected_objects).items()))))
    if args.same_callback_goal:
        print("동일 인식 묶음의 목표 후보 안전 사유 " +
              str(dict(sorted(collections.Counter(goal_reasons).items()))))
    print(f"후보 수 일치 {matching}/{len(paired)}, "
          f"인식 입력 시각 차이 최대 {max(delays):.1f}ms")
    print("인식 메시지와 출발 계획의 실제 콜백 짝 및 실행 매개변수는 기록에 없습니다")


if __name__ == "__main__":
    main()
