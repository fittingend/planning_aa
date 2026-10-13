"""기록된 충돌 거리와 여유 거리로 원본 정지점을 독립적으로 예측합니다."""

import argparse
import bisect
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import (
    decode_float32_multi_array_stamped, decode_odometry,
    decode_trajectory, messages)

PREFIX = "/planning/scenario_planning/lane_driving/"
INPUT = PREFIX + "motion_planning/motion_velocity_planner/trajectory"
OUTPUT = PREFIX + "trajectory"
DEBUG = (PREFIX + "motion_planning/obstacle_cruise_planner/"
         "debug/stop_planning_info")
ODOMETRY = "/localization/kinematic_state"


def arcs(points):
    values = [0.0]
    for front, back in zip(points, points[1:]):
        values.append(values[-1] + math.hypot(
            back["x"] - front["x"], back["y"] - front["y"]))
    return values


def yaw(point):
    x, y = point["orientation_x"], point["orientation_y"]
    z, w = point["orientation_z"], point["orientation_w"]
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def nearest_segment(points, target, soft=False):
    nearest = None
    if soft:
        for use_yaw in (True, False):
            best = math.inf
            found = False
            for i, point in enumerate(points):
                squared = ((point["x"] - target["x"]) ** 2 +
                           (point["y"] - target["y"]) ** 2)
                angle = math.remainder(yaw(point) - yaw(target), 2 * math.pi)
                if squared > 9.0 or (use_yaw and abs(angle) > 1.046):
                    if found:
                        break
                    continue
                found = True
                if squared < best:
                    best, nearest = squared, i
            if found:
                break
    if nearest is None:
        nearest = min(range(len(points)), key=lambda i: math.hypot(
            points[i]["x"] - target["x"], points[i]["y"] - target["y"]))
    if nearest == 0:
        return 0
    if nearest == len(points) - 1:
        return nearest - 1
    dx = points[nearest + 1]["x"] - points[nearest]["x"]
    dy = points[nearest + 1]["y"] - points[nearest]["y"]
    projection = (target["x"] - points[nearest]["x"]) * dx + (
        target["y"] - points[nearest]["y"]) * dy
    return nearest - 1 if projection <= 0.0 else nearest


def projected_arc(points, lengths, target):
    segment = nearest_segment(points, target)
    front, back = points[segment:segment + 2]
    dx, dy = back["x"] - front["x"], back["y"] - front["y"]
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return lengths[segment]
    ratio = max(0.0, min(1.0, ((target["x"] - front["x"]) * dx +
                             (target["y"] - front["y"]) * dy) / length ** 2))
    return lengths[segment] + ratio * length


def inserted_index(source, target):
    for i in range(len(source)):
        if math.hypot(source[i]["x"] - target[i]["x"],
                      source[i]["y"] - target[i]["y"]) > 1e-4:
            return i
    return len(source)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--odometry-at-output", action="store_true",
                        help="차량 위치를 출력 기록 시각 직전에서 고릅니다")
    parser.add_argument("--stateful-hold", action="store_true",
                        help="저속에서 이전 독립 정지점을 0.3m 안이면 유지합니다")
    parser.add_argument("--show-outliers", action="store_true",
                        help="0.05m를 넘는 프레임의 시각과 상태를 출력합니다")
    parser.add_argument("--odometry-steps-back", type=int, default=0,
                        help="진단용으로 차량 상태를 지정한 메시지 수만큼 이전으로 옮깁니다")
    args = parser.parse_args()
    inputs = {item["stamp_ns"]: (time, item["points"])
              for time, payload in messages(args.bag, INPUT)
              if (item := decode_trajectory(payload))}
    debug = list(messages(args.bag, DEBUG))
    debug_times = [time for time, _ in debug]
    odometry = list(messages(args.bag, ODOMETRY))
    odometry_times = [time for time, _ in odometry]
    errors = []
    low_speed_large = 0
    delayed_odometry_large = 0
    skipped = 0
    previous_stop = None
    held = 0
    for time, payload in messages(args.bag, OUTPUT):
        message = decode_trajectory(payload)
        input_item = inputs.get(message["stamp_ns"])
        source = input_item[1] if input_item else None
        if source is None or len(message["points"]) != len(source) + 1:
            continue
        index = inserted_index(source, message["points"])
        if index >= len(message["points"]) or (
                abs(message["points"][index]["velocity"]) >= 1e-4):
            continue
        stop = message["points"][index]
        debug_index = bisect.bisect_right(debug_times, time)
        odometry_time = time if args.odometry_at_output else input_item[0]
        odometry_index = (bisect.bisect_right(odometry_times, odometry_time) - 1 -
                          args.odometry_steps_back)
        if stop is None or debug_index == len(debug) or odometry_index < 0 or (
                debug_times[debug_index] - time > 20_000_000):
            skipped += 1
            continue
        values = decode_float32_multi_array_stamped(debug[debug_index][1])["data"]
        if len(values) < 6 or values[3] <= 0.0:
            skipped += 1
            continue
        ego = decode_odometry(odometry[odometry_index][1])
        lengths = arcs(source)
        predicted = max(0.0, lengths[nearest_segment(source, ego, soft=True)] +
                        values[3] - values[5])
        held_this_frame = False
        if args.stateful_hold and abs(ego["velocity_mps"]) < 0.01 and previous_stop:
            previous_path, previous_arc = previous_stop
            previous_lengths = arcs(previous_path)
            old_front_offset = projected_arc(
                previous_path, previous_lengths, source[0])
            carried = previous_arc - old_front_offset
            if abs(carried - predicted) < 0.3:
                predicted = carried
                held += 1
                held_this_frame = True
        observed = projected_arc(source, lengths, stop)
        previous_stop = source, predicted
        error = abs(predicted - observed)
        errors.append(error)
        if error > 0.05:
            if args.show_outliers:
                print(f"초과 출력시각 {time}, 입력시각 {input_item[0]}, "
                      f"독립 거리 {predicted:.6f}m, 기록 거리 {observed:.6f}m, "
                      f"차량 속도 {ego['velocity_mps']:.6f}m/s, "
                      f"정지점 유지 {held_this_frame}")
            if abs(ego["velocity_mps"]) < 0.01:
                low_speed_large += 1
            if odometry_time - odometry_times[odometry_index] > 100_000_000:
                delayed_odometry_large += 1
    print(f"정지 거리 비교 {len(errors)}프레임, 제외 {skipped}프레임, "
          f"최대 차이 {max(errors, default=0.0):.6f}m, "
          f"0.05m 초과 {sum(value > 0.05 for value in errors)}프레임, "
          f"이 중 저속 {low_speed_large}프레임, "
          f"차량 상태 기록 0.1초 초과 지연 {delayed_odometry_large}프레임, "
          f"이전 정지점 유지 {held}프레임")


if __name__ == "__main__":
    main()
