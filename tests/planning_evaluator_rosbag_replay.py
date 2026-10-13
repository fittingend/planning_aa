"""실측 최종 궤적을 evaluator 코어에 넣어 기본 지표를 원본과 대조합니다."""

import argparse
import bisect
import math
import statistics
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import (
    decode_diagnostic_array, decode_odometry, decode_predicted_objects,
    decode_trajectory, messages)


TRAJECTORY = "/planning/scenario_planning/trajectory"
ODOMETRY = "/localization/kinematic_state"
METRICS = "/planning/planning_evaluator/metrics"
REFERENCE = ("/planning/scenario_planning/lane_driving/motion_planning/"
             "path_optimizer/trajectory")
OBJECTS = "/perception/object_recognition/objects"
NAMES = ("length", "point_interval", "velocity", "acceleration",
         "jerk", "duration", "curvature", "relative_angle",
         "lateral_deviation", "yaw_deviation", "velocity_deviation",
         "obstacle_distance")
BASE_NAMES = NAMES[:6]
FIELDS = ("min", "max", "mean")


def latest(rows, times, stamp):
    index = bisect.bisect_right(times, stamp) - 1
    return rows[index] if index >= 0 else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--input-settle-ms", type=float, default=0.0,
                        help="출력 직전 도착한 입력의 callback 경합을 배제할 시간")
    parser.add_argument("--reference-settle-ms", type=float, default=0.0,
                        help="기준 궤적 callback 경합만 배제할 시간")
    parser.add_argument("--maximum-basic-mismatches", type=int,
                        default=None)
    parser.add_argument("--maximum-extended-mismatches", type=int,
                        default=None)
    args = parser.parse_args()
    trajectories = [(t, decode_trajectory(raw))
                    for t, raw in messages(args.bag, TRAJECTORY)]
    odometry = [(t, decode_odometry(raw))
                for t, raw in messages(args.bag, ODOMETRY)]
    references = [(t, decode_trajectory(raw))
                  for t, raw in messages(args.bag, REFERENCE)]
    objects = [(t, decode_predicted_objects(raw))
               for t, raw in messages(args.bag, OBJECTS)]
    recorded = [(t, decode_diagnostic_array(raw))
                for t, raw in messages(args.bag, METRICS)]
    if not trajectories or not odometry or not recorded:
        raise ValueError("궤적·차량 상태·원본 지표가 모두 필요합니다")
    trajectory_times = [t for t, _ in trajectories]
    odometry_times = [t for t, _ in odometry]
    reference_times = [t for t, _ in references]
    object_times = [t for t, _ in objects]
    selected = []
    for stamp, metric_array in recorded:
        trajectory = latest(trajectories, trajectory_times,
                            stamp - round(args.input_settle_ms * 1e6))
        ego = latest(odometry, odometry_times, stamp)
        reference = latest(references, reference_times,
                           stamp - round(args.reference_settle_ms * 1e6))
        perceived = latest(objects, object_times, stamp)
        if not trajectory or not ego or len(trajectory[1]["points"]) < 2:
            continue
        diagnostic = {status["name"]: status["values"]
                      for status in metric_array["statuses"]}
        if not all(name in diagnostic for name in BASE_NAMES):
            continue
        selected.append((stamp, trajectory, ego, reference, perceived,
                         diagnostic))
        if args.frames and len(selected) >= args.frames:
            break
    if not selected:
        raise ValueError("비교할 지표가 없습니다")
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory) / "input.txt"
        output = Path(directory) / "output.txt"
        with source.open("w") as stream:
            stream.write(f"{len(selected)}\n")
            for _, trajectory, ego, reference, perceived, _ in selected:
                e = ego[1]
                points = trajectory[1]["points"]
                stream.write(
                    f"{e['x']} {e['y']} {e['z']} "
                    f"{e['orientation_x']} {e['orientation_y']} "
                    f"{e['orientation_z']} {e['orientation_w']} "
                    f"{e['velocity_mps']} {len(points)}\n")
                for point in points:
                    stream.write(
                        f"{point['x']} {point['y']} {point['z']} "
                        f"{point['orientation_x']} {point['orientation_y']} "
                        f"{point['orientation_z']} {point['orientation_w']} "
                        f"{point['velocity']} {point['acceleration']}\n")
                reference_points = (reference[1]["points"]
                                    if reference else [])
                stream.write(f"{len(reference_points)}\n")
                for point in reference_points:
                    stream.write(
                        f"{point['x']} {point['y']} {point['z']} "
                        f"{point['orientation_x']} {point['orientation_y']} "
                        f"{point['orientation_z']} {point['orientation_w']} "
                        f"{point['velocity']} {point['acceleration']}\n")
                centers = ([item["pose"][:2]
                            for item in perceived[1]["objects"]]
                           if perceived else [])
                stream.write(f"{len(centers)}\n")
                for x, y in centers:
                    stream.write(f"{x} {y}\n")
        process = subprocess.run([str(args.runner), str(source), str(output)],
                                 capture_output=True, text=True)
        if process.returncode:
            raise RuntimeError(f"평가 코어 실행 실패: {process.stderr}")
        rows = [list(map(float, line.split()))
                for line in output.read_text().splitlines()]
    if len(rows) != len(selected):
        raise AssertionError("평가 코어 출력 수가 다릅니다")
    errors = {name: [] for name in NAMES}
    mismatches = []
    optional_mismatches = []
    examples = {name: [] for name in NAMES}
    legacy_maxima = 0
    for index, (actual, (_, _, _, _, _, expected)) in enumerate(
            zip(rows, selected)):
        if len(actual) != len(NAMES) * len(FIELDS):
            raise AssertionError("평가 지표 필드 수가 다릅니다")
        different = False
        optional_different = False
        for number, name in enumerate(NAMES):
            if name not in expected or math.isnan(actual[number * 3]):
                continue
            metric_errors = []
            for field, key in enumerate(FIELDS):
                measured = actual[number * 3 + field]
                original = float(expected[name][key])
                # 실측 DiagnosticArray 버전은 음수만 누적한 경우 max를
                # DBL_MIN으로 기록한다. 현행 소스와 코어는 음수 최대값을 쓴다.
                if (key == "max" and measured < 0 and
                        original == sys.float_info.min):
                    legacy_maxima += 1
                    continue
                metric_errors.append(abs(measured - original))
            errors[name].append(max(metric_errors))
            if name in BASE_NAMES:
                different |= max(metric_errors) > 1e-6
            else:
                optional_different |= max(metric_errors) > 1e-6
            if max(metric_errors) > 1e-6 and len(examples[name]) < 3:
                examples[name].append((index, actual[number * 3:number * 3 + 3],
                                       [expected[name][key] for key in FIELDS]))
        if different:
            mismatches.append(index)
        if optional_different:
            optional_mismatches.append(index)
    print(f"기본 지표 12종, 비교 {len(selected)}프레임, "
          f"1e-6 초과 {len(mismatches)}프레임 {mismatches[:10]}")
    print(f"확장 지표 1e-6 초과 {len(optional_mismatches)}프레임 "
          f"{optional_mismatches[:10]}")
    print(f"구버전 음수 최대값 초기화 차이: {legacy_maxima}개 지표값")
    for name in NAMES:
        values = errors[name]
        if not values:
            print(f"{name}: 유효 지표 없음")
            continue
        print(f"{name}: 중앙 {statistics.median(values):.9g}, "
              f"최대 {max(values):.9g}, 1e-6 이내 "
              f"{sum(value <= 1e-6 for value in values)}/{len(values)}")
        if examples[name]:
            print(f"  예: {examples[name]}")
    if (args.maximum_basic_mismatches is not None and
            len(mismatches) > args.maximum_basic_mismatches):
        raise AssertionError("기본 Planning 지표 불일치 프레임이 기준을 넘었습니다")
    if (args.maximum_extended_mismatches is not None and
            len(optional_mismatches) > args.maximum_extended_mismatches):
        raise AssertionError("확장 Planning 지표 불일치 프레임이 기준을 넘었습니다")


if __name__ == "__main__":
    main()
