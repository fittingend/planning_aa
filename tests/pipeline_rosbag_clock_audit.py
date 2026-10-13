"""기록된 계획 단계의 헤더 시각과 수신 시각을 비교합니다."""

import argparse
import bisect
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import Reader, decode_path, decode_trajectory, messages


PREFIX = "/planning/scenario_planning/lane_driving/"
STAGES = (
    ("행동 경로", PREFIX + "behavior_planning/path_with_lane_id",
     lambda payload: decode_path(payload, with_lane_ids=True)),
    ("행동 속도 후 경로", PREFIX + "behavior_planning/path", decode_path),
    ("탄성 평활화", PREFIX + "motion_planning/elastic_band_smoother/output/traj",
     decode_trajectory),
    ("평활화 경로", PREFIX + "motion_planning/path_smoother/path", decode_path),
    ("경로 최적화", PREFIX + "motion_planning/path_optimizer/trajectory",
     decode_trajectory),
    ("동작 속도 후 궤적", PREFIX + "motion_planning/motion_velocity_planner/trajectory",
     decode_trajectory),
    ("최종 차로 궤적", PREFIX + "trajectory", decode_trajectory),
)
VALIDATOR = "/planning/planning_validator/validation_status"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--assert-ipc-stopline-reference", action="store_true",
                        help="IPC 정지선 기록의 공통 헤더 시각을 검사합니다")
    args = parser.parse_args()
    if not args.bag.is_file():
        parser.error("rosbag db3 파일을 지정해야 합니다")
    clocks = []
    first_stage_records = []
    downstream_outputs = {}
    for label, topic, decoder in STAGES:
        by_stamp = {}
        total = 0
        for receipt_ns, payload in messages(args.bag, topic):
            item = decoder(payload)
            total += 1
            by_stamp[item["stamp_ns"]] = receipt_ns
            if len(clocks) in (4, 5, 6):
                downstream_outputs.setdefault(len(clocks), {})[
                    item["stamp_ns"]] = tuple(
                        (point["x"], point["y"], point["velocity"])
                        for point in item["points"])
            if len(clocks) < 2:
                points = item["points"]
                endpoints = ((points[0][0], points[0][1]),
                             (points[-1][0], points[-1][1])) if points else None
                first_stage_records.append((len(clocks), receipt_ns, endpoints))
        clocks.append(by_stamp)
        print(f"{label}: 수신 {total}, 고유 헤더 시각 {len(by_stamp)}")
    shared = set.intersection(*(set(clock) for clock in clocks))
    downstream_shared = set.intersection(*(set(clock) for clock in clocks[2:]))
    print(f"모든 단계에 공통인 헤더 시각: {len(shared)}")
    print(f"탄성 평활화 이후 공통 헤더 시각: {len(downstream_shared)}")
    identical_later = []
    for before, after in ((4, 5), (5, 6)):
        common = set(downstream_outputs[before]) & set(downstream_outputs[after])
        identical = sum(downstream_outputs[before][stamp] ==
                        downstream_outputs[after][stamp] for stamp in common)
        identical_later.append(identical)
        print(f"{STAGES[before][0]} → {STAGES[after][0]}: "
              f"점 수·위치·속도 동일 {identical}/{len(common)}")
    first = [record for record in first_stage_records if record[0] == 0]
    second = [record for record in first_stage_records if record[0] == 1]
    pair_delays = [(later[1] - earlier[1]) / 1e6
                   for earlier, later in zip(first, second)]
    endpoint_errors = [max(math.hypot(x1 - x2, y1 - y2)
                           for (x1, y1), (x2, y2) in
                           zip(earlier[2], later[2]))
                       for earlier, later in zip(first, second)
                       if earlier[2] and later[2]]
    print(f"행동 경로 단순 순서 짝: {len(pair_delays)}쌍, "
          f"끝점 최대 차이 {max(endpoint_errors, default=math.inf):.6f}m, "
          f"수신 간격 중앙값 {statistics.median(pair_delays):.3f}ms, "
          f"범위 {min(pair_delays):.3f}~{max(pair_delays):.3f}ms")
    second_times = [record[1] for record in second]
    used = set()
    clock_geometry_pairs = []
    for earlier in first:
        if not earlier[2]:
            continue
        start = bisect.bisect_left(second_times, earlier[1] - 20_000_000)
        choices = []
        for index in range(start, len(second)):
            later = second[index]
            if later[1] > earlier[1] + 100_000_000:
                break
            if index in used or not later[2]:
                continue
            error = max(math.hypot(x1 - x2, y1 - y2)
                        for (x1, y1), (x2, y2) in
                        zip(earlier[2], later[2]))
            if error <= 1e-3:
                delay_ms = (later[1] - earlier[1]) / 1e6
                choices.append((abs(delay_ms - 20.0), index, delay_ms))
        if choices:
            _, index, delay_ms = min(choices)
            used.add(index)
            clock_geometry_pairs.append(delay_ms)
    print(f"행동 경로 시각·끝점 짝: {len(clock_geometry_pairs)}쌍, "
          f"음수 수신 간격 {sum(delay < 0 for delay in clock_geometry_pairs)}개")
    negative_counts = []
    for index in range(len(clocks) - 1):
        pair_stamps = set(clocks[index]) & set(clocks[index + 1])
        print(f"{STAGES[index][0]} → {STAGES[index + 1][0]}: "
              f"공통 헤더 시각 {len(pair_stamps)}개")
        delays_ms = [(clocks[index + 1][stamp] - clocks[index][stamp]) / 1e6
                     for stamp in pair_stamps]
        if not delays_ms:
            continue
        negative = sum(delay < 0 for delay in delays_ms)
        negative_counts.append(negative)
        print(f"{STAGES[index][0]} → {STAGES[index + 1][0]}: "
              f"중앙 수신 간격 {statistics.median(delays_ms):.3f}ms, "
              f"범위 {min(delays_ms):.3f}~{max(delays_ms):.3f}ms, "
              f"뒤 단계가 먼저 기록된 시각 {negative}개")
    validation = []
    for receipt_ns, payload in messages(args.bag, VALIDATOR):
        reader = Reader(payload)
        stamp_ns = reader.value("i", 4) * 1_000_000_000 + reader.value("I", 4)
        validation.append((receipt_ns, stamp_ns))
    validation_receipts = [receipt for receipt, _ in validation]
    matched_validation = 0
    for final_receipt in clocks[-1].values():
        index = bisect.bisect_left(validation_receipts, final_receipt)
        if (index < len(validation) and
                validation[index][0] - final_receipt <= 100_000_000):
            matched_validation += 1
    common_validator_stamps = len(set(clocks[-1]) &
                                  {stamp for _, stamp in validation})
    print(f"검증 상태: 수신 {len(validation)}, 최종 궤적과 공통 시각 "
          f"{common_validator_stamps}, 뒤 100ms 이내의 단순 수신 짝 "
          f"{matched_validation}")
    if args.assert_ipc_stopline_reference:
        if (len(shared) != 0 or len(downstream_shared) != 571 or
                len(clocks) != 7 or len(pair_delays) != 576 or
                max(endpoint_errors, default=math.inf) > 1e-6 or
                min(pair_delays) <= 0 or
                len(clock_geometry_pairs) != 576 or
                common_validator_stamps != 0 or
                matched_validation < 560 or
                identical_later != [575, 575]):
            raise AssertionError(
                "IPC 정지선 공통 헤더 시각 기준 불일치: "
                f"전체 {len(shared)}, 탄성 평활화 이후 "
                f"{len(downstream_shared)}")
        if len(negative_counts) < 4:
            raise AssertionError("단계별 수신 순서를 비교할 수 없습니다")


if __name__ == "__main__":
    main()
