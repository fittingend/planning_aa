"""K-City 행동 경로 후보와 기록된 모듈 기준 경로의 형상을 집계합니다."""

import argparse
import bisect
import collections
import math
import sqlite3
import statistics
from pathlib import Path

from offline_rosbag_cdr import decode_path


MODULES = ("lane_change_left", "start_planner",
           "static_obstacle_avoidance")


def messages(database, topic):
    row = database.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
    if row is None:
        return []
    return list(database.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (row[0],)))


def length(points):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(points, points[1:]))


def distance_to_path(point, reference):
    result = math.inf
    for first, last in zip(reference, reference[1:]):
        dx, dy = last[0] - first[0], last[1] - first[1]
        norm = dx * dx + dy * dy
        fraction = max(0.0, min(1.0, ((point[0] - first[0]) * dx +
                                  (point[1] - first[1]) * dy) / norm)) if norm else 0.0
        result = min(result, math.hypot(point[0] - first[0] - fraction * dx,
                                       point[1] - first[1] - fraction * dy))
    return result


def inspect(database, module):
    candidates = messages(database, "/planning/path_candidate/" + module)
    references = messages(database, "/planning/path_reference/" + module)
    reference_times = [time for time, _ in references]
    metrics = []
    active = 0
    for time, raw in candidates:
        candidate = decode_path(raw)["points"]
        if not candidate:
            continue
        active += 1
        index = bisect.bisect_left(reference_times, time)
        choices = [offset for offset in (index - 1, index)
                   if 0 <= offset < len(references) and
                   abs(reference_times[offset] - time) <= 50_000_000]
        if not choices:
            continue
        nearest = min(choices, key=lambda offset:
                      abs(reference_times[offset] - time))
        reference = decode_path(references[nearest][1])["points"]
        if len(reference) < 2:
            continue
        distances = [distance_to_path(point, reference)
                     for point in candidate]
        metrics.append((len(candidate), length(candidate),
                        len(reference), length(reference),
                        math.hypot(candidate[0][0] - reference[0][0],
                                   candidate[0][1] - reference[0][1]),
                        max(distances),
                        distance_to_path(candidate[-1], reference)))
    return active, metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_root", type=Path)
    args = parser.parse_args()
    totals = collections.defaultdict(lambda: [0, []])
    print("기록 | 모듈 | 활성 후보 | 50ms 기준 경로 대응 | 후보 점 수 중앙값 | 후보 길이 중앙값 | 기준 경로 점 수 중앙값 | 기준 경로 길이 중앙값 | 시작점 차이 중앙값 | 후보 끝점과 기준 경로 차이 중앙값 | 최대 선분 거리 중앙값/최대")
    for bag in sorted(args.bag_root.glob("*/*/*.db3")):
        if not bag.is_file():
            continue
        with sqlite3.connect(f"file:{bag}?mode=ro", uri=True) as database:
            for module in MODULES:
                active, metrics = inspect(database, module)
                if not active:
                    continue
                totals[module][0] += active
                totals[module][1].extend(metrics)
                values = [statistics.median(row[index] for row in metrics)
                          for index in range(7)] if metrics else [0] * 7
                maximum = max((row[5] for row in metrics), default=0.0)
                print(f"{'/'.join(bag.relative_to(args.bag_root).parts[:2])} | "
                      f"{module} | {active} | {len(metrics)} | "
                      f"{values[0]:.0f} | {values[1]:.2f}m | "
                      f"{values[2]:.0f} | {values[3]:.2f}m | "
                      f"{values[4]:.3f}m | {values[6]:.3f}m | "
                      f"{values[5]:.3f}/{maximum:.3f}m")
    for module, (active, metrics) in sorted(totals.items()):
        maximums = [row[5] for row in metrics]
        print(f"합계 {module}: 활성 {active}, 대응 {len(metrics)}, "
              f"최대 선분 거리 중앙값 {statistics.median(maximums):.3f}m, "
              f"최대값 {max(maximums):.3f}m")


if __name__ == "__main__":
    main()
