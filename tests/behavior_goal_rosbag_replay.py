"""기록된 목표 기준 경로의 종점을 독립 계산하고 최종 행동 경로와 대조합니다."""

import argparse
import bisect
import math
import sqlite3
import statistics
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import (decode_lanelet_map, decode_lanelet_route,
                                decode_path)


REFERENCE = "/planning/path_reference/goal_planner"
FINAL = ("/planning/scenario_planning/lane_driving/behavior_planning/"
         "path_with_lane_id")


def messages(database, topic):
    row = database.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
    if row is None:
        return []
    return list(database.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (row[0],)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frames", type=int, default=10000)
    parser.add_argument("--radius", type=float, default=7.5)
    parser.add_argument("--preferred-only", action="store_true")
    parser.add_argument("--diagnose", action="store_true")
    args = parser.parse_args()
    with sqlite3.connect(f"file:{args.bag}?mode=ro", uri=True) as database:
        references = messages(database, REFERENCE)[:args.frames]
        finals = messages(database, FINAL)
        routes = messages(database, "/planning/mission_planning/route")
        maps = messages(database, "/map/vector_map")
    if not references or not finals or not routes or not maps:
        raise ValueError("목표 기준 경로·최종 경로·경로 목표 중 일부가 없습니다")
    route = decode_lanelet_route(routes[0][1])
    goal = route["goal"]
    yaw = math.atan2(2 * (goal[6] * goal[5] + goal[3] * goal[4]),
                     1 - 2 * (goal[4] * goal[4] + goal[5] * goal[5]))
    goal_lane_id = route["segments"][-1]["preferred"][0]
    route_lane_ids = (sorted({segment["preferred"][0]
                              for segment in route["segments"]})
                      if args.preferred_only else
                      sorted({primitive[0] for segment in route["segments"]
                              for primitive in segment["primitives"]}))
    final_times = [time for time, _ in finals]
    frames = []
    for time, raw in references:
        reference = decode_path(raw)
        if len(reference["points"]) < 2:
            continue
        index = bisect.bisect_left(final_times, time)
        choices = [offset for offset in (index - 1, index)
                   if 0 <= offset < len(finals) and
                   abs(final_times[offset] - time) <= 50_000_000]
        if not choices:
            continue
        chosen = min(choices, key=lambda offset:
                     abs(final_times[offset] - time))
        final = decode_path(finals[chosen][1], with_lane_ids=True)
        if not final["points"]:
            continue
        frames.append((time, reference, final,
                       (final_times[chosen] - time) * 1e-6))
    with tempfile.TemporaryDirectory() as directory:
        map_file = Path(directory) / "지도.bin"
        source = Path(directory) / "입력.txt"
        result = Path(directory) / "결과.txt"
        map_file.write_bytes(decode_lanelet_map(maps[0][1]))
        with source.open("w") as stream:
            stream.write(f"{len(frames)} {goal[0]:.17g} {goal[1]:.17g} "
                         f"{yaw:.17g} {goal_lane_id} {args.radius:.17g} "
                         f"{len(route_lane_ids)} " +
                         " ".join(map(str, route_lane_ids)) + "\n")
            for _, reference, _, _ in frames:
                stream.write(f"{len(reference['points'])}\n")
                for point in reference["points"]:
                    stream.write(" ".join(f"{value:.17g}" for value in
                                          point[:10]) + "\n")
        subprocess.run([str(args.runner), str(map_file),
                        str(source), str(result)],
                       check=True)
        lines = iter(result.read_text().splitlines())
        outputs = []
        for _ in frames:
            count = int(next(lines))
            outputs.append([tuple(map(float, next(lines).split()))
                            for _ in range(count)])
    counts_equal = 0
    endpoint_errors = []
    pregoal_errors = []
    point_errors = []
    velocity_errors = []
    mismatches = []
    for index, ((_, _, final, lag_ms), output) in enumerate(zip(frames, outputs)):
        target = final["points"]
        counts_equal += len(output) == len(target)
        endpoint_errors.append(math.hypot(output[-1][0] - target[-1][0],
                                          output[-1][1] - target[-1][1]))
        velocity_errors.append(abs(output[-1][3] - target[-1][7]))
        if len(output) > 1 and len(target) > 1:
            pregoal_errors.append(math.hypot(output[-2][0] - target[-2][0],
                                             output[-2][1] - target[-2][1]))
        if len(output) == len(target):
            point_errors.append(max(math.hypot(a[0] - b[0], a[1] - b[1])
                                    for a, b in zip(output, target)))
        if len(output) != len(target) or endpoint_errors[-1] > 0.01:
            mismatches.append((index, len(output), len(target),
                               round(endpoint_errors[-1], 4), round(lag_ms, 2)))
    print(f"목표 기준 경로 {len(references)}개, 50ms 최종 경로 대응 "
          f"{len(frames)}개, 점 수 일치 {counts_equal}개")
    if endpoint_errors:
        print(f"종점 위치 차이 중앙값 {statistics.median(endpoint_errors):.6f}m, "
              f"최대 {max(endpoint_errors):.6f}m, "
              f"0.01m 이내 {sum(error <= 0.01 for error in endpoint_errors)}개")
        print(f"끝점 속도 차이 최대 {max(velocity_errors):.6f}m/s")
    if pregoal_errors:
        print(f"목표 직전 점 위치 차이 중앙값 "
              f"{statistics.median(pregoal_errors):.6f}m, "
              f"최대 {max(pregoal_errors):.6f}m")
    if point_errors:
        print(f"점 수 일치 경로의 최대 인덱스 위치 차이 중앙값 "
              f"{statistics.median(point_errors):.6f}m, "
              f"최대 {max(point_errors):.6f}m")
    if args.diagnose:
        print(f"점 수 또는 종점 불일치 첫 사례 {mismatches[:20]}")


if __name__ == "__main__":
    main()
