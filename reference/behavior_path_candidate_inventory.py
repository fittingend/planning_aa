"""K-City rosbag의 행동 경로 후보와 승인 모듈을 ROS 없이 집계합니다."""

import argparse
import bisect
import collections
import re
import sqlite3
from pathlib import Path

from offline_rosbag_cdr import Reader


STATE_TOPIC = (
    "/planning/scenario_planning/lane_driving/behavior_planning/"
    "behavior_path_planner/debug/internal_state"
)


def inspect_bag(path):
    candidate_counts = collections.defaultdict(lambda: [0, 0])
    active_times = collections.defaultdict(list)
    reference_matches = {}
    goal_reference_count = 0
    running_counts = collections.Counter()
    state_count = 0
    with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as database:
        topics = list(database.execute(
            "SELECT id, name FROM topics WHERE name LIKE "
            "'/planning/path_candidate/%' OR name=?", (STATE_TOPIC,)))
        for topic_id, name in topics:
            for timestamp, data in database.execute(
                    "SELECT timestamp, data FROM messages WHERE topic_id=?",
                    (topic_id,)):
                reader = Reader(data)
                if name == STATE_TOPIC:
                    reader.value("i", 4)
                    reader.value("I", 4)
                    state = reader.text()
                    approved = re.search(r"^approved modules\s*:(.*)$", state, re.M)
                    if approved:
                        state_count += 1
                        running_counts.update(re.findall(
                            r"([a-z_]+)\(RUNNING\)", approved.group(1)))
                else:
                    reader.header()
                    count = reader.value("I", 4)
                    module = name.rsplit("/", 1)[-1]
                    candidate_counts[module][0] += 1
                    candidate_counts[module][1] += count > 0
                    if count:
                        active_times[module].append(timestamp)
        for module, times in active_times.items():
            reference = database.execute(
                "SELECT id FROM topics WHERE name=?",
                ("/planning/path_reference/" + module,)).fetchone()
            if reference is None:
                reference_matches[module] = 0
                continue
            recorded = []
            for timestamp, data in database.execute(
                    "SELECT timestamp, data FROM messages WHERE topic_id=? "
                    "ORDER BY timestamp", (reference[0],)):
                reader = Reader(data)
                reader.header()
                if reader.value("I", 4):
                    recorded.append(timestamp)
            paired = 0
            for timestamp in times:
                index = bisect.bisect_left(recorded, timestamp)
                nearest = [abs(recorded[k] - timestamp) for k in
                           (index - 1, index) if 0 <= k < len(recorded)]
                paired += bool(nearest and min(nearest) <= 50_000_000)
            reference_matches[module] = paired
        goal_reference = database.execute(
            "SELECT id FROM topics WHERE name=?",
            ("/planning/path_reference/goal_planner",)).fetchone()
        if goal_reference:
            for (data,) in database.execute(
                    "SELECT data FROM messages WHERE topic_id=?",
                    (goal_reference[0],)):
                reader = Reader(data)
                reader.header()
                goal_reference_count += reader.value("I", 4) > 0
    return (candidate_counts, running_counts, state_count,
            reference_matches, goal_reference_count)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_root", type=Path)
    args = parser.parse_args()
    bags = sorted(path for path in args.bag_root.glob("*/*/*.db3")
                  if path.is_file())
    print(f"실제 rosbag {len(bags)}개")
    print("기록 | 승인 상태 메시지 | 승인 RUNNING 모듈 | 비어 있지 않은 후보 | 50ms 안의 기준 경로 | 목표 기준 경로")
    for bag in bags:
        candidates, running, states, paired, goal_refs = inspect_bag(bag)
        active = [f"{name}:{counts[1]}/{counts[0]}" for name, counts
                  in sorted(candidates.items()) if counts[1]]
        approved = [f"{name}:{count}" for name, count in
                    running.most_common()]
        references = [f"{name}:{count}/{candidates[name][1]}" for
                      name, count in sorted(paired.items())]
        print("/".join(bag.relative_to(args.bag_root).parts[:2]), "|",
              states, "|", ", ".join(approved) or "없음", "|",
              ", ".join(active) or "없음", "|",
              ", ".join(references) or "없음", "|", goal_refs)


if __name__ == "__main__":
    main()
