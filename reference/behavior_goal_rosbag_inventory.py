"""K-City 경로 종점과 기록된 경로 목표의 위치·속도를 집계합니다."""

import argparse
import math
import sqlite3
from pathlib import Path

from offline_rosbag_cdr import Reader, decode_path


ROUTE = "/planning/mission_planning/route"
PATH = ("/planning/scenario_planning/lane_driving/behavior_planning/"
        "path_with_lane_id")


def inspect_bag(path):
    with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as database:
        topics = dict(database.execute("SELECT name, id FROM topics"))
        route = database.execute(
            "SELECT data FROM messages WHERE topic_id=? "
            "ORDER BY timestamp LIMIT 1", (topics[ROUTE],)).fetchone()
        reader = Reader(route[0])
        reader.header()
        for _ in range(7):
            reader.value("d", 8)
        goal = tuple(reader.value("d", 8) for _ in range(7))
        frames = near_goal = stopped = both = 0
        maximum_error = 0.0
        for (data,) in database.execute(
                "SELECT data FROM messages WHERE topic_id=?",
                (topics[PATH],)):
            decoded = decode_path(data, with_lane_ids=True)
            if not decoded["points"]:
                continue
            endpoint = decoded["points"][-1]
            error = math.hypot(endpoint[0] - goal[0],
                               endpoint[1] - goal[1])
            close = error <= 0.1
            zero = abs(endpoint[7]) < 0.01
            frames += 1
            near_goal += close
            stopped += zero
            both += close and zero
            maximum_error = max(maximum_error, error)
    return frames, near_goal, stopped, both, maximum_error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_root", type=Path)
    args = parser.parse_args()
    total = [0, 0, 0, 0]
    print("기록 | 경로 | 종점 0.1m 안 | 종점 영속도 | 둘 다 | 최대 종점 거리")
    for bag in sorted(args.bag_root.glob("*/*/*.db3")):
        if not bag.is_file():
            continue
        frames, near_goal, stopped, both, maximum_error = inspect_bag(bag)
        for index, value in enumerate((frames, near_goal, stopped, both)):
            total[index] += value
        name = "/".join(bag.relative_to(args.bag_root).parts[:2])
        print(f"{name} | {frames} | {near_goal} | {stopped} | "
              f"{both} | {maximum_error:.3f}m")
    print(f"합계 | {total[0]} | {total[1]} | {total[2]} | "
          f"{total[3]} | -")


if __name__ == "__main__":
    main()
