"""K-City rosbag의 행동 속도 요인 배열 길이와 상태를 집계합니다."""

import argparse
import collections
import sqlite3
from pathlib import Path

from offline_rosbag_cdr import Reader, decode_velocity_factors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_root", type=Path)
    args = parser.parse_args()
    totals = collections.defaultdict(
        lambda: [0, 0, 0, collections.Counter()])
    bags = sorted(bag for bag in args.bag_root.glob("*/*/*.db3")
                  if bag.is_file())
    for bag in bags:
        with sqlite3.connect(f"file:{bag}?mode=ro", uri=True) as database:
            topics = list(database.execute(
                "SELECT id, name, type FROM topics "
                "WHERE name LIKE '/planning/velocity_factors/%'"))
            for topic_id, topic_name, topic_type in topics:
                if topic_type != "autoware_adapi_v1_msgs/msg/VelocityFactorArray":
                    continue
                module = topic_name.rsplit("/", 1)[-1]
                for (payload,) in database.execute(
                        "SELECT data FROM messages WHERE topic_id=?", (topic_id,)):
                    reader = Reader(payload)
                    reader.header()
                    count = reader.value("I", 4)
                    totals[module][0] += 1
                    totals[module][1] += count > 0
                    totals[module][2] += count
                    if count:
                        totals[module][3].update(
                            factor["status"] for factor in
                            decode_velocity_factors(payload)["factors"])
    print(f"실제 rosbag {len(bags)}개")
    print("모듈 | 메시지 | 비어 있지 않은 메시지 | 요인 | 상태 0/1/2")
    for module, (messages, active, factors, status) in sorted(
            totals.items(), key=lambda item: (-item[1][1], item[0])):
        print(f"{module} | {messages} | {active} | {factors} | "
              f"{status[0]}/{status[1]}/{status[2]}")


if __name__ == "__main__":
    main()
