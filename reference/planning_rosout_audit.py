"""Inspect planning node logs saved in a ROS 2 sqlite bag."""

import argparse
from collections import Counter
from pathlib import Path
import sqlite3

from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--pattern", default="obstacle_cruise")
    parser.add_argument("--limit", type=int, default=40)
    args = parser.parse_args()
    connection = sqlite3.connect(f"file:{args.bag}?mode=ro", uri=True)
    topics = dict(connection.execute("SELECT name, id FROM topics"))
    if "/rosout" not in topics:
        raise ValueError("bag에 /rosout topic이 없습니다")
    counts = Counter()
    selected = []
    for stamp, payload in connection.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topics["/rosout"],)):
        message = deserialize_message(payload, Log)
        counts[message.name] += 1
        if args.pattern.lower() in (message.name + " " + message.msg).lower():
            selected.append((stamp, message.name, message.level, message.msg))
    print("messages", sum(counts.values()), "nodes", len(counts))
    print("matching", len(selected))
    for item in selected[:args.limit]:
        print(*item, sep=" | ")
    if len(selected) > args.limit:
        print("remaining", len(selected) - args.limit)


if __name__ == "__main__":
    main()
