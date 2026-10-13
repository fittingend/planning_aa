"""원본 계획 플러그인의 필수 입력이 K-City 기록에 남았는지 검사합니다."""

import argparse
import sqlite3
from pathlib import Path


TOPICS = (
    "/perception/obstacle_segmentation/pointcloud",
    "/awapi/tmp/virtual_traffic_light_states",
    "/perception/virtual_traffic_light_states",
    "/parameter_events",
    "/perception/object_recognition/objects",
    "/perception/occupancy_grid_map/map",
)


def inspect(bag):
    with sqlite3.connect(f"file:{bag}?mode=ro", uri=True) as database:
        rows = database.execute(
            "SELECT topics.name, COUNT(messages.id) FROM topics "
            "LEFT JOIN messages ON topics.id=messages.topic_id "
            "WHERE topics.name IN (" + ",".join("?" for _ in TOPICS) + ") "
            "GROUP BY topics.id", TOPICS).fetchall()
    observed = dict(rows)
    return [observed.get(topic, 0) for topic in TOPICS]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    bags = sorted((*args.root.glob("hpc/rosbag*/rosbag*.db3"),
                   *args.root.glob("ipc/rosbag*/rosbag*.db3")))
    bags = [bag for bag in bags if bag.is_file()]
    if not bags:
        raise ValueError("검사할 원본 rosbag db3가 없습니다")
    totals = [0] * len(TOPICS)
    print("기록 | 점군 | 가상 신호 A | 가상 신호 B | 매개변수 | 객체 | 점유 격자")
    for bag in bags:
        counts = inspect(bag)
        totals = [left + right for left, right in zip(totals, counts)]
        print(f"{bag.parent.parent.name}/{bag.parent.name} | " +
              " | ".join(map(str, counts)))
    print("합계 | " + " | ".join(map(str, totals)))
    if len(bags) == 10 and totals[:4] == [0, 0, 0, 0]:
        print("원본 점군·가상 신호·매개변수 기록이 없어 전체 입력 재생은 불가능합니다")


if __name__ == "__main__":
    main()
