"""보유 K-City 기록 열 개의 스무더 재생을 같은 조건으로 실행합니다."""

import argparse
import sqlite3
import subprocess
import sys
from pathlib import Path


INPUT_TOPIC = "/planning/scenario_planning/scenario_selector/trajectory"


def input_count(bag):
    with sqlite3.connect(f"file:{bag}?mode=ro", uri=True) as connection:
        return connection.execute(
            "SELECT COUNT(*) FROM messages "
            "JOIN topics ON messages.topic_id = topics.id "
            "WHERE topics.name = ?", (INPUT_TOPIC,)).fetchone()[0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("build_dir", type=Path)
    parser.add_argument("bag_root", type=Path)
    parser.add_argument("--start", type=int, default=25)
    args = parser.parse_args()
    if args.start < 0:
        raise ValueError("시작 입력 인덱스는 음수가 될 수 없습니다")
    bags = sorted(bag for group in ("ipc", "hpc")
                  for bag in (args.bag_root / group).glob("rosbag_*/*.db3")
                  if bag.is_file())
    if not bags:
        raise ValueError("K-City rosbag 데이터베이스를 찾을 수 없습니다")
    replay = Path(__file__).with_name("smoother_rosbag_long_replay.py")
    failures = 0
    for bag in bags:
        count = input_count(bag)
        if count <= args.start:
            print(f"{bag.parent.parent.name}/{bag.parent.name}: 입력 부족 ({count}개)")
            failures += 1
            continue
        print(f"{bag.parent.parent.name}/{bag.parent.name}: "
              f"{count - args.start}프레임 재생", flush=True)
        command = (
            sys.executable, str(replay),
            str(args.build_dir / "smoother_preprocess_runner"),
            str(args.build_dir / "smoother_reference_runner"),
            str(bag), "--start", str(args.start),
            "--frames", str(count - args.start),
            "--stateful-external-limit")
        failures += subprocess.run(command, check=False).returncode != 0
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
