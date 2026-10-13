"""보유 K-City 기록 열 개의 validator 상태를 전 구간 비교합니다."""

import argparse
import subprocess
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("build_dir", type=Path)
    parser.add_argument("bag_root", type=Path)
    args = parser.parse_args()
    bags = sorted(bag for group in ("ipc", "hpc")
                  for bag in (args.bag_root / group).glob("rosbag_*/*.db3")
                  if bag.is_file())
    if not bags:
        raise ValueError("K-City rosbag 데이터베이스를 찾을 수 없습니다")
    replay = Path(__file__).with_name("validator_rosbag_offline_replay.py")
    failures = 0
    for bag in bags:
        print(f"{bag.parent.parent.name}/{bag.parent.name}: "
              "전 구간 validator 비교", flush=True)
        command = (sys.executable, str(replay),
                   str(args.build_dir / "validator_reference_runner"),
                   str(bag), "--frames", "100000")
        failures += subprocess.run(command, check=False).returncode != 0
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
