"""보유한 K-City 기록에서 출발 자세 후보 표식이 있는 기록을 찾습니다."""

import argparse
from pathlib import Path

from offline_rosbag_cdr import decode_marker_array, messages


TOPIC = ("/planning/scenario_planning/lane_driving/behavior_planning/"
         "behavior_path_planner/debug/start_planner")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    bags = [path for path in sorted(args.root.glob("*/rosbag_*/*.db3"))
            if path.is_file()]
    for bag in bags:
        count = sum(any(marker["namespace"] == "start_pose_candidates_idx"
                        for marker in decode_marker_array(payload))
                    for _, payload in messages(bag, TOPIC))
        print(f"{bag.relative_to(args.root)}: {count}프레임")
    print(f"검사 기록 {len(bags)}건")


if __name__ == "__main__":
    main()
