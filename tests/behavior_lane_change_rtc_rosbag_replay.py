"""K-City RTC 협력 상태를 독립 C++ 채널 조립기에 재생합니다."""

import argparse
import collections
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import decode_cooperate_status_array, messages


def expected(statuses):
    approved = False
    forced = False
    deactivated = False
    for status in statuses:
        state = status["state"]
        command = status["command_activated"]
        safe = status["safe"]
        requested = status["requested"]
        automatic = status["auto_mode"]
        approved |= state not in (3, 4) and (
            safe if automatic and not requested else command)
        forced |= state in (0, 1) and command and (not safe or requested)
        deactivated |= state == 1 and not command and safe and not automatic
    return int(approved), int(forced), int(deactivated)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    args = parser.parse_args()
    frames = []
    for side in ("lane_change_left", "lane_change_right"):
        topic = "/planning/cooperate_status/" + side
        for receipt, raw in messages(args.bag, topic):
            frames.append((receipt, side,
                           decode_cooperate_status_array(raw)["statuses"]))
    frames.sort(key=lambda item: item[0])
    if not frames:
        raise ValueError("차로 변경 RTC 협력 상태가 없습니다")
    rows = [str(len(frames))]
    for _, _, statuses in frames:
        rows.append(str(len(statuses)))
        rows.extend(" ".join(str(int(value)) for value in (
            status["safe"], status["requested"],
            status["command_activated"], status["auto_mode"],
            status["state"])) for status in statuses)
    result = subprocess.run([str(args.runner)], input="\n".join(rows) + "\n",
                            text=True, capture_output=True, check=True)
    outputs = [tuple(map(int, line.split()))
               for line in result.stdout.splitlines()]
    if len(outputs) != len(frames):
        raise ValueError("RTC 출력 프레임 수가 다릅니다")
    mismatches = [index for index, ((_, _, statuses), output) in
                  enumerate(zip(frames, outputs))
                  if expected(statuses) != output]
    status_counts = collections.Counter(
        (status["state"], status["safe"],
         status["command_activated"])
        for _, _, statuses in frames for status in statuses)
    decisions = collections.Counter(outputs)
    print(f"RTC 상태 배열 {len(frames)}개, 비어 있지 않은 배열 "
          f"{sum(bool(statuses) for _, _, statuses in frames)}개, "
          f"상태 {sum(len(statuses) for _, _, statuses in frames)}개")
    print(f"독립 RTC 신호 분포 {dict(sorted(decisions.items()))}")
    print(f"기록된 상태 분포 {dict(sorted(status_counts.items()))}")
    print(f"원본 규칙과 신호 불일치 {len(mismatches)}개")
    if mismatches:
        raise AssertionError(f"첫 RTC 신호 불일치: {mismatches[:5]}")


if __name__ == "__main__":
    main()
