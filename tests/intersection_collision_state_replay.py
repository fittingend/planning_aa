"""원본 교차로의 충돌 해제 유지 시간을 연속 입력으로 비교합니다."""

import json
import subprocess
import sys
from pathlib import Path


def main():
    runner, fixture = map(Path, sys.argv[1:])
    data = json.loads(fixture.read_text())
    times = data["intersection_decision_times_s"]
    decisions = data["intersection_decisions"]
    object_frames = data["synthetic_car"]["object_frames"]
    assert len(times) == len(decisions) == 8
    payload = [len(times)]
    for index, stamp in enumerate(times):
        payload.extend((stamp, int(index < object_frames)))
    process = subprocess.run([str(runner)], input=" ".join(map(str, payload)),
                             text=True, capture_output=True, check=True)
    actual = [bool(int(line)) for line in process.stdout.splitlines()]
    expected = [line.splitlines()[1] == "NonOccludedCollisionStop:"
                for line in decisions]
    print(f"원본 충돌 유지 {expected}, 코어 충돌 유지 {actual}")
    assert actual == expected


if __name__ == "__main__":
    main()
