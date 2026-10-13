"""원본 ROS 진단의 이름·심각도·문구를 독립 코어에 대조합니다."""

import json
import subprocess
import sys
from pathlib import Path


def main():
    runner, fixture = map(Path, sys.argv[1:])
    expected = json.loads(fixture.read_text())
    process = subprocess.run(
        [str(runner)], capture_output=True, text=True, check=True)
    actual = {}
    for line in process.stdout.splitlines():
        case, name, level, message = line.split("\t")
        actual.setdefault(case, []).append((name, int(level), message))
    for record in expected:
        expected_items = [
            (item["name"].removeprefix("planning_validator: "),
             item["level"], item["message"])
            for item in record["diagnostics"]
        ]
        assert actual[record["case"]] == expected_items, record["case"]
    print("원본 validator 진단 3상태·42항목 일치")


if __name__ == "__main__":
    main()
