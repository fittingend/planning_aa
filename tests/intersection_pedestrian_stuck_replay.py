"""뒤쪽 교차로 정체 차량의 결정·정지점·전체 경로를 원본과 비교합니다."""

import sys
from pathlib import Path

from intersection_stuck_map_replay import run_case


def main():
    runner, map_file, *fixtures = map(Path, sys.argv[1:])
    # 원본의 재표본화 경계에서 최대 약 2.3mm의 좌표 차이를 허용합니다.
    for fixture in fixtures:
        run_case(runner, map_file, fixture,
                 expected_lane_id=24739, tolerance=0.003)


if __name__ == "__main__":
    main()
