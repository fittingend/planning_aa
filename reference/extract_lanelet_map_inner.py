"""rosbag의 LaneletMapBin에서 Lanelet2 내부 직렬화 자료를 추출합니다."""

import argparse
from pathlib import Path

from offline_rosbag_cdr import decode_lanelet_map, messages


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    record = next(messages(args.bag, "/map/vector_map"), None)
    if record is None:
        raise ValueError("rosbag에 Lanelet2 지도 메시지가 없습니다")
    data = decode_lanelet_map(record[1])
    args.output.write_bytes(data)
    print(f"이진 지도 내부 자료 {len(data)}바이트 추출")


if __name__ == "__main__":
    main()
