"""수정한 Lanelet2 이진 자료를 기록된 LaneletMapBin 메시지에 다시 넣습니다."""

import argparse
from pathlib import Path

from autoware_map_msgs.msg import LaneletMapBin
from rclpy.serialization import deserialize_message, serialize_message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--original", type=Path, required=True)
    parser.add_argument("--inner", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    message = deserialize_message(args.original.read_bytes(), LaneletMapBin)
    message.data = list(args.inner.read_bytes())
    args.output.write_bytes(serialize_message(message))


if __name__ == "__main__":
    main()
