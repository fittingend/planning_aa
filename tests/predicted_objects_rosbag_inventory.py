"""K-City 기록의 예측 객체 CDR 메시지를 전부 검사하고 수를 집계합니다."""

import argparse
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import decode_predicted_objects, messages

TOPIC = "/perception/object_recognition/objects"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+", type=Path)
    args = parser.parse_args()
    for bag in args.bags:
        count = nonempty = object_count = path_count = 0
        labels = Counter()
        for _, payload in messages(bag, TOPIC):
            decoded = decode_predicted_objects(payload)
            count += 1
            nonempty += bool(decoded["objects"])
            object_count += len(decoded["objects"])
            for item in decoded["objects"]:
                path_count += len(item["predicted_paths"])
                if item["classification"]:
                    labels[item["classification"][0][0]] += 1
        print(f"{bag.parent.name}: 메시지 {count}, 객체 있는 메시지 {nonempty}, "
              f"객체 {object_count}, 예측 경로 {path_count}, 분류 {dict(labels)}")


if __name__ == "__main__":
    main()
