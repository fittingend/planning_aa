"""실제 K-City 점군의 XYZ 통계를 독립 PointCloud2 해석기와 비교합니다."""

import argparse
import math
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import decode_pointcloud2_frame, messages

TOPIC = ("/perception/object_recognition/detection/"
         "pointcloud_map_filtered/pointcloud")


def python_statistics(frame):
    fields = {field["name"]: field for field in frame["fields"]}
    endian = ">" if frame["big_endian"] else "<"
    result = [0, 0.0, 0.0, 0.0]
    for row in range(frame["height"]):
        for column in range(frame["width"]):
            base = row * frame["row_step"] + column * frame["point_step"]
            values = []
            for name in ("x", "y", "z"):
                field = fields[name]
                code = "f" if field["datatype"] == 7 else "d"
                values.append(struct.unpack_from(
                    endian + code, frame["data"],
                    base + field["offset"])[0])
            if not all(math.isfinite(value) for value in values):
                continue
            result[0] += 1
            for index, value in enumerate(values, start=1):
                result[index] += value
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frames", type=int, default=20)
    args = parser.parse_args()
    frames = []
    for _, payload in messages(args.bag, TOPIC):
        frames.append(decode_pointcloud2_frame(payload))
        if len(frames) >= args.frames:
            break
    if not frames:
        print(f"{args.bag}: 비교할 점군 메시지가 없습니다")
        return
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory) / "점군입력.bin"
        target = Path(directory) / "점군출력.txt"
        with source.open("wb") as stream:
            stream.write(struct.pack("<I", len(frames)))
            for frame in frames:
                fields = {field["name"]: field for field in frame["fields"]}
                stream.write(struct.pack("<IIIII", frame["width"],
                                         frame["height"], frame["point_step"],
                                         frame["row_step"], frame["big_endian"]))
                for name in ("x", "y", "z"):
                    field = fields[name]
                    stream.write(struct.pack("<II", field["offset"],
                                             field["datatype"]))
                stream.write(struct.pack("<I", len(frame["data"])))
                stream.write(frame["data"])
        subprocess.run([str(args.runner), str(source), str(target)], check=True)
        independent = [tuple(map(float, line.split()))
                       for line in target.read_text().splitlines()]
    assert len(independent) == len(frames)
    maximum_error = 0.0
    count = 0
    for frame, row in zip(frames, independent):
        reference = python_statistics(frame)
        if int(row[0]) != reference[0]:
            raise AssertionError("점군 유효 점 수가 일치하지 않습니다")
        count += reference[0]
        maximum_error = max(maximum_error, *(
            abs(a - b) for a, b in zip(reference[1:], row[1:])))
    if maximum_error > 1e-6:
        raise AssertionError(f"점군 좌표 합계 차이가 큽니다: {maximum_error}")
    print(f"{args.bag}: 실제 점군 {len(frames)}메시지, "
          f"유효 점 {count}, XYZ 합계 최대 차이 "
          f"{maximum_error:.9f}m")


if __name__ == "__main__":
    main()
