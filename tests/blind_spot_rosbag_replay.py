"""K-City 경로·객체·자차 기록으로 사각지대 지도 코어를 독립 재생합니다."""

import argparse
import bisect
import sqlite3
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))
from offline_rosbag_cdr import (decode_lanelet_map, decode_odometry, decode_path,
                                decode_predicted_objects)


PATH_TOPIC = (
    "/planning/scenario_planning/lane_driving/behavior_planning/"
    "path_with_lane_id"
)
OBJECT_TOPIC = "/perception/object_recognition/objects"
ODOMETRY_TOPIC = "/localization/kinematic_state"


def messages(database, name):
    row = database.execute("SELECT id FROM topics WHERE name=?", (name,)).fetchone()
    if row is None:
        return []
    return list(database.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? "
        "ORDER BY timestamp", (row[0],)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runner", type=Path)
    parser.add_argument("map", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frames", type=int, default=10000)
    args = parser.parse_args()
    with sqlite3.connect(f"file:{args.bag}?mode=ro", uri=True) as database:
        paths = messages(database, PATH_TOPIC)
        objects = messages(database, OBJECT_TOPIC)
        odometry = messages(database, ODOMETRY_TOPIC)
        map_messages = messages(database, "/map/vector_map")
    if not paths or not odometry:
        raise ValueError("경로 또는 자차 위치 메시지가 없습니다")
    object_times = [stamp for stamp, _ in objects]
    odometry_times = [stamp for stamp, _ in odometry]
    previous_object_index = -2
    decoded_objects = []
    sent = 0
    with tempfile.TemporaryDirectory() as directory, tempfile.TemporaryFile(
            mode="w+t", encoding="utf-8") as stream:
        map_path = args.map
        if str(map_path) == "-":
            if not map_messages:
                raise ValueError("기록에 벡터 지도가 없습니다")
            map_path = Path(directory) / "기록지도.bin"
            map_path.write_bytes(decode_lanelet_map(map_messages[0][1]))
        for stamp, path_data in paths[:args.frames]:
            odometry_index = bisect.bisect_right(odometry_times, stamp) - 1
            if odometry_index < 0:
                continue
            object_index = bisect.bisect_right(object_times, stamp) - 1
            if object_index != previous_object_index:
                decoded_objects = []
                if object_index >= 0:
                    for obj in decode_predicted_objects(
                            objects[object_index][1])["objects"]:
                        if not obj["classification"]:
                            continue
                        target = max(obj["classification"], key=lambda item: item[1])
                        if target[0] not in (5, 6, 7):
                            continue
                        decoded_objects.append((obj["uuid"], target[0],
                                                obj["pose"][0], obj["pose"][1],
                                                obj["twist"][0]))
                previous_object_index = object_index
            ego = decode_odometry(odometry[odometry_index][1])
            path = decode_path(path_data, with_lane_ids=True)
            if len(path["points"]) < 2:
                continue
            stream.write(f"F {stamp * 1e-9:.9f} {ego['x']:.17g} "
                         f"{ego['y']:.17g} {ego['velocity_mps']:.17g} "
                         f"{len(path['points'])} {len(decoded_objects)}\n")
            for point, lanes in zip(path["points"], path["lane_ids"]):
                pose = " ".join(f"{value:.17g}" for value in point[:7])
                lane_text = " ".join(str(lane) for lane in lanes)
                stream.write(f"P {pose} {point[7]:.17g} {len(lanes)} "
                             f"{lane_text}\n")
            for obj in decoded_objects:
                stream.write("O " + " ".join(str(value) for value in obj) + "\n")
            sent += 1
        stream.flush()
        stream.seek(0)
        result = subprocess.run(
            [str(args.runner), str(map_path)], stdin=stream,
            text=True, capture_output=True)
        if result.returncode:
            raise RuntimeError(result.stderr or
                               f"사각지대 재생 실행기 종료 코드 {result.returncode}")
    decisions = unsafe = active_frames = 0
    for line in result.stdout.splitlines():
        _, count, stopped = (int(value) for value in line.split())
        decisions += count
        unsafe += stopped
        active_frames += stopped > 0
    print(f"전달 프레임 {sent}, 판단 {decisions}, 정지 판단 "
          f"{unsafe}, 정지 프레임 {active_frames}")


if __name__ == "__main__":
    main()
