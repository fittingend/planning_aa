"""차선 변경 디버그의 충돌 다각형과 판정 문자열을 직접 대조합니다."""

import argparse
import collections
import math
from pathlib import Path

from offline_rosbag_cdr import decode_marker_array, messages


TOPIC = ("/planning/scenario_planning/lane_driving/behavior_planning/"
         "behavior_path_planner/debug/lane_change_left")


def intersects(first, second):
    """볼록 다각형에 분리축이 없으면 경계 접촉도 겹침으로 셉니다."""
    if len(first) < 3 or len(second) < 3:
        return False
    for polygon in (first, second):
        for start, end in zip(polygon, polygon[1:] + polygon[:1]):
            axis = (end[1] - start[1], start[0] - end[0])
            if abs(axis[0]) + abs(axis[1]) < 1e-12:
                continue
            first_projection = [x * axis[0] + y * axis[1]
                                for x, y, _ in first]
            second_projection = [x * axis[0] + y * axis[1]
                                 for x, y, _ in second]
            if (max(first_projection) < min(second_projection) - 1e-8 or
                    max(second_projection) < min(first_projection) - 1e-8):
                return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    args = parser.parse_args()
    counts = collections.Counter()
    pose_gaps = []
    for _, payload in messages(args.bag, TOPIC):
        markers = decode_marker_array(payload)
        statuses = [marker for marker in markers
                    if marker["namespace"] == "collision_check_object_info"]
        if not statuses:
            continue
        polygons = [marker["points"] for marker in markers
                    if marker["namespace"] ==
                    "ego_and_target_polygon_relation"]
        red_poses = [marker["pose"] for marker in markers
                     if marker["namespace"] == "ego_predicted_path" and
                     marker["color"][0] > 0.9 and
                     marker["color"][1] < 0.1]
        counts["판정 수"] += len(statuses)
        if len(statuses) != 1 or len(polygons) != 2 or len(red_poses) != 2:
            counts["형식 불일치"] += 1
            continue
        reason = next((line.split(":", 1)[1].strip()
                       for line in statuses[0]["text"].splitlines()
                       if line.startswith("Unsafe reason:")), "없음")
        counts[reason] += 1
        counts["다각형 겹침" if intersects(*polygons)
               else "다각형 이격"] += 1
        for polygon, pose in zip(polygons, red_poses):
            if abs(pose[0]) + abs(pose[1]) < 1e-9:
                counts["기대 자세 미기록"] += 1
                continue
            centroid = (sum(point[0] for point in polygon[:-1]) /
                        max(1, len(polygon) - 1),
                        sum(point[1] for point in polygon[:-1]) /
                        max(1, len(polygon) - 1))
            pose_gaps.append(math.hypot(centroid[0] - pose[0],
                                        centroid[1] - pose[1]))
    print("원본 다각형 대조:", dict(counts))
    if pose_gaps:
        pose_gaps.sort()
        print(f"다각형 중심과 기대 자세 간격: 중앙값 "
              f"{pose_gaps[len(pose_gaps) // 2]:.3f}m, "
              f"최대 {pose_gaps[-1]:.3f}m")


if __name__ == "__main__":
    main()
