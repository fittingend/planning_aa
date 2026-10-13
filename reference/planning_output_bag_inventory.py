"""원본 K-City 10개 rosbag의 기본 Planning 출력 존재 여부를 집계합니다."""

import argparse
import re
import sqlite3
from pathlib import Path


P = "/planning/"
S = P + "scenario_planning/"
B = S + "lane_driving/behavior_planning/"
M = S + "lane_driving/motion_planning/"

OUTPUTS = {
    "mission route": P + "mission_planning/route",
    "mission state": P + "mission_planning/state",
    "mission route marker": P + "mission_planning/route_marker",
    "main route": P + "mission_planning/route_selector/main/route",
    "main state": P + "mission_planning/route_selector/main/state",
    "MRM route": P + "mission_planning/route_selector/mrm/route",
    "MRM state": P + "mission_planning/route_selector/mrm/state",
    "goal echo": P + "mission_planning/echo_back_goal_pose",
    "scenario": S + "scenario",
    "external max velocity": S + "max_velocity",
    "current max velocity": S + "current_max_velocity",
    "behavior path lane ids": B + "path_with_lane_id",
    "turn indicators": P + "turn_indicators_cmd",
    "hazard lights": P + "hazard_lights_cmd",
    "modified goal": S + "modified_goal",
    "reroute availability": B + "behavior_path_planner/output/is_reroute_available",
    "behavior velocity path": B + "path",
    "infrastructure commands": S + "status/infrastructure_commands",
    "smoothed path": M + "path_smoother/path",
    "elastic band trajectory": M + "elastic_band_smoother/output/traj",
    "optimized trajectory": M + "path_optimizer/trajectory",
    "motion velocity trajectory": M + "motion_velocity_planner/trajectory",
    "motion velocity metrics": M + "motion_velocity_planner/metrics",
    "lane driving trajectory": S + "lane_driving/trajectory",
    "parking occupancy": S + "parking/costmap_generator/occupancy_grid",
    "parking grid map": S + "parking/costmap_generator/grid_map",
    "parking trajectory": S + "parking/trajectory",
    "parking complete": S + "parking/is_completed",
    "selected trajectory": S + "scenario_selector/trajectory",
    "velocity smoother trajectory": S + "velocity_smoother/trajectory",
    "distance to stopline": S + "velocity_smoother/distance_to_stopline",
    "validated trajectory": S + "trajectory",
    "validation status": P + "planning_validator/validation_status",
    "evaluator metrics": P + "planning_evaluator/metrics",
    "remaining distance time": P + "mission_remaining_distance_time",
    "stop reasons": S + "status/stop_reasons",
    "velocity limit candidates": S + "max_velocity_candidates",
    "clear velocity limit": S + "clear_velocity_limit",
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_root", type=Path)
    args = parser.parse_args()
    candidates = sorted(bag for bag in args.bag_root.rglob("*.db3")
                        if "work_0714" not in bag.parts)
    bags = []
    excluded = []
    for bag in candidates:
        try:
            with sqlite3.connect(f"file:{bag}?mode=ro", uri=True) as connection:
                connection.execute("SELECT count(*) FROM topics").fetchone()
            bags.append(bag)
        except sqlite3.DatabaseError as error:
            excluded.append((bag, error))
    if not bags:
        raise ValueError("원본 rosbag을 찾지 못했습니다")
    counts = {topic: [] for topic in OUTPUTS.values()}
    kinds = {topic: set() for topic in OUTPUTS.values()}
    qos_profiles = {topic: set() for topic in OUTPUTS.values()}
    for bag in bags:
        with sqlite3.connect(f"file:{bag}?mode=ro", uri=True) as connection:
            rows = connection.execute(
                "SELECT t.name,t.type,t.offered_qos_profiles,count(m.id) FROM topics t "
                "LEFT JOIN messages m ON m.topic_id=t.id GROUP BY t.id")
            found = {name: (kind, qos, count)
                     for name, kind, qos, count in rows}
        for topic in counts:
            kind, qos, count = found.get(topic, ("", "", 0))
            counts[topic].append(count)
            if kind:
                kinds[topic].add(kind)
            if qos:
                reliabilities = re.findall(r"(?m)^\s*reliability:\s*(\d+)", qos)
                durabilities = re.findall(r"(?m)^\s*durability:\s*(\d+)", qos)
                depths = re.findall(r"(?m)^\s*depth:\s*(\d+)", qos)
                for reliability, durability, depth in zip(
                        reliabilities, durabilities, depths):
                    qos_profiles[topic].add((reliability, durability, depth))
    print(f"# K-City 기본 Planning 출력 관측 ({len(bags)}개 원본 기록)\n")
    print("`0개 기록`은 이 기록들에서 메시지가 없다는 뜻이며 노드 비활성의 "
          "증거는 아닙니다. 서비스 호출은 SQLite topic 집계로 확인할 수 없습니다. "
          "endpoint 기준은 `decision_planning_contract_matrix.md`입니다.\n")
    print("QoS는 rosbag metadata의 publisher 제공 프로필이다. "
          "표기의 순서는 reliability/durability/depth이며 "
          "R=reliable, B=best effort, T=transient local, V=volatile이다.\n")
    print("| 출력 | 메시지가 있는 기록 | 총 메시지 | 기록된 ROS 형식 | 기록 QoS |")
    print("| --- | ---: | ---: | --- | --- |")
    for label, topic in OUTPUTS.items():
        observed = sum(count > 0 for count in counts[topic])
        total = sum(counts[topic])
        types = ", ".join(sorted(kinds[topic])) or "없음"
        profiles = ", ".join(sorted(
            f"{ {'1': 'R', '2': 'B'}.get(reliability, reliability)}/"
            f"{ {'1': 'T', '2': 'V'}.get(durability, durability)}/"
            f"{depth}"
            for reliability, durability, depth in qos_profiles[topic])) or "없음"
        print(f"| `{label}` (`{topic}`) | {observed}/{len(bags)} | "
              f"{total} | `{types}` | `{profiles}` |")
    print("\n## 입력 기록\n")
    for bag in bags:
        print(f"- `{bag.relative_to(args.bag_root)}`")
    if excluded:
        print("\n읽을 수 없어 제외한 파일:\n")
        for bag, error in excluded:
            print(f"- `{bag.relative_to(args.bag_root)}`: {error}")


if __name__ == "__main__":
    main()
