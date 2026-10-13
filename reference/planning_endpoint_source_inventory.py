"""기준 Decision Planning 생성 코드의 정적 입출력 목록을 재생성합니다.

source-api-evidence.json은 ROS graph가 아니므로 이 결과는 동적 plugin/RTC endpoint의
실제 생성 여부와 remap 최종 이름을 증명하지 않습니다.
"""

import argparse
import json
import re
from collections import defaultdict
from pathlib import Path


PACKAGES = {
    "autoware_behavior_path_planner": "behavior_path_planner",
    "autoware_behavior_velocity_planner": "behavior_velocity_planner",
    "autoware_costmap_generator": "costmap_generator",
    "autoware_external_velocity_limit_selector": "external_velocity_limit_selector",
    "autoware_freespace_planner": "freespace_planner",
    "autoware_motion_velocity_planner_node_universe": "motion_velocity_planner",
    "autoware_obstacle_cruise_planner": "obstacle_cruise_planner",
    "autoware_path_optimizer": "path_optimizer",
    "autoware_path_smoother": "elastic_band_smoother",
    "autoware_planning_evaluator": "planning_evaluator",
    "autoware_planning_validator": "planning_validator",
    "autoware_remaining_distance_time_calculator": "remaining_distance_time_calculator",
    "autoware_scenario_selector": "scenario_selector",
    "autoware_velocity_smoother": "velocity_smoother",
}
MISSION_FILES = {
    "default_planner.cpp": "mission_planner",
    "goal_pose_visualizer.cpp": "goal_pose_visualizer",
    "mission_planner.cpp": "mission_planner",
    "route_selector.cpp": "route_selector",
}
AUXILIARY_FILES = {
    "calculation_time_plotter.py", "trajectory_visualizer.py",
    "motion_evaluator_node.cpp", "invalid_trajectory_publisher.cpp",
}
# The decision launch selects pid_base rather than optimization_base.
OPTIONAL_FILES = {"optimization_based_planner.cpp"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--input-output", type=Path)
    args = parser.parse_args()
    entries = json.loads(args.source.read_text())
    grouped = defaultdict(list)
    inputs = defaultdict(list)
    optional = defaultdict(list)
    for item in entries:
        if item["api"] not in {
                "create_publisher", "create_service", "create_client",
                "create_subscription", "InterProcessPollingSubscriber"}:
            continue
        path = Path(item["source"])
        if "test" in path.parts or path.name.startswith("test_"):
            continue
        if path.name in AUXILIARY_FILES:
            continue
        snippet = " ".join(item["snippet"].split())
        if item["api"] == "create_subscription" and not re.search(
                r"\bcreate_subscription\s*(?:<|\()", snippet):
            continue
        if item["api"] == "InterProcessPollingSubscriber" and \
                "::SharedPtr" in snippet:
            continue
        package = item["package"]
        if package == "autoware_mission_planner_universe":
            node = MISSION_FILES.get(path.name)
        else:
            node = PACKAGES.get(package)
        if node is None:
            continue
        literal = re.search(r'"([^"\n]+)"', snippet)
        endpoint = literal.group(1) if literal else "동적 이름: " + snippet[:105]
        location = f"{path.name}:{item['line']}"
        if path.name in OPTIONAL_FILES:
            optional[node].append((item["api"], endpoint, location))
            continue
        if item["api"] in {"create_publisher", "create_service"}:
            grouped[node].append((item["api"], endpoint, location))
        else:
            inputs[node].append((item["api"], endpoint, location))
    expected = set(PACKAGES.values()) | set(MISSION_FILES.values())
    if set(grouped) != expected:
        raise RuntimeError(f"노드 누락/추가: {sorted(expected ^ set(grouped))}")
    publishers = sum(api == "create_publisher" for values in grouped.values()
                     for api, _, _ in values)
    services = sum(api == "create_service" for values in grouped.values()
                   for api, _, _ in values)
    if services != 13:
        raise RuntimeError(f"예상 서비스 생성 코드 13개와 다름: {services}")
    lines = ["# 기준 Planning 정적 출력·서비스 생성 코드 목록", "",
             f"근거: `{args.source.name}`. 기본 Planning 계산 노드 {len(grouped)}개, "
             f"publisher 생성 코드 {publishers}개, service 생성 코드 {services}개. "
             "테스트·별도 보조 노드·선택되지 않은 알고리즘은 제외했다. "
             "동적 이름·플러그인/RTC·실행 graph·최종 remap은 "
             "별도 검증 대상이다.", "",
             "| 노드 | API | 코드의 topic/service 식 | 근거 파일:줄 |",
             "| --- | --- | --- | --- |"]
    for node, values in sorted(grouped.items()):
        for api, endpoint, location in values:
            lines.append(f"| {node} | {api} | `{endpoint}` | `{location}` |")
    if optional:
        lines += ["", "## 기준 launch에서 선택하지 않은 알고리즘", "",
                  "`obstacle_cruise_planner`는 `pid_base`를 사용한다. 아래 "
                  "`optimization_base` 내부 생성 위치는 활성 출력 개수에 "
                  "포함하지 않았다.", "",
                  "| 노드 | API | 코드의 topic/service 식 | 근거 파일:줄 |",
                  "| --- | --- | --- | --- |"]
        for node, values in sorted(optional.items()):
            for api, endpoint, location in values:
                lines.append(f"| {node} | {api} | `{endpoint}` | `{location}` |")
    args.output.write_text("\n".join(lines) + "\n")
    print(f"{len(grouped)} nodes, {publishers} publishers, {services} services")
    if args.input_output:
        subscriptions = sum(api != "create_client" for values in inputs.values()
                            for api, _, _ in values)
        clients = sum(api == "create_client" for values in inputs.values()
                      for api, _, _ in values)
        input_lines = ["# 기준 Planning 정적 입력 구독·서비스 client 생성 코드 목록", "",
                       f"근거: `{args.source.name}`. 활성 Planning 노드 "
                       f"{len(inputs)}개에서 구독 생성 코드 {subscriptions}개, "
                       f"서비스 client 생성 코드 {clients}개. 테스트·별도 보조 "
                       "노드·선택되지 않은 알고리즘·선언만 있는 항목은 제외했다. "
                       "polling 구독은 일반 callback과 별도 표기했다. 런타임 "
                       "plugin 구독, 동적 이름, 최종 remap·QoS·실제 graph는 "
                       "이 정적 목록만으로 확정되지 않는다.", "",
                       "| 노드 | API | 코드의 입력/service 식 | 근거 파일:줄 |",
                       "| --- | --- | --- | --- |"]
        for node, values in sorted(inputs.items()):
            for api, endpoint, location in values:
                input_lines.append(
                    f"| {node} | {api} | `{endpoint}` | `{location}` |")
        args.input_output.write_text("\n".join(input_lines) + "\n")
        print(f"{subscriptions} subscriptions, {clients} clients")


if __name__ == "__main__":
    main()
