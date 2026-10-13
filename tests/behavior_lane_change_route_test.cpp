#include "planning/behavior_lane_change_route.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {
void check(bool valid, const char *message) {
  if (!valid) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

planning::LaneChangeCenterline line(
    std::int64_t id, double start, double end) {
  planning::LaneChangeCenterline result;
  result.lane_id = id;
  for (double x = start; x <= end; x += 1.0) {
    planning::TrajectoryPoint point;
    point.x = x;
    result.points.push_back(point);
  }
  return result;
}
}

int main() {
  const std::vector<planning::LaneChangeCenterline> lanes{
      line(11, 0.0, 20.0), line(12, 20.0, 60.0)};
  const auto result = planning::buildLaneChangeMappedReference(
      lanes, {10.2, 0.3}, 20.0, 5.0, 25.0, 5.0);
  check(result && std::abs(result->start_arc_m - 10.2) < 1.0e-9 &&
            std::abs(result->end_arc_m - 55.0) < 1.0e-9 &&
            std::abs(result->path.front().x - 10.2) < 1.0e-9 &&
            std::abs(result->path.back().x - 55.0) < 1.0e-9 &&
            result->lane_ids.front() == std::vector<std::int64_t>{11} &&
            result->lane_ids.back() == std::vector<std::int64_t>{12},
        "두 목표 차로의 연결·시작 투영·종점 여유·차로 ID를 보존합니다");
  const auto goal = planning::buildLaneChangeMappedReference(
      lanes, {10.2, 0.3}, 20.0, 5.0, 25.0, 5.0,
      planning::BehaviorPathShiftAnchor{35.2, 0.0});
  check(goal && std::abs(goal->end_arc_m - 30.2) < 1.0e-9,
        "목표점이 목표 차로에 있으면 해당 목표점 앞에서 자릅니다");
  check(!planning::buildLaneChangeMappedReference(
             lanes, {10.2, 0.3}, 20.0, 5.0, 25.0, 5.0,
             planning::BehaviorPathShiftAnchor{34.0, 0.0}),
        "목표점 앞 공간이 횡이동 길이보다 짧으면 후보를 제외합니다");
  check(!planning::buildLaneChangeMappedReference(
             {line(11, 0.0, 20.0), line(12, 30.0, 50.0)},
             {10.0, 0.0}, 20.0, 5.0, 20.0, 0.0),
        "연결되지 않은 지도 차로를 하나의 경로로 잇지 않습니다");
  const auto regulatory = planning::laneChangeNextRegulatoryDistance(
      {45.0, 20.0, 30.0}, true, false, true);
  check(regulatory && *regulatory == 30.0 &&
            !planning::laneChangeNextRegulatoryDistance(
                {45.0, 20.0, 30.0}, false, false, false),
        "활성 규제 요소 중 가장 가까운 거리를 고릅니다");
}
