#include "planning/crosswalk_occlusion.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {
void check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}
}

int main() {
  planning::CrosswalkOccupancyGrid grid;
  grid.width = 20;
  grid.height = 20;
  grid.resolution_m = 1.0;
  grid.origin_y = -10.0;
  grid.values.assign(grid.width * grid.height, 0);
  const std::vector<std::vector<planning::CrosswalkPoint>> areas{
      {{9.0, -1.0}, {12.0, -1.0}, {12.0, 2.0}, {9.0, 2.0}}};
  grid.values[10 * grid.width + 10] = 50;
  check(planning::isCrosswalkOccluded(grid, areas),
        "감지 영역의 미확인 점유 셀을 가림으로 판정합니다");
  planning::CrosswalkOcclusionParameters two_meter;
  two_meter.minimum_size_m = 2.0;
  check(!planning::isCrosswalkOccluded(grid, areas, two_meter),
        "설정한 크기보다 작은 가림 영역은 제외합니다");
  for (std::size_t row : {10U, 11U}) {
    for (std::size_t col : {10U, 11U}) {
      grid.values[row * grid.width + col] = 50;
    }
  }
  check(planning::isCrosswalkOccluded(grid, areas, two_meter),
        "설정한 크기의 가림 영역을 찾습니다");
  grid.values[10 * grid.width + 10] = 100;
  check(!planning::isCrosswalkOccluded(grid, areas, two_meter),
        "점유 셀을 가림 영역으로 세지 않습니다");

  planning::CrosswalkOccupancyGrid shadow_grid;
  shadow_grid.width = shadow_grid.height = 20;
  shadow_grid.values.assign(400, 0);
  shadow_grid.values[10 * 20 + 15] = 50;
  const std::vector<std::vector<planning::CrosswalkPoint>> shadow_area{
      {{15.0, 10.0}, {16.0, 10.0}, {16.0, 11.0}, {15.0, 11.0}}};
  const planning::CrosswalkOcclusionMaskObject moving_object{
      {12.0, 10.0, 0.0}, 2.0, 2.0, 1.0, 0.5};
  check(planning::isCrosswalkOccluded(shadow_grid, shadow_area),
        "객체 차폐를 적용하기 전 가림 셀을 찾습니다");
  check(!planning::isCrosswalkOccluded(
            shadow_grid, shadow_area, {}, {moving_object}),
        "이동 객체 뒤의 가림 셀을 제거합니다");
  auto stopped_object = moving_object;
  stopped_object.longitudinal_speed_mps = 0.1;
  check(planning::isCrosswalkOccluded(
            shadow_grid, shadow_area, {}, {stopped_object}),
        "속도 기준 미만 객체 뒤의 가림은 유지합니다");

  const auto detection = planning::crosswalkDetectionAreas(
      {{10.0, -2.0}, {10.0, 2.0}},
      {{12.0, -2.0}, {12.0, 2.0}}, {11.0, 0.0}, 5.0);
  check(detection.size() == 3 &&
            std::abs(detection[1][1].y + 5.0) < 1e-9 &&
            std::abs(detection[2][1].y - 5.0) < 1e-9,
        "횡단보도 양쪽의 감지 영역을 연장합니다");
  check(std::abs(planning::crosswalkDetectionRange(1.0, 10.0, 2.0) - 5.0) < 1e-9,
        "차량 도착 시각으로 가림 감지 거리를 계산합니다");

  planning::CrosswalkOcclusionPlanner planner;
  check(!planner.update(0.0, true, true, false, false, false),
        "가림이 시작된 직후에는 감속하지 않습니다");
  check(planner.update(0.5, true, true, false, false, false),
        "가림이 지속되면 감속합니다");
  check(planner.update(0.8, false, true, false, false, false),
        "가림이 사라져도 유지 시간 동안 감속합니다");
  check(!planner.update(1.1, false, true, false, false, false),
        "유지 시간이 지나면 감속을 해제합니다");
  check(!planner.update(2.0, true, true, true, false, false),
        "신호등 횡단보도의 가림 감속을 건너뜁니다");

  planning::CrosswalkOcclusionParameters slow_parameters;
  slow_parameters.vehicle_front_m = 1.0;
  planning::CrosswalkOcclusionPlanner slow_planner(slow_parameters);
  planning::Trajectory path(21);
  for (std::size_t i = 0; i < path.size(); ++i) {
    path[i].x = static_cast<double>(i);
    path[i].longitudinal_velocity_mps = 5.0F;
  }
  const auto slowed = slow_planner.applySlowdown(path, 10.0, 12.0, 0.0, 5.0, 0.0);
  check(slowed[6].longitudinal_velocity_mps == 5.0F &&
            slowed[7].longitudinal_velocity_mps < 5.0F,
        "횡단보도 앞 감속점부터 속도를 제한합니다");
  slow_planner.applySlowdown(path, 10.0, 12.0, 8.0, 5.0, 0.0);
  const auto held = slow_planner.applySlowdown(path, 10.0, 12.0, 9.0, 5.0, 0.0);
  check(held.front().longitudinal_velocity_mps < 5.0F,
        "감속점을 지난 뒤 횡단보도 끝까지 속도를 제한합니다");
}
