#include "planning/intersection_occlusion_core.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {

void check(bool valid, const char *message) {
  if (!valid) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

}  // 익명 네임스페이스

int main() {
  planning::CrosswalkOccupancyGrid grid;
  grid.width = 100;
  grid.resolution_m = 1.0;
  grid.height = 70;
  grid.values.assign(grid.width * grid.height, 0);
  const std::vector<std::vector<planning::CrosswalkPoint>> area_long{{
      {0.0, 0.0}, {99.0, 0.0}, {99.0, 69.0}, {0.0, 69.0}}};
  const std::vector<std::vector<planning::CrosswalkPoint>> area{{
      {0.0, 0.0}, {99.0, 0.0}, {99.0, 29.0}, {0.0, 29.0}}};
  planning::Trajectory path(2);
  path[0].x = 10.0;
  path[0].y = 10.0;
  path[1].x = 20.0;
  path[1].y = 10.0;
  const auto patch = [&](std::size_t first_column, int value) {
    for (std::size_t row = 8; row < 12; ++row) {
      for (std::size_t col = first_column; col < first_column + 4; ++col) {
        grid.values[row * grid.width + col] = value;
      }
    }
  };
  patch(28, 50);
  check(planning::detectIntersectionOcclusion(grid, area, path),
        "가까운 미확인 영역은 가림으로 판정합니다");
  patch(28, 0);
  patch(80, 50);
  check(!planning::detectIntersectionOcclusion(grid, area, path),
        "경로에서 먼 미확인 영역은 제외합니다");
  check(!planning::detectIntersectionOcclusion(grid, area, {}),
        "유효한 경로가 없으면 가림 정지를 만들지 않습니다");
  const std::vector<std::vector<planning::CrosswalkPoint>> division{{
      {15.0, 10.0}, {15.0, 25.0}, {70.0, 25.0},
      {70.0, 15.0}, {25.0, 15.0}}};
  patch(80, 0);
  for (std::size_t row = 13; row < 17; ++row) {
    for (std::size_t col = 23; col < 27; ++col) {
      grid.values[row * grid.width + col] = 50;
    }
  }
  check(planning::detectIntersectionOcclusion(grid, area, path),
        "평면 거리만 사용하면 돌아오는 차로의 미확인 영역을 감지합니다");
  check(!planning::detectIntersectionOcclusion(grid, area, path, {}, division),
        "차로를 따라 55m 넘게 떨어진 미확인 영역을 제외합니다");
  for (std::size_t row = 13; row < 17; ++row) {
    for (std::size_t col = 23; col < 27; ++col) {
      grid.values[row * grid.width + col] = 0;
    }
  }
  for (std::size_t row = 18; row < 22; ++row) {
    for (std::size_t col = 13; col < 17; ++col) {
      grid.values[row * grid.width + col] = 50;
    }
  }
  check(planning::detectIntersectionOcclusion(grid, area, path, {}, division),
        "교차점에서 차로 방향으로 가까운 미확인 영역을 감지합니다");
  grid.values.assign(grid.width * grid.height, 0);
  for (std::size_t row = 30; row < 34; ++row) {
    for (std::size_t col = 13; col < 17; ++col) {
      grid.values[row * grid.width + col] = 50;
    }
  }
  const std::vector<std::vector<planning::CrosswalkPoint>> straight{
      {{13.5, 10.0}, {13.5, 60.0}},
      {{14.5, 10.0}, {14.5, 60.0}},
      {{15.5, 10.0}, {15.5, 60.0}},
      {{16.5, 10.0}, {16.5, 60.0}}};
  check(planning::detectIntersectionOcclusion(grid, area_long, path, {}, straight),
        "정차 차량이 없는 차로의 미확인 영역을 감지합니다");
  planning::IntersectionOcclusionBlocker blocker;
  blocker.pose = {15.0, 20.0, 1.5707963267948966};
  blocker.length_m = 4.0;
  blocker.width_m = 6.0;
  check(!planning::detectIntersectionOcclusion(
            grid, area_long, path, {}, straight, {blocker}),
        "정차 차량 뒤쪽의 미확인 영역을 제외합니다");
  auto curved = straight;
  for (auto &line : curved) {
    const double x = line.front().x;
    line = {{x, 10.0}, {x, 20.0},
            {x + 5.0, 25.0}, {x, 30.0}, {x, 60.0}};
  }
  check(planning::detectIntersectionOcclusion(
            grid, area_long, path, {}, curved, {blocker}),
        "크게 꺾인 탐색선에서는 정차 차량 차폐를 보수적으로 유지합니다");
  blocker.speed_mps = 1.0;
  check(planning::detectIntersectionOcclusion(
            grid, area_long, path, {}, straight, {blocker}),
        "이동 차량 뒤쪽의 미확인 영역은 유지합니다");
  blocker.speed_mps = 0.0;
  blocker.pose.y = 40.0;
  check(planning::detectIntersectionOcclusion(
            grid, area_long, path, {}, straight, {blocker}),
        "미확인 영역보다 뒤쪽의 정차 차량은 가림을 지우지 않습니다");
  const auto classify = [&](std::vector<planning::IntersectionOcclusionBlocker> vehicles) {
    return planning::classifyIntersectionOcclusion(
        grid, area_long, path, {10.0, 10.0}, {}, straight, vehicles);
  };
  check(classify({}) == planning::IntersectionOcclusionStatus::Static,
        "가시 영역에 객체가 없으면 정적 가림입니다");
  planning::IntersectionOcclusionBlocker visible_vehicle;
  visible_vehicle.pose = {12.0, 22.0, 0.0};
  visible_vehicle.length_m = 2.0;
  visible_vehicle.width_m = 2.0;
  visible_vehicle.speed_mps = 1.0;
  check(classify({visible_vehicle}) ==
            planning::IntersectionOcclusionStatus::Dynamic,
        "가시 영역의 이동 차량은 동적 가림입니다");
  visible_vehicle.pose.x = 22.0;
  check(classify({visible_vehicle}) ==
            planning::IntersectionOcclusionStatus::Static,
        "가시 영역 밖의 이동 차량은 정적 가림을 바꾸지 않습니다");
  grid.values.assign(grid.width * grid.height, 0);
  check(classify({visible_vehicle}) ==
            planning::IntersectionOcclusionStatus::Clear,
        "미확인 영역이 없으면 가림이 없습니다");
  planning::IntersectionOcclusionReleaseState release;
  using Status = planning::IntersectionOcclusionStatus;
  check(!release.update(Status::Static, true, false, false, true, 0.0),
        "기본 정지선을 거치기 전에는 정적 가림을 해제하지 않습니다");
  check(!release.update(Status::Dynamic, true, false, true, true, 0.1),
        "동적 가림은 정적 가림 해제 시간을 시작하지 않습니다");
  check(!release.update(Status::Static, true, false, true, true, 0.2),
        "정적 가림을 만났을 때 해제 시간을 시작합니다");
  check(!release.update(Status::Static, true, false, true, true, 0.7),
        "원본과 같이 0.5초 경계에서는 계속 정지합니다");
  check(release.update(Status::Static, true, false, true, true, 0.71),
        "정적 가림 정차 시간이 0.5초를 넘으면 해제합니다");
  check(!release.update(Status::Static, true, true, true, true, 0.8),
        "충돌이 생기면 정적 가림 해제를 초기화합니다");
  check(!release.update(Status::Static, true, false, true, true, 0.9),
        "충돌 해소 뒤 정적 가림 해제 시간을 다시 시작합니다");
  check(!release.update(Status::Static, false, false, true, true, 2.0),
        "신호등 없는 교차로에는 정적 가림 시간 해제를 적용하지 않습니다");
  bool rejected = false;
  try {
    planning::detectIntersectionOcclusion(grid, area, path, {0.0, 53.0});
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, "잘못된 가림 크기 설정을 거부합니다");
}
