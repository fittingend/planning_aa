#include "planning/occlusion_spot_core.hpp"

#include <cstdlib>
#include <cmath>
#include <iostream>

namespace {
void check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}
}  // 익명 네임스페이스

int main() {
  planning::Trajectory path;
  for (int index = 0; index <= 40; ++index) {
    planning::TrajectoryPoint point;
    point.x = index;
    point.longitudinal_velocity_mps = 8.0F;
    path.push_back(point);
  }
  planning::Trajectory sparse;
  for (int index = 0; index <= 3; ++index) {
    planning::TrajectoryPoint point;
    point.x = index * 2.0;
    point.y = index == 0 ? 0.0 :
        index == 1 ? 0.0 : 2.0;
    point.longitudinal_velocity_mps =
        static_cast<float>(8 - 2 * index);
    sparse.push_back(point);
  }
  const auto interpolated = planning::resampleOcclusionDetectionPath(
      sparse, 100.0);
  check(interpolated.size() > sparse.size() &&
            interpolated.front().x == sparse.front().x &&
            std::abs(interpolated.back().x - sparse.back().x) < 1.0e-9 &&
            std::abs(interpolated[1].y + 0.0883883476483) < 1.0e-6 &&
            interpolated.back().longitudinal_velocity_mps ==
                sparse.back().longitudinal_velocity_mps &&
            interpolated[1].longitudinal_velocity_mps == 8.0F,
        "가림 감지 경로를 1m 간격으로 보간하고 이전점 속도를 유지합니다");
  const auto clipped = planning::resampleOcclusionDetectionPath(
      sparse, 3.0);
  check(!clipped.empty() && clipped.back().x <= 2.0 + 1.0e-9,
        "감지 길이를 넘는 원본 점은 보간 전에 제외합니다");
  planning::CrosswalkOccupancyGrid grid;
  grid.width = 50;
  grid.height = 20;
  grid.values.assign(grid.width * grid.height, 0);
  for (std::size_t y = 3; y < 6; ++y)
    for (std::size_t x = 18; x < 21; ++x)
      grid.values[y * grid.width + x] = 50;
  const auto occluded = planning::planOcclusionSpot(
      path, {0.0, 0.0}, 8.0, 0.0, grid, {});
  check(!occluded.collision_arcs_m.empty() &&
            occluded.trajectory.back().longitudinal_velocity_mps < 8.0F,
        "미확인 영역에 접근하면 가림 감속을 적용합니다");
  planning::OcclusionSpotParameters erosion_parameters;
  erosion_parameters.minimum_occlusion_size_m = 3.0;
  const auto eroded_patch = planning::planOcclusionSpot(
      path, {0.0, 0.0}, 8.0, 0.0, grid, {}, erosion_parameters);
  check(!eroded_patch.collision_arcs_m.empty(),
        "2x2 침식을 두 번 적용해 3x3 미확인 영역을 유지합니다");
  grid.values.assign(grid.width * grid.height, 0);
  grid.values[4 * grid.width + 19] = 50;
  const auto eroded_single = planning::planOcclusionSpot(
      path, {0.0, 0.0}, 8.0, 0.0, grid, {}, erosion_parameters);
  check(eroded_single.collision_arcs_m.empty(),
        "크기 기준에 못 미치는 미확인 셀은 제거합니다");
  grid.values[2 * grid.width + 19] = 100;
  const auto blocked_by_grid = planning::planOcclusionSpot(
      path, {0.0, 0.0}, 8.0, 0.0, grid, {});
  check(blocked_by_grid.collision_arcs_m.empty(),
        "보행자 경로를 막는 점유 셀이 있으면 가림 후보를 제외합니다");
  grid.values[2 * grid.width + 19] = 0;
  planning::CrosswalkOcclusionMaskObject parked_barrier;
  parked_barrier.pose = {19.5, 2.5, 0.0};
  parked_barrier.length_m = 2.0;
  parked_barrier.width_m = 1.0;
  const auto blocked_by_vehicle = planning::planOcclusionSpot(
      path, {0.0, 0.0}, 8.0, 0.0, grid, {parked_barrier});
  check(blocked_by_vehicle.collision_arcs_m.empty(),
        "보행자 경로를 막는 정차 차량도 가림 후보를 제외합니다");
  planning::CrosswalkOccupancyGrid rotated_grid = grid;
  rotated_grid.values.assign(grid.values.size(), 0);
  rotated_grid.origin_x = 30.0;
  rotated_grid.origin_y = -18.0;
  rotated_grid.origin_yaw_rad = std::acos(-1.0) / 2.0;
  for (std::size_t y = 10; y < 13; ++y)
    for (std::size_t x = 20; x < 23; ++x)
      rotated_grid.values[y * grid.width + x] = 50;
  const auto rotated_occlusion = planning::planOcclusionSpot(
      path, {0.0, 0.0}, 8.0, 0.0, rotated_grid, {});
  check(!rotated_occlusion.collision_arcs_m.empty(),
        "회전한 격자 원점에서도 미확인 영역을 경로에 투영합니다");
  for (std::size_t y = 10; y < 13; ++y)
    rotated_grid.values[y * grid.width + 19] = 100;
  const auto rotated_blocked = planning::planOcclusionSpot(
      path, {0.0, 0.0}, 8.0, 0.0, rotated_grid, {});
  check(rotated_blocked.collision_arcs_m.empty(),
        "회전한 격자의 점유 셀도 보행자 통로를 막습니다");
  for (std::size_t y = 3; y < 6; ++y)
    for (std::size_t x = 18; x < 21; ++x)
      grid.values[y * grid.width + x] = 50;
  planning::CrosswalkOcclusionMaskObject moving;
  moving.pose = {19.5, 4.5, 0.0};
  moving.length_m = 4.0;
  moving.width_m = 4.0;
  moving.longitudinal_speed_mps = 2.0;
  const auto masked = planning::planOcclusionSpot(
      path, {0.0, 0.0}, 8.0, 0.0, grid, {moving});
  check(masked.collision_arcs_m.empty(),
        "이동 객체가 가린 격자는 감속 후보에서 제외합니다");
  planning::OcclusionSpotParameters no_ray_parameters;
  no_ray_parameters.use_moving_vehicle_ray_mask = false;
  const auto moving_without_ray = planning::planOcclusionSpot(
      path, {0.0, 0.0}, 8.0, 0.0, grid, {moving}, no_ray_parameters);
  check(!moving_without_ray.collision_arcs_m.empty(),
        "이동 차량 광선 마스크를 끄면 해당 미확인 격자를 유지합니다");
  planning::CrosswalkOccupancyGrid shadow_grid;
  shadow_grid.width = 50;
  shadow_grid.height = 10;
  shadow_grid.values.assign(shadow_grid.width * shadow_grid.height, 0);
  for (std::size_t y = 3; y < 6; ++y)
    for (std::size_t x = 14; x < 18; ++x)
      shadow_grid.values[y * shadow_grid.width + x] = 50;
  moving.pose = {20.5, 4.5, 0.0};
  moving.length_m = 2.0;
  moving.width_m = 2.0;
  const auto shadowed = planning::planOcclusionSpot(
      path, {0.0, 0.0}, 8.0, 0.0, shadow_grid, {moving});
  check(shadowed.collision_arcs_m.empty(),
        "이동 객체 뒤쪽 광선 영역의 가림 후보를 제거합니다");
  planning::OcclusionSpotParameters object_parameters;
  object_parameters.method =
      planning::OcclusionSpotDetectionMethod::StoppedObjects;
  moving.longitudinal_speed_mps = 0.0;
  const auto stopped = planning::planOcclusionSpot(
      path, {0.0, 0.0}, 8.0, 0.0, std::nullopt, {moving},
      object_parameters);
  check(!stopped.collision_arcs_m.empty(),
        "정차 객체 입력에서도 가림 감속 후보를 생성합니다");
  const planning::VelocityLimiterSegment wall{{18.0, 2.0},
                                               {23.0, 2.0}};
  const auto divided = planning::planOcclusionSpot(
      path, {0.0, 0.0}, 8.0, 0.0, std::nullopt, {moving},
      object_parameters, {wall});
  check(divided.collision_arcs_m.empty(),
        "차로 분할선이 가린 객체는 감속 후보에서 제외합니다");
  auto extended_path = path;
  for (int index = 41; index <= 80; ++index) {
    auto point = path.back();
    point.x = index;
    extended_path.push_back(point);
  }
  moving.pose = {60.0, 3.0, 0.0};
  const auto distant_object = planning::planOcclusionSpot(
      extended_path, {0.0, 0.0}, 8.0, 0.0, std::nullopt, {moving},
      object_parameters);
  check(!distant_object.collision_arcs_m.empty(),
        "정차 객체 방식은 전방 100m 감지 길이를 사용합니다");
}
