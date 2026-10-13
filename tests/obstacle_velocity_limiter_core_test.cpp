// SPDX-License-Identifier: Apache-2.0
// 정적 점·선 장애물의 전방 투영과 감속 상한을 검사합니다.

#include "planning/obstacle_velocity_limiter_core.hpp"

#include <cmath>
#include <stdexcept>

namespace {
void check(bool condition) {
  if (!condition) throw std::runtime_error("장애물 속도 제한 검사 실패");
}
}  // 익명 네임스페이스

int main() {
  planning::Trajectory path(3);
  for (std::size_t i = 0; i < path.size(); ++i) {
    path[i].x = static_cast<double>(i) * 5.0;
    path[i].longitudinal_velocity_mps = 10.0F;
  }
  const std::vector<double> times{0.0, 0.5, 1.0};
  auto plan = planning::planObstacleVelocityLimitParticle(
      path, times, {{10.0, 0.0}}, {}, 10.0);
  check(plan.slowdowns.size() == 3);
  check(std::abs(plan.slowdowns[0].velocity - 10.0) < 1e-12);
  check(std::abs(plan.slowdowns[1].velocity - 9.0) < 1e-12);
  check(std::abs(plan.slowdowns[2].velocity - 8.0) < 1e-12);
  plan = planning::planObstacleVelocityLimitParticle(
      path, times, {{10.0, 2.0}}, {}, 10.0);
  check(plan.slowdowns.empty());
  plan = planning::planObstacleVelocityLimitParticle(
      path, times, {}, {{{8.0, -2.0}, {8.0, 2.0}}}, 0.0);
  check(plan.slowdowns.size() == 2);
  check(std::abs(plan.slowdowns.front().velocity - 4.26) < 1e-12);
  check(std::abs(plan.slowdowns.back().velocity - 2.5) < 1e-12);
  planning::ObstacleVelocityLimiterParameters approximate;
  approximate.distance_method =
      planning::VelocityLimiterDistanceMethod::Approximation;
  const auto exact_off_axis = planning::planObstacleVelocityLimitParticle(
      path, times, {{8.0, 0.8}}, {}, 0.0);
  const auto approximate_off_axis = planning::planObstacleVelocityLimitParticle(
      path, times, {{8.0, 0.8}}, {}, 0.0, approximate);
  check(exact_off_axis.slowdowns.size() ==
        approximate_off_axis.slowdowns.size());
  check(approximate_off_axis.slowdowns.front().velocity >
        exact_off_axis.slowdowns.front().velocity);

  planning::Trajectory long_path(21);
  std::vector<double> long_times(21);
  for (std::size_t i = 0; i < long_path.size(); ++i) {
    long_path[i].x = static_cast<double>(i);
    long_path[i].longitudinal_velocity_mps = 10.0F;
    long_times[i] = static_cast<double>(i) / 10.0;
  }
  planning::ObstacleVelocityLimiterParameters parameters;
  parameters.start_distance = 2.1;
  parameters.maximum_path_length = 4.1;
  parameters.downsample_factor = 2;
  const auto sampled = planning::sampleObstacleVelocityLimiterPath(
      long_path, long_times, 2, parameters);
  check(sampled.path.size() == 3);
  check(sampled.path[0].x == 5.0 && sampled.path[1].x == 7.0 &&
        sampled.path[2].x == 9.0);
  check(std::abs(sampled.time_from_start[0] - 0.5) < 1e-12);

  planning::BicycleVelocityProjectionParameters bicycle;
  bicycle.steering_offset = 0.0;
  const auto straight = planning::bicycleVelocityProjectionLines(
      {0.0, 0.0}, 0.0, 10.0, 0.0, bicycle);
  check(straight.size() == 1 && straight.front().size() == 5);
  check(std::abs(straight.front().back().x - 13.74) < 1e-12);
  check(std::abs(straight.front().back().y) < 1e-12);
  bicycle.steering_offset = 0.01;
  const auto spread = planning::bicycleVelocityProjectionLines(
      {0.0, 0.0}, 0.0, 10.0, 0.0, bicycle);
  check(spread.size() == 3 && spread[0].back().y > 0.0 &&
        spread[2].back().y < 0.0);
  check(std::abs(planning::bicycleProjectionArcDistance(
      {0.0, 0.0}, 0.0, {5.0, 5.0}) -
      5.0 * 3.141592653589793 / 2.0) < 1e-12);
  check(std::abs(planning::bicycleProjectionArcDistance(
      {0.0, 0.0}, 0.0, {10.0, 0.0}) - 10.0) < 1e-12);
  auto bicycle_plan = planning::planObstacleVelocityLimitBicycle(
      path, times, {{10.0, 0.0}}, {}, 10.0);
  check(bicycle_plan.slowdowns.size() == 3);
  check(std::abs(bicycle_plan.slowdowns[1].velocity - 9.0) < 1e-12);
  bicycle_plan = planning::planObstacleVelocityLimitBicycle(
      path, times, {{10.0, 10.0}}, {}, 10.0);
  check(bicycle_plan.slowdowns.empty());
  bicycle_plan = planning::planObstacleVelocityLimitBicycle(
      path, times, {}, {{{8.0, -2.0}, {8.0, 2.0}}}, 0.0);
  check(bicycle_plan.slowdowns.size() == 2);
  check(bicycle_plan.slowdowns.front().velocity > 4.26 &&
        bicycle_plan.slowdowns.front().velocity < 4.5);

  planning::VelocityLimiterObject moving;
  moving.center = {5.0, 0.0};
  moving.length = 2.0;
  moving.width = 2.0;
  moving.velocity_x = 1.0;
  const auto object_masks = planning::velocityLimiterMovingObjectMasks(
      {moving}, 0.0, 0.5);
  check(object_masks.size() == 1);
  check(planning::velocityLimiterMovingObjectMasks(
      {moving}, 0.0, 1.1).empty());
  planning::VelocityLimiterObstacleMasks masks;
  masks.positive = {{0.0, -2.0}, {10.0, -2.0},
                    {10.0, 2.0}, {0.0, 2.0}};
  masks.negative = object_masks;
  const auto filtered = planning::filterVelocityLimiterPointCloud(
      {{2.0, 0.0}, {5.0, 0.0}, {8.0, 0.0}, {11.0, 0.0}}, masks);
  check(filtered.size() == 2);
  check(filtered.front().x == 2.0 && filtered.back().x == 8.0);
  masks.negative.push_back(planning::velocityLimiterTrajectoryMask(path, 0.9));
  check(planning::filterVelocityLimiterPointCloud(
      {{2.0, 0.0}, {2.0, 1.5}}, masks).size() == 1);
  planning::ObstacleVelocityLimiterParameters cloud_parameters;
  cloud_parameters.downsample_factor = 1;
  const auto cloud_plan = planning::planObstacleVelocityLimitPointCloud(
      path, times, 0, 10.0, {{10.0, 0.0}}, {}, cloud_parameters);
  check(cloud_plan.slowdowns.size() == 3);
  masks.negative.clear();
  masks.negative.push_back(planning::velocityLimiterObjectPolygon(moving));
  check(planning::planObstacleVelocityLimitPointCloud(
      path, times, 0, 10.0, {{5.0, 0.0}}, masks,
      cloud_parameters, {}, true).slowdowns.empty());
  check(!planning::planObstacleVelocityLimitPointCloud(
      path, times, 0, 10.0, {{5.0, 0.0}}, masks,
      cloud_parameters).slowdowns.empty());
  planning::Trajectory turning(2);
  turning[0].longitudinal_velocity_mps = 10.0F;
  turning[1].longitudinal_velocity_mps = 10.0F;
  turning[1].x = 5.0;
  turning[1].orientation_z = std::sin(3.141592653589793 / 4.0);
  turning[1].orientation_w = std::cos(3.141592653589793 / 4.0);
  planning::calculateVelocityLimiterSteeringAngles(turning, 2.74);
  check(std::abs(turning[1].front_wheel_angle_rad -
        std::atan2(2.74 * 3.141592653589793 / 2.0, 5.0)) < 1e-6);
}
