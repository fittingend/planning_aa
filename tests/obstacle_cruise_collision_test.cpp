// SPDX-License-Identifier: Apache-2.0
// 예측 차량 외형과 한 단계 자차 영역의 실제 교차를 검사합니다.

#include "planning/obstacle_cruise_collision.hpp"

#include <cmath>
#include <stdexcept>

int main() {
  const planning::CruisePredictedPath bent{
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
      1.0, 1.0};
  const auto interpolated = planning::resampleCruisePredictedPath(
      bent, 0.5, 2.0);
  if (interpolated.size() != 5 ||
      std::abs(interpolated[1].pose.x - 0.6875) > 1e-12 ||
      std::abs(interpolated.back().pose.x) > 1e-12)
    throw std::runtime_error("순항 객체 예측 스플라인이 잘못되었습니다");
  planning::Trajectory path(5);
  for (std::size_t i = 0; i < path.size(); ++i) {
    path[i].x = static_cast<double>(i) * 2.0;
    path[i].longitudinal_velocity_mps = 4.0;
  }
  const auto resampled = planning::resampleCruiseCollisionTrajectory(
      path, 1, 2.0, 4.0);
  if (resampled.size() != 7 || resampled.front().x != 2.0 ||
      resampled[3].x != 8.0 || resampled.back().x != 12.0)
    throw std::runtime_error("순항 충돌 경로의 축소 또는 연장이 잘못되었습니다");
  const planning::CruiseObjectShape shape{0, 2.0, 2.0, {}};
  if (planning::cruiseObjectLateralDistance(
          path, {4.0, 1.2, 0.0}, shape, 1.0, 1.0, 2.0) != 0.0 ||
      planning::cruiseObjectLateralDistance(
          path, {4.0, 5.0, 0.0}, shape, 1.0, 1.0, 2.0) < 2.9)
    throw std::runtime_error("순항 객체 정밀 횡거리가 잘못되었습니다");
  if (planning::cruiseObjectLateralDistance(
          path, {0.0, 2.4, 0.0}, shape, 1.0, 1.0, 2.0) < 0.39 ||
      planning::cruiseObjectLateralDistance(
          path, {0.0, 2.4, 0.0}, shape, 1.0, 1.0, 2.0,
          planning::CruisePredictedPose{0.0, 0.6, 0.0}) != 0.0)
    throw std::runtime_error("현재 자차 자세 오차가 충돌 외형에 반영되지 않았습니다");
  planning::Trajectory vertical(5);
  for (std::size_t index = 0; index < vertical.size(); ++index) {
    vertical[index].y = static_cast<double>(index) * 2.0;
    vertical[index].orientation_z = std::sin(std::acos(-1.0) * 0.25);
    vertical[index].orientation_w = std::cos(std::acos(-1.0) * 0.25);
  }
  const planning::CruiseObjectShape cylinder{1, 2.0, 2.0, {}};
  const double cylinder_gap = planning::cruiseObjectLateralDistance(
      vertical, {2.8, 4.0, 0.0}, cylinder, 1.0, 1.0, 2.0);
  if (cylinder_gap < 0.93 || cylinder_gap > 0.94)
    throw std::runtime_error("원본 육각형 원기둥 외곽 거리가 잘못되었습니다");
  const std::vector<planning::CruiseTimedPose> predicted{
      {{4.0, 5.0, 0.0}, 0.0},
      {{4.0, 1.2, 0.0}, 0.5},
      {{4.5, 1.2, 0.0}, 1.0}};
  const auto collision = planning::findCruiseCollisionPoints(
      path, predicted, shape, 1.0, 1.0, 2.0, 0.25, 2.0, 10.0);
  if (collision.size() != 2 ||
      collision.front().time_from_object_stamp_s != 0.5 ||
      collision.front().trajectory_index >= path.size())
    throw std::runtime_error("순항 충돌점의 시각 또는 위치가 잘못되었습니다");
  if (!planning::findCruiseCollisionPoints(
           path, predicted, shape, 1.0, 1.0, 2.0, 1.5, 2.0, 10.0)
           .empty())
    throw std::runtime_error("지난 예측 객체가 충돌점으로 남았습니다");
  if (!planning::findCruiseCollisionPoints(
           path, predicted, shape, 1.0, 1.0, 2.0, 0.25, 2.0, 1.0)
           .empty())
    throw std::runtime_error("거리 범위 밖 장애물이 충돌점으로 남았습니다");
  const auto stop_collision = planning::findCruiseCurrentStopCollision(
      path, {8.0, 0.0, 0.0}, shape, 3.74, 1.03, 1.83, 0.0);
  if (!stop_collision || stop_collision->trajectory_index != 2 ||
      std::abs(stop_collision->distance_from_start_m - 3.26) > 1e-9)
    throw std::runtime_error("현재 장애물의 앞부분 기준 정지 교점이 잘못되었습니다");
  const planning::CruiseObjectShape narrow{0, 2.0, 1.0, {}};
  if (planning::findCruiseCurrentStopCollision(
          path, {8.0, 1.5, 0.0}, narrow,
          3.74, 1.03, 1.83, 0.0))
    throw std::runtime_error("차량 외형 밖 객체가 정지 교점으로 잡혔습니다");
  if (!planning::findCruiseCurrentStopCollision(
           path, {8.0, 1.5, 0.0}, narrow,
           3.74, 1.03, 1.83, 0.3))
    throw std::runtime_error("정지 횡여유가 교점 계산에 반영되지 않았습니다");
  if (planning::findCruiseSlowdownCollisionInterval(
          path, {8.0, 2.5, 0.0}, narrow,
          3.74, 1.03, 1.83, 0.0))
    throw std::runtime_error("감속 횡여유 밖의 객체가 선택되었습니다");
  const auto slowdown = planning::findCruiseSlowdownCollisionInterval(
      path, {8.0, 2.5, 0.0}, narrow,
      3.74, 1.03, 1.83, 1.3);
  if (!slowdown || !std::isfinite(slowdown->front_arc_m) ||
      slowdown->front_arc_m >= slowdown->back_arc_m)
    throw std::runtime_error("감속 구간의 첫·마지막 교점이 잘못되었습니다");
  if (std::abs(planning::cruiseCurveStopMargin(
          path, {8.0, 0.0, 0.0}, {8.0, 0.0, 0.0}, shape,
          2.0, 3.0, 6.0, 3.0, 3.0) - 6.0) > 1e-9 ||
      std::abs(planning::cruiseCurveStopMargin(
          path, {8.0, 0.0, 0.0}, {8.0, 10.0, 0.0}, shape,
          2.0, 3.0, 6.0, 3.0, 3.0) - 3.0) > 1e-9)
    throw std::runtime_error("곡선 정지 여유의 직선 충돌 경계가 잘못되었습니다");
}
