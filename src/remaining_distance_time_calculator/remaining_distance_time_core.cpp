// SPDX-License-Identifier: Apache-2.0

#include "planning/remaining_distance_time_core.hpp"

#include <cmath>
#include <utility>

namespace planning {

RemainingDistanceTimeCore::RemainingDistanceTimeCore(
    RemainingRouteDistance distance)
    : distance_(std::move(distance)) {}

void RemainingDistanceTimeCore::onEgo(RouteCheckpoint ego) {
  ego_ = ego;
}

void RemainingDistanceTimeCore::onGoal(RouteCheckpoint goal) {
  goal_ = goal;
}

void RemainingDistanceTimeCore::onScenario(
    RemainingDistanceScenario scenario) {
  scenario_ = scenario;
}

void RemainingDistanceTimeCore::onVelocityLimit(double max_velocity_mps) {
  if (std::isfinite(max_velocity_mps) && max_velocity_mps > 1.0e-5)
    velocity_limit_mps_ = max_velocity_mps;
}

std::optional<RemainingDistanceTime> RemainingDistanceTimeCore::tick() {
  if (!scenario_) return std::nullopt;
  if (*scenario_ == RemainingDistanceScenario::Parking) {
    last_ = {};
    return last_;
  }
  if (!distance_ || !goal_ || !ego_) return std::nullopt;
  if (const auto distance = distance_(*ego_, *goal_))
    last_.distance_m = *distance;
  // 원본은 route 계산 실패 시 직전 거리와 시간을 계속 발행한다.
  if (velocity_limit_mps_ > 0.0)
    last_.time_s = last_.distance_m / velocity_limit_mps_;
  return last_;
}

}  // namespace planning
