// SPDX-License-Identifier: Apache-2.0
// 남은 거리와 시간 계산기의 입력 상태·주기 발행 정책.

#pragma once

#include "planning/lanelet_bin_adapter.hpp"

#include <functional>
#include <optional>

namespace planning {

enum class RemainingDistanceScenario { LaneDriving, Parking, Other };

struct RemainingDistanceTime {
  double distance_m = 0.0;
  double time_s = 0.0;
};

using RemainingRouteDistance = std::function<std::optional<double>(
    RouteCheckpoint ego, RouteCheckpoint goal)>;

class RemainingDistanceTimeCore {
 public:
  explicit RemainingDistanceTimeCore(RemainingRouteDistance distance = {});

  void onEgo(RouteCheckpoint ego);
  void onGoal(RouteCheckpoint goal);
  void onScenario(RemainingDistanceScenario scenario);
  void onVelocityLimit(double max_velocity_mps);
  // 기준 노드는 10Hz timer에서 계산·발행한다. nullopt는 발행 안 함이다.
  std::optional<RemainingDistanceTime> tick();
  double velocityLimit() const { return velocity_limit_mps_; }

 private:
  RemainingRouteDistance distance_;
  std::optional<RouteCheckpoint> ego_;
  std::optional<RouteCheckpoint> goal_;
  std::optional<RemainingDistanceScenario> scenario_;
  double velocity_limit_mps_ = 99.99;
  RemainingDistanceTime last_;
};

}  // namespace planning
