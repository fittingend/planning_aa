// Copyright 2021 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// autoware_planning_validator의 무효 궤적 발행 정책을 이관했습니다.

#pragma once

#include <optional>

namespace planning {

enum class InvalidTrajectoryPolicy {
  PublishAsIs = 0,
  StopPublishing = 1,
  UsePreviousResult = 2,
};

// T는 호출자가 선택한 내부 궤적 자료형입니다. 게이트는 완성된 궤적만 선택하며
// 유효성을 계산하거나 시간을 변경하지 않습니다.
template <typename T>
class TrajectoryGate {
public:
  explicit TrajectoryGate(InvalidTrajectoryPolicy policy) : policy_(policy) {}

  std::optional<T> select(const T &current, bool data_ready, bool is_valid) {
    if (!data_ready) {
      return std::nullopt;
    }
    if (is_valid) {
      previous_valid_ = current;
      return current;
    }
    switch (policy_) {
      case InvalidTrajectoryPolicy::PublishAsIs:
        return current;
      case InvalidTrajectoryPolicy::StopPublishing:
        return std::nullopt;
      case InvalidTrajectoryPolicy::UsePreviousResult:
        return previous_valid_;
    }
    return std::nullopt;
  }

  const std::optional<T> &previousValid() const noexcept { return previous_valid_; }

private:
  InvalidTrajectoryPolicy policy_;
  std::optional<T> previous_valid_;
};

}  // planning 네임스페이스
