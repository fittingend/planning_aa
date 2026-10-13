// Copyright 2021 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// autoware_external_velocity_limit_selector/src/external_velocity_limit_selector_node.cpp를 바탕으로 작성했습니다.

#include "planning/velocity_limit_selector.hpp"

#include <cmath>
#include <cstddef>
#include <utility>

namespace planning {
namespace {

VelocityLimit getHardestLimit(
    const std::unordered_map<std::string, VelocityLimit> &limits,
    const VelocityLimitSelectorConfig &config) {
  VelocityLimit hardest;
  hardest.max_velocity = static_cast<float>(config.max_velocity);

  const auto normal = config.normal;
  double hardest_max_velocity = config.max_velocity;
  double hardest_max_jerk = 0.0;
  double hardest_max_acceleration = 0.0;
  std::string hardest_max_acceleration_key;
  std::size_t constraint_num = 0;
  std::size_t acceleration_constraint_num = 0;

  for (const auto &[sender, limit] : limits) {
    const double max_velocity =
        std::isfinite(limit.max_velocity) ? limit.max_velocity : config.max_velocity;
    if (max_velocity < hardest_max_velocity) {
      hardest.stamp = limit.stamp;
      hardest.sender = sender;
      hardest.max_velocity = static_cast<float>(max_velocity);
      hardest_max_velocity = max_velocity;
    }

    const auto constraints =
        limit.use_constraints && std::isfinite(limit.constraints.max_jerk)
            ? limit.constraints
            : normal;

    if (limit.use_constraints) {
      ++constraint_num;
      if (limit.constraints.max_acceleration > normal.max_acceleration) {
        ++acceleration_constraint_num;
        hardest_max_acceleration_key = sender;
      }
    }

    if (hardest_max_acceleration < limit.constraints.max_acceleration) {
      hardest_max_acceleration_key = sender;
      hardest_max_acceleration = limit.constraints.max_acceleration;
    }

    if (hardest_max_jerk < constraints.max_jerk) {
      hardest.constraints = constraints;
      hardest.use_constraints = true;
      hardest_max_jerk = constraints.max_jerk;
    }
  }

  if (constraint_num > 0 && constraint_num == acceleration_constraint_num) {
    const auto candidate = limits.find(hardest_max_acceleration_key);
    if (candidate != limits.end()) {
      hardest.constraints = candidate->second.constraints;
      hardest.use_constraints = true;
    }
  }
  return hardest;
}

}  // 익명 네임스페이스

VelocityLimitSelector::VelocityLimitSelector(VelocityLimitSelectorConfig config)
    : config_(std::move(config)) {}

VelocityLimit VelocityLimitSelector::onApiLimit(const VelocityLimit &limit, Stamp now) {
  limits_.insert_or_assign("api", limit);
  return update(now);
}

VelocityLimit VelocityLimitSelector::onInternalLimit(const VelocityLimit &limit, Stamp now) {
  limits_.insert_or_assign(limit.sender, limit);
  return update(now);
}

std::optional<VelocityLimit> VelocityLimitSelector::onClear(
    const std::string &sender, bool command, Stamp now) {
  if (!command) {
    return std::nullopt;
  }
  limits_.erase(sender);
  return update(now);
}

void VelocityLimitSelector::setConfig(VelocityLimitSelectorConfig config) {
  config_ = std::move(config);
}

VelocityLimit VelocityLimitSelector::update(Stamp now) {
  if (limits_.empty()) {
    current_ = {};
    current_.stamp = now;
    current_.max_velocity = static_cast<float>(config_.max_velocity);
  } else {
    current_ = getHardestLimit(limits_, config_);
  }
  return current_;
}

}  // planning 네임스페이스
