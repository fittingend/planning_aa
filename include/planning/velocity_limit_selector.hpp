// Copyright 2021 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// autoware_external_velocity_limit_selector의 로직을 이관했습니다.

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

namespace planning {

// 시간 기준은 호출자가 제공하며 이 코어에는 ROS 시계나 전송 자료형이 들어오지 않습니다.
struct Stamp {
  std::int64_t nanoseconds = 0;
};

struct VelocityConstraints {
  float max_acceleration = 0.0F;
  float min_acceleration = 0.0F;
  float max_jerk = 0.0F;
  float min_jerk = 0.0F;
};

struct VelocityLimit {
  Stamp stamp;
  float max_velocity = 0.0F;
  bool use_constraints = false;
  VelocityConstraints constraints;
  std::string sender;
};

struct VelocityLimitSelectorConfig {
  double max_velocity = 4.17;
  VelocityConstraints normal{1.0F, -1.0F, 1.0F, -1.0F};
};

class VelocityLimitSelector {
public:
  explicit VelocityLimitSelector(VelocityLimitSelectorConfig config);

  // 원본 콜백처럼 수락된 입력 이벤트마다 값을 생성합니다.
  VelocityLimit onApiLimit(const VelocityLimit &limit, Stamp now);
  VelocityLimit onInternalLimit(const VelocityLimit &limit, Stamp now);
  std::optional<VelocityLimit> onClear(const std::string &sender, bool command, Stamp now);

  // ROS 매개변수 갱신과 같이 새 값은 다음 입력 이벤트부터 사용합니다.
  void setConfig(VelocityLimitSelectorConfig config);
  const VelocityLimit &current() const noexcept { return current_; }

private:
  VelocityLimit update(Stamp now);

  VelocityLimitSelectorConfig config_;
  std::unordered_map<std::string, VelocityLimit> limits_;
  VelocityLimit current_;
};

}  // planning 네임스페이스
