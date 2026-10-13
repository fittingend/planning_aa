// Copyright 2022 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 장애물 순항 계획의 기본 PID 속도 제한 계산입니다.

#pragma once

#include <optional>
#include <vector>

namespace planning {

struct CruisePidObstacle {
  double distance_from_ego_front = 0.0;
  double velocity = 0.0;
};

struct CruisePidParameters {
  double idling_time = 2.0;
  double minimum_ego_acceleration_for_rss = -1.0;
  double minimum_object_acceleration_for_rss = -1.0;
  double safe_distance_margin = 4.0;
  double evaluation_time = 0.0;
  double normalized_error_filter_gain = 0.2;
  double measurement_filter_gain = 0.5;
  double output_filter_gain = 0.5;
  double kp = 10.0;
  double ki = 0.0;
  double kd = 2.0;
  double acceleration_output_ratio = 0.6;
  double minimum_target_velocity = 0.0;
  double minimum_cruise_acceleration = -2.0;
  bool quadratic_error = true;
};

struct CruisePidVelocityLimit {
  double maximum_velocity = 0.0;
  double minimum_acceleration = 0.0;
  double obstacle_distance = 0.0;
  double target_distance = 0.0;
  double error_distance = 0.0;
};

class ObstacleCruisePidCore {
 public:
  explicit ObstacleCruisePidCore(CruisePidParameters parameters = {});
  std::optional<CruisePidVelocityLimit> update(
      double ego_velocity,
      const std::vector<CruisePidObstacle> &obstacles);
  void reset();

 private:
  CruisePidParameters parameters_;
  std::optional<double> filtered_distance_;
  std::optional<double> filtered_obstacle_velocity_;
  std::optional<double> filtered_error_distance_;
  std::optional<double> filtered_normalized_error_;
  std::optional<double> filtered_output_velocity_;
  std::optional<double> previous_pid_error_;
  double pid_error_sum_ = 0.0;
};

}  // planning 네임스페이스
