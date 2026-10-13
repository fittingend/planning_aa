// Copyright 2024 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 교차로에서 자차와 예측 차량의 시간별 점유 충돌을 계산합니다.

#pragma once

#include "planning/crosswalk_core.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace planning {

struct IntersectionPredictedPath {
  double time_step_s = 0.5;
  std::vector<CrosswalkPose2d> poses;
  double confidence = 1.0;
};

struct IntersectionPredictedVehicle {
  double length_m = 3.0;
  double width_m = 3.0;
  double time_step_s = 0.5;
  std::vector<CrosswalkPose2d> predicted_poses;
  bool target_vehicle = true;
  std::uint8_t classification = 1;
  double confidence = 1.0;
  std::vector<IntersectionPredictedPath> additional_predicted_paths;
  std::optional<CrosswalkPose2d> initial_pose;
  std::optional<double> initial_speed_mps;
  std::optional<double> distance_to_stopline_m;
  std::optional<double> stopline_to_ego_path_m;
};

struct IntersectionCollisionParameters {
  double front_m = 3.79;
  double rear_m = 1.1;
  double width_m = 1.896;
  double intersection_speed_mps = 2.778;
  double minimum_speed_mps = 1.388;
  double time_sample_s = 0.1;
  double collision_start_margin_s = 3.0;
  double collision_end_margin_s = 2.0;
  double minimum_predicted_path_confidence = 0.05;
  bool fully_prioritized = false;
  bool partially_prioritized = false;
  double traffic_control_margin_m = 2.0;
  std::optional<double> ego_velocity_mps;
  double ego_acceleration_mps2 = 0.0;
};

struct IntersectionCollisionResult {
  bool collision = false;
  std::optional<double> collision_time_s;
  std::optional<double> ego_arc_m;
};

class IntersectionCollisionState {
 public:
  explicit IntersectionCollisionState(double hold_time_s = 0.5);
  bool update(bool collision, double now_s);
  bool stopped() const noexcept { return stopped_; }

 private:
  double hold_time_s_ = 0.5;
  bool stopped_ = false;
  std::optional<double> clear_started_s_;
};

IntersectionCollisionResult detectIntersectionCollision(
    const Trajectory &path, double ego_x, double ego_y,
    double intersection_entry_arc_m,
    const std::vector<IntersectionPredictedVehicle> &vehicles,
    IntersectionCollisionParameters parameters = {});

}  // planning 네임스페이스
