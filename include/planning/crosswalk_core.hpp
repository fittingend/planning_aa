// Copyright 2020 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 횡단보도 경계, 정지점, 통과 시각 판단에 필요한 독립 계산입니다.

#pragma once

#include "planning/trajectory.hpp"

#include <optional>
#include <vector>

namespace planning {

struct CrosswalkPoint {
  double x = 0.0;
  double y = 0.0;
};

struct CrosswalkParameters {
  double stop_distance_m = 3.5;
  double front_offset_m = 3.79;
};

struct CrosswalkResult {
  Trajectory trajectory;
  std::optional<double> entry_arc_m;
  std::optional<double> exit_arc_m;
  std::optional<double> stop_arc_m;
  std::optional<CrosswalkPoint> stop_point;
};

CrosswalkResult planCrosswalkDefaultStop(
    const Trajectory &path, const std::vector<CrosswalkPoint> &polygon,
    CrosswalkParameters parameters = {});

enum class CrosswalkCollisionState {
  Yield, EgoPassFirst, EgoPassLater, Ignore
};

struct CrosswalkPassParameters {
  double ego_pass_first_margin_s = 4.0;
  double ego_pass_first_additional_margin_s = 0.5;
  double ego_pass_later_margin_s = 13.0;
  double ego_pass_later_additional_margin_s = 0.5;
};

struct CrosswalkPose2d {
  double x = 0.0;
  double y = 0.0;
  double yaw_rad = 0.0;
};

struct CrosswalkPredictedObject {
  double length_m = 0.0;
  double width_m = 0.0;
  double velocity_x_mps = 0.0;
  double velocity_y_mps = 0.0;
  std::vector<std::vector<CrosswalkPose2d>> predicted_paths;
};

struct CrosswalkCollisionPoint {
  CrosswalkPoint point;
  double ego_distance_m = 0.0;
  double object_distance_m = 0.0;
  double time_to_collision_s = 0.0;
  double time_to_vehicle_s = 0.0;
};

struct CrosswalkCollisionParameters {
  double vehicle_front_m = 3.79;
  double vehicle_back_m = 1.1;
  double vehicle_width_m = 1.896;
  double attention_margin_m = 2.0;
  double sparse_interval_m = 4.0;
  double ego_min_assumed_speed_mps = 2.0;
  double object_min_assumed_speed_mps = 1.39;
};

std::optional<CrosswalkCollisionPoint> estimateCrosswalkCollision(
    const Trajectory &path, CrosswalkPoint ego,
    double entry_arc_m, double exit_arc_m,
    double ego_speed_mps, const CrosswalkPredictedObject &object,
    CrosswalkCollisionParameters parameters = {});

struct CrosswalkObjectStopParameters {
  double preferred_margin_m = 3.0;
  double limit_margin_m = 3.0;
  double overrun_threshold_m = 1.0;
};

struct CrosswalkBrakingParameters {
  double preferred_min_acc_mps2 = -1.0;
  double preferred_min_jerk_mps3 = -1.0;
  double strong_min_acc_mps2 = -1.5;
  double strong_min_jerk_mps3 = -1.5;
  double recovery_jerk_mps3 = 10.0;
};

std::optional<double> calcCrosswalkDecelDistance(
    double current_velocity_mps, double target_velocity_mps,
    double current_acceleration_mps2, double min_acceleration_mps2,
    double recovery_jerk_mps3, double braking_jerk_mps3);

// 호출자는 양보로 분류된 충돌점과 별도로 계산한 두 제동 거리를 전달합니다.
CrosswalkResult planCrosswalkObjectStop(
    const Trajectory &path, const std::vector<CrosswalkPoint> &polygon,
    CrosswalkPoint ego, const CrosswalkCollisionPoint &collision,
    double weak_braking_distance_m, double strong_braking_distance_m,
    CrosswalkParameters crosswalk_parameters = {},
    CrosswalkObjectStopParameters object_parameters = {});

CrosswalkResult planCrosswalkObjectStopWithDynamics(
    const Trajectory &path, const std::vector<CrosswalkPoint> &polygon,
    CrosswalkPoint ego, const CrosswalkCollisionPoint &collision,
    double ego_velocity_mps, double ego_acceleration_mps2,
    CrosswalkParameters crosswalk_parameters = {},
    CrosswalkObjectStopParameters object_parameters = {},
    CrosswalkBrakingParameters braking_parameters = {});

CrosswalkResult applyCrosswalkRestartSuppression(
    const Trajectory &path, CrosswalkResult result,
    CrosswalkPoint ego, bool vehicle_stopped,
    double minimum_distance_m = 0.5, double maximum_distance_m = 1.0);

struct CrosswalkStuckVehicle {
  CrosswalkPoint position;
  double length_m = 0.0;
  double speed_mps = 0.0;
  bool vehicle_type = true;
};

struct CrosswalkStuckParameters {
  double maximum_speed_mps = 1.0;
  double maximum_lateral_offset_m = 2.0;
  double required_clearance_m = 6.0;
  double ego_vehicle_length_m = 4.89;
  double minimum_acceleration_mps2 = -1.0;
  double recovery_jerk_mps3 = 1.0;
  double braking_jerk_mps3 = -1.0;
  bool enable_in_intersection = false;
};

CrosswalkResult planCrosswalkStuckVehicleStop(
    const Trajectory &path, const std::vector<CrosswalkPoint> &polygon,
    CrosswalkPoint ego, double ego_velocity_mps,
    double ego_acceleration_mps2,
    const std::vector<CrosswalkStuckVehicle> &objects,
    bool road_has_turn_direction, bool road_has_traffic_light,
    CrosswalkParameters crosswalk_parameters = {},
    CrosswalkStuckParameters stuck_parameters = {});

CrosswalkCollisionState classifyCrosswalkPass(
    double time_to_collision_s, double time_to_vehicle_s,
    CrosswalkCollisionState previous = CrosswalkCollisionState::Yield,
    CrosswalkPassParameters parameters = {});

}  // planning 네임스페이스
