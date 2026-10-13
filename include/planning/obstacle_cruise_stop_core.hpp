// SPDX-License-Identifier: Apache-2.0
// 장애물 순항 계획의 정지 후보 선택과 정지점 삽입입니다.

#pragma once

#include "planning/trajectory.hpp"

#include <optional>
#include <string>
#include <vector>

namespace planning {

struct CruiseStopBehaviorParameters {
  double maximum_lateral_margin_m = 0.3;
  double maximum_lateral_time_s = 4.5;
  double collision_time_margin_s = 1.0;
};

struct CruiseStopBehaviorInput {
  bool supported_class = false;
  bool inside_class = false;
  bool outside_class = false;
  double precise_lateral_distance_m = 0.0;
  double approach_velocity_mps = 0.0;
  bool has_predicted_path = false;
  bool has_collision_point = false;
  double collision_time_gap_s = 0.0;
  double time_to_reach_stop_point_s = 0.0;
  bool checked_future_collision = false;
  bool has_future_collision = false;
};

enum class CruiseStopRejection {
  None, UnsupportedClass, OutsideUnsupportedClass,
  OutsideTooFar, NoPredictedPath, NoCollision,
  OutsideTimeGap, InsideUnsupportedClass, TransientInsideObject
};

// 정지 객체의 내외부 분기와 예측 충돌의 시간 조건입니다.
CruiseStopRejection classifyCruiseStopBehavior(
    const CruiseStopBehaviorInput &input,
    const CruiseStopBehaviorParameters &parameters = {});

double cruiseStopTimeToReachCollision(
    double signed_distance_m, double ego_offset_m,
    double ego_speed_mps, double minimum_speed_mps = 2.0);

double cruiseStopCollisionTimeGap(
    double ego_time_to_reach_s, double ego_offset_m,
    double ego_speed_mps, double object_start_s,
    double object_end_s, double minimum_speed_mps = 2.0);

// 경로 끝과 앞선 동작 정지점이 있을 때 객체 정지 여유를 선택합니다.
double chooseCruiseStopMargin(
    double collision_arc_m, double trajectory_length_m,
    std::optional<double> behavior_stop_arc_m,
    double safe_margin_m, double terminal_margin_m,
    double minimum_behavior_margin_m);

struct CruiseStopObservation {
  std::string id;
  double local_velocity_x_mps = 0.0;
};

struct CruiseStopTrackedCandidate {
  std::string id;
  int classification = 0;
  double object_stamp_s = 0.0;
  double collision_distance_from_start_m = 0.0;
};

// 직전 종류별 최근접 정지 객체가 계속 관측될 때 제한 시간 동안 유지합니다.
class CruiseStopConsistency {
 public:
  std::vector<CruiseStopTrackedCandidate> update(
      double current_time_s,
      const std::vector<CruiseStopObservation> &observed,
      const std::vector<CruiseStopTrackedCandidate> &current,
      double hold_time_s = 1.0, double maximum_velocity_mps = 1.5);
  void reset();

 private:
  std::vector<CruiseStopTrackedCandidate> previous_closest_;
};

struct ObstacleCruiseStopCandidate {
  int classification = 0;
  double collision_distance_from_start = 0.0;
  double desired_margin = 0.0;
  double sudden_acceleration_limit = -3.0;
  bool abandon_sudden_stop = false;
  bool type_specific_parameters = false;
  double sudden_object_acceleration_threshold = -1.1;
  double sudden_object_distance_threshold = 1000.0;
};

struct ObstacleCruiseStopParameters {
  double driving_acceleration_limit = 2.0;
  double braking_acceleration_limit = -3.0;
  double stop_hold_velocity = 0.01;
  double stop_hold_distance = 0.3;
  bool suppress_sudden_stop = false;
};

struct ObstacleCruiseStopResult {
  Trajectory trajectory;
  std::optional<double> stop_distance_from_start;
  std::optional<std::size_t> stop_index;
  std::optional<int> classification;
};

class ObstacleCruiseStopCore {
 public:
  explicit ObstacleCruiseStopCore(ObstacleCruiseStopParameters parameters = {});
  ObstacleCruiseStopResult plan(
      const Trajectory &trajectory, double ego_distance_from_start,
      double ego_velocity,
      const std::vector<ObstacleCruiseStopCandidate> &obstacles);
  void reset();

 private:
  ObstacleCruiseStopParameters parameters_;
  std::optional<double> previous_stop_distance_;
  Trajectory previous_trajectory_;
};

}  // planning 네임스페이스
