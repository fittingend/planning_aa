// Copyright 2024 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 차로 밖 주행 플러그인의 충돌 시각과 정지·감속 결정입니다.

#pragma once

#include "planning/motion_velocity_core.hpp"

#include <optional>
#include <vector>

namespace planning {

enum class OutOfLaneTimeMode { TimeToCollision, ArrivalThreshold };

struct OutOfLaneParameters {
  OutOfLaneTimeMode time_mode = OutOfLaneTimeMode::TimeToCollision;
  double time_to_collision_threshold = 1.0;
  double object_arrival_threshold = 5.0;
  double stop_distance_threshold = 15.0;
  double slowdown_velocity = 2.0;
  double minimum_decision_duration = 1.0;
};

struct OutOfLaneCollisionTime {
  std::optional<double> minimum_object_arrival;
  std::optional<double> maximum_object_arrival;
  std::optional<double> time_to_collision;
  bool avoid = false;
};

using OutOfLanePolygon = std::vector<MotionVelocityPoint2D>;

struct OutOfLanePredictedPath {
  std::vector<TrajectoryPoint> poses;
  double time_step = 0.0;
  double object_length = 0.0;
  double object_width = 0.0;
  double confidence = 1.0;
};

struct OutOfLaneStopLine {
  std::vector<MotionVelocityPoint2D> vertices;
  // 정지선이 속한 차로의 다각형을 함께 받아 교차 차로를 구분합니다.
  OutOfLanePolygon lane_polygon;
};

struct OutOfLaneTrackedObject {
  MotionVelocityPoint2D initial_position;
  bool is_pedestrian = false;
  double width = 0.0;
  std::vector<OutOfLanePredictedPath> predicted_paths;
};

struct OutOfLaneObjectFilterParameters {
  bool ignore_behind_ego = true;
  bool cut_at_stop_lines = true;
  double minimum_confidence = 0.1;
  double left_offset = 0.915;
  double right_offset = 0.915;
  double extra_left_offset = 0.0;
  double extra_right_offset = 0.0;
};

// 차로 밖 주행 후보 계산에 사용할 객체 예측 경로만 남깁니다.
std::vector<OutOfLanePredictedPath> filterOutOfLanePredictedObjects(
    const Trajectory &ego_path, MotionVelocityPoint2D ego_position,
    const std::vector<OutOfLaneTrackedObject> &objects,
    const std::vector<OutOfLaneStopLine> &stop_lines,
    const OutOfLaneObjectFilterParameters &parameters = {});

// 객체 경로와 처음 만나는 정지선에서 차량 앞쪽 길이만큼 일찍 자릅니다.
OutOfLanePredictedPath cutOutOfLanePredictedPathAtStopLines(
    OutOfLanePredictedPath path,
    const std::vector<OutOfLaneStopLine> &stop_lines,
    double object_front_overhang);

struct OutOfLaneGeometryParameters {
  double ego_front_offset = 3.74;
  double ego_rear_offset = -1.03;
  double ego_left_offset = 0.915;
  double ego_right_offset = -0.915;
  double extra_front_offset = 0.0;
  double extra_rear_offset = 0.0;
  double extra_left_offset = 0.0;
  double extra_right_offset = 0.0;
  double front_distance_buffer = 1.5;
  double lateral_distance_buffer = 1.0;
  double position_precision = 0.1;
  double minimum_object_confidence = 0.1;
};

// 객체 예측 경로가 겹침 영역에 닿은 시각으로 충돌 시간 조건을 계산합니다.
OutOfLaneCollisionTime classifyOutOfLaneCollisionTimes(
    const std::vector<double> &object_arrival_times,
    double ego_arrival_time, const OutOfLaneParameters &parameters = {});

// 호출자가 선택한 다른 도로 차로 다각형과 필터링된 예측 경로에서 후보를 찾습니다.
std::optional<MotionVelocityPoint2D> findOutOfLaneSlowdownCandidate(
    const Trajectory &path, const std::vector<double> &time_from_start,
    const std::vector<OutOfLanePolygon> &other_lane_polygons,
    const std::vector<OutOfLanePredictedPath> &objects,
    double minimum_stop_arc,
    const OutOfLaneParameters &decision_parameters = {},
    const OutOfLaneGeometryParameters &geometry_parameters = {});

class OutOfLaneDecisionCore {
 public:
  explicit OutOfLaneDecisionCore(OutOfLaneParameters parameters = {});
  // 후보 위치는 앞단의 차로 겹침 형상 계산에서 전달합니다.
  MotionVelocityPlan plan(const Trajectory &path, MotionVelocityPoint2D ego,
                          std::optional<MotionVelocityPoint2D> candidate,
                          double now_seconds);
  std::optional<MotionVelocityPoint2D> previousSlowdownPoint() const noexcept {
    return previous_;
  }
  void reset();

 private:
  OutOfLaneParameters parameters_;
  std::optional<MotionVelocityPoint2D> previous_;
  double previous_time_ = 0.0;
};

}  // planning 네임스페이스
