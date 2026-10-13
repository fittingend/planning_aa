// Copyright 2023-2024 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 동작 속도 계획의 동적 장애물 필터와 충돌 유지 상태입니다.

#pragma once

#include "planning/motion_velocity_core.hpp"

#include <array>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace planning {

enum class DynamicObjectType {
  Unknown, Car, Bus, Truck, Trailer, Motorcycle, Bicycle, Pedestrian
};

struct DynamicObject {
  std::string id;
  DynamicObjectType type = DynamicObjectType::Unknown;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double velocity = 0.0;
  double length = 0.0;
  double width = 0.0;
};

struct DynamicObstacleStopParameters {
  double extra_object_width = 1.0;
  double minimum_object_velocity = 0.5;
  double stop_distance_buffer = 0.5;
  double time_horizon = 5.0;
  double hysteresis = 1.0;
  double add_duration_buffer = 0.5;
  double remove_duration_buffer = 1.0;
  double minimum_object_distance_from_ego_trajectory = 1.0;
  double ego_longitudinal_offset = 3.74;
  double ego_lateral_offset = 0.915;
  bool ignore_unavoidable_collisions = true;
};

struct DynamicStopCollision {
  std::string object_id;
  MotionVelocityPoint2D point;
};

using DynamicStopRectangle = std::array<MotionVelocityPoint2D, 4>;

double dynamicStopArc(const Trajectory &path, MotionVelocityPoint2D point);
double dynamicStopLateralOffset(const Trajectory &path, MotionVelocityPoint2D point);

// 원본 동작 속도 계획의 저크·가속도 제약을 적용한 최소 제동 거리입니다.
double dynamicStopMinimumBrakingDistance(
    double velocity_mps, double acceleration_mps2,
    double minimum_acceleration_mps2, double maximum_jerk_mps3,
    double minimum_jerk_mps3);

// 원본의 객체 종류·속도·경로 거리·접근 방향 조건을 적용합니다.
std::vector<DynamicObject> filterDynamicStopObjects(
    const std::vector<DynamicObject> &objects, const Trajectory &path,
    const TrajectoryPoint &ego, double longitudinal_offset_to_first_point,
    std::optional<TrajectoryPoint> earliest_stop_pose,
    const DynamicObstacleStopParameters &parameters,
    double hysteresis = 0.0);

// 객체의 현재 앞면부터 시간 지평선만큼 진행한 사각형입니다.
DynamicStopRectangle dynamicStopForwardFootprint(
    const DynamicObject &object,
    const DynamicObstacleStopParameters &parameters,
    double hysteresis = 0.0);

// 객체 진행 영역과 경로 각 점의 차량 외형 경계 교점을 찾습니다.
std::vector<DynamicStopCollision> findDynamicStopCollisions(
    const Trajectory &path, MotionVelocityPoint2D ego,
    const std::vector<DynamicObject> &objects,
    const DynamicObstacleStopParameters &parameters,
    double hysteresis = 0.0);

// 가장 앞선 충돌점과 차량 앞부분·안전 여유로 정지 위치를 정합니다.
std::optional<MotionVelocityPoint2D> dynamicStopPointFromCollision(
    const Trajectory &path, MotionVelocityPoint2D ego,
    MotionVelocityPoint2D collision, double earliest_stop_distance,
    const DynamicObstacleStopParameters &parameters);

class DynamicObstacleStopDecisionCore {
 public:
  explicit DynamicObstacleStopDecisionCore(
      DynamicObstacleStopParameters parameters = {});
  void update(const Trajectory &path,
              const std::vector<DynamicStopCollision> &collisions,
              double now_seconds);
  std::optional<MotionVelocityPoint2D> earliestCollision(
      const Trajectory &path, MotionVelocityPoint2D ego) const;
  bool hasActiveDecision() const;
  void reset();

 private:
  struct State {
    MotionVelocityPoint2D collision;
    bool detected = true;
    std::optional<double> first_detection_time;
    std::optional<double> last_stop_decision_time;
  };
  DynamicObstacleStopParameters parameters_;
  std::unordered_map<std::string, State> states_;
};

class DynamicObstacleStopCore {
 public:
  explicit DynamicObstacleStopCore(DynamicObstacleStopParameters parameters = {});

  // 한 시각의 객체를 검사하고 상태 유지 시간을 반영한 정지 요청을 반환합니다.
  MotionVelocityPlan plan(const Trajectory &path, const TrajectoryPoint &ego,
                          const std::vector<DynamicObject> &objects,
                          double earliest_stop_distance,
                          double longitudinal_offset_to_first_point,
                          double now_seconds);
  void reset();

 private:
  DynamicObstacleStopParameters parameters_;
  DynamicObstacleStopDecisionCore decisions_;
};

}  // planning 네임스페이스
