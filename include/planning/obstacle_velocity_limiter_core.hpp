// Copyright 2022-2024 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 장애물 속도 제한 플러그인의 직선 전방 투영 계산입니다.

#pragma once

#include "planning/motion_velocity_core.hpp"

#include <cstddef>
#include <vector>

namespace planning {

struct VelocityLimiterSegment {
  MotionVelocityPoint2D from;
  MotionVelocityPoint2D to;
};

using VelocityLimiterPolygon = std::vector<MotionVelocityPoint2D>;

struct VelocityLimiterObject {
  MotionVelocityPoint2D center;
  double heading = 0.0;
  double length = 0.0;
  double width = 0.0;
  double velocity_x = 0.0;
  double velocity_y = 0.0;
};

struct VelocityLimiterObstacleMasks {
  VelocityLimiterPolygon positive;
  std::vector<VelocityLimiterPolygon> negative;
};

// 점군의 2차원 경로 주변 영역을 남기고 이동 객체·주행 경로 영역을 제외합니다.
std::vector<MotionVelocityPoint2D> filterVelocityLimiterPointCloud(
    const std::vector<MotionVelocityPoint2D> &points,
    const VelocityLimiterObstacleMasks &masks);

// 원본의 이동 객체 외곽과 주행 경로 제외 영역을 만듭니다.
VelocityLimiterPolygon velocityLimiterObjectPolygon(
    const VelocityLimiterObject &object, double buffer = 0.0);
std::vector<VelocityLimiterPolygon> velocityLimiterMovingObjectMasks(
    const std::vector<VelocityLimiterObject> &objects,
    double buffer, double minimum_velocity);
VelocityLimiterPolygon velocityLimiterTrajectoryMask(
    const Trajectory &path, double lateral_offset);

enum class VelocityLimiterProjectionModel { Particle, Bicycle };
enum class VelocityLimiterDistanceMethod { Exact, Approximation };

struct ObstacleVelocityLimiterParameters {
  VelocityLimiterProjectionModel model = VelocityLimiterProjectionModel::Particle;
  VelocityLimiterDistanceMethod distance_method =
      VelocityLimiterDistanceMethod::Exact;
  double minimum_time_to_collision = 1.0;
  double extra_projection_length = 3.74;
  double minimum_adjusted_velocity = 2.5;
  double maximum_deceleration = 2.0;
  double vehicle_lateral_offset = 0.915;
  double start_distance = 0.0;
  double maximum_path_length = 100.0;
  double maximum_path_duration = 10.0;
  std::size_t downsample_factor = 10;
  bool calculate_steering_angles = false;
};

struct VelocityLimiterSampledPath {
  Trajectory path;
  std::vector<double> time_from_start;
};

struct BicycleVelocityProjectionParameters {
  double wheel_base = 2.74;
  double duration = 1.0;
  double extra_length = 3.74;
  double steering_offset = 0.01;
  std::size_t points_per_projection = 5;
};

MotionVelocityPlan planObstacleVelocityLimitPointCloud(
    const Trajectory &path, const std::vector<double> &time_from_start,
    std::size_t ego_index, double current_ego_velocity,
    const std::vector<MotionVelocityPoint2D> &pointcloud,
    const VelocityLimiterObstacleMasks &masks = {},
    const ObstacleVelocityLimiterParameters &parameters = {},
    const BicycleVelocityProjectionParameters &bicycle_parameters = {},
    bool apply_masks = false);

// 자전거 모형의 좌·중앙·우 전방 투영선을 만듭니다.
std::vector<std::vector<MotionVelocityPoint2D>> bicycleVelocityProjectionLines(
    MotionVelocityPoint2D origin, double heading, double velocity,
    double steering_angle,
    const BicycleVelocityProjectionParameters &parameters = {});

// 시작 접선 방향을 유지하는 원호를 지날 때의 거리를 계산합니다.
double bicycleProjectionArcDistance(
    MotionVelocityPoint2D origin, double heading,
    MotionVelocityPoint2D target);

// 원본과 같은 시작·끝 인덱스를 고른 뒤 고정 간격으로 점을 남깁니다.
VelocityLimiterSampledPath sampleObstacleVelocityLimiterPath(
    const Trajectory &path, const std::vector<double> &time_from_start,
    std::size_t ego_index,
    const ObstacleVelocityLimiterParameters &parameters = {});

// 선택적 자전거 모형 입력을 경로 방향 변화로 다시 계산합니다.
void calculateVelocityLimiterSteeringAngles(
    Trajectory &path, double wheel_base);

// 정적 점·선 장애물을 직선 전방 투영에 대조해 감속 구간을 계산합니다.
// 시간값은 입력 경로의 각 점과 같은 순서이며 원본 궤적의 time_from_start입니다.
MotionVelocityPlan planObstacleVelocityLimitParticle(
    const Trajectory &path, const std::vector<double> &time_from_start,
    const std::vector<MotionVelocityPoint2D> &obstacle_points,
    const std::vector<VelocityLimiterSegment> &obstacle_segments,
    double current_ego_velocity,
    const ObstacleVelocityLimiterParameters &parameters = {});

// 자전거 모형의 세 전방 투영선과 정적 장애물 접촉으로 감속 구간을 계산합니다.
MotionVelocityPlan planObstacleVelocityLimitBicycle(
    const Trajectory &path, const std::vector<double> &time_from_start,
    const std::vector<MotionVelocityPoint2D> &obstacle_points,
    const std::vector<VelocityLimiterSegment> &obstacle_segments,
    double current_ego_velocity,
    const ObstacleVelocityLimiterParameters &velocity_parameters = {},
    const BicycleVelocityProjectionParameters &projection_parameters = {});

}  // planning 네임스페이스
