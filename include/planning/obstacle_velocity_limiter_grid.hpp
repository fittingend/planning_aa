// Copyright 2022 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 점유 격자에서 장애물 윤곽을 추출하는 선택적 OpenCV 어댑터입니다.

#pragma once

#include "planning/obstacle_velocity_limiter_core.hpp"

#include <cstdint>
#include <vector>

namespace planning {

struct VelocityLimiterOccupancyGrid {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  double resolution = 0.0;
  MotionVelocityPoint2D origin;
  std::vector<std::int8_t> cells;
};

struct VelocityLimiterGridObstacles {
  std::vector<MotionVelocityPoint2D> points;
  std::vector<VelocityLimiterSegment> segments;
};

VelocityLimiterGridObstacles extractVelocityLimiterGridObstacleGeometry(
    const VelocityLimiterOccupancyGrid &grid, int threshold,
    const VelocityLimiterObstacleMasks &masks = {});

// 원본의 임계값·다각형 마스크·두 번의 팽창과 침식을 적용합니다.
std::vector<VelocityLimiterSegment> extractVelocityLimiterGridObstacles(
    const VelocityLimiterOccupancyGrid &grid, int threshold,
    const VelocityLimiterObstacleMasks &masks = {});

MotionVelocityPlan planVelocityLimiterOccupancyGrid(
    const Trajectory &path, const std::vector<double> &time_from_start,
    std::size_t ego_index, double current_ego_velocity,
    const VelocityLimiterOccupancyGrid &grid, int threshold,
    const VelocityLimiterObstacleMasks &masks = {},
    const ObstacleVelocityLimiterParameters &parameters = {},
    const BicycleVelocityProjectionParameters &bicycle_parameters = {});

}  // planning 네임스페이스
