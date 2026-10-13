// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 경로 최적화 기준점에서 좌우 차선 경계까지의 횡방향 거리입니다.

#pragma once

#include "planning/path_optimizer_qp.hpp"
#include "planning/path_optimizer_reference_geometry.hpp"
#include "planning/trajectory.hpp"

#include <vector>

namespace planning {

struct PathOptimizerPoint2D {
  double x = 0.0;
  double y = 0.0;
};

// 차량 중심 기준 10m 횡단선과 지도 경계 선분의 교점을 찾습니다.
double pathOptimizerLateralBound(
    const TrajectoryPoint &reference,
    const std::vector<PathOptimizerPoint2D> &polyline,
    double vehicle_half_width_with_clearance,
    bool left);

// 첫 기준점은 원본처럼 차량 앞 최소 1m 기준점의 경계를 복사합니다.
std::vector<PathOptimizerBound> pathOptimizerCenterBounds(
    const Trajectory &reference,
    const std::vector<PathOptimizerPoint2D> &left_polyline,
    const std::vector<PathOptimizerPoint2D> &right_polyline,
    double vehicle_width, double soft_clearance, double interval = 1.0);

// 원본의 최소 주행 가능 폭 보정과 경계 침범 구간의 전후방 확장입니다.
void keepPathOptimizerMinimumBoundsWidth(
    std::vector<PathOptimizerBound> &bounds,
    const std::vector<double> &curvature, double minimum_width,
    double vehicle_width, double maximum_longitudinal_offset);
std::vector<PathOptimizerBound> extendPathOptimizerViolatedBounds(
    const std::vector<PathOptimizerBound> &bounds,
    double maximum_margin, double interval);

// 현재 차량 속도에 해당하는 앞 구간의 이전 경계를 재사용합니다.
void reusePathOptimizerPreviousBounds(
    std::vector<PathOptimizerBound> &bounds,
    const PathOptimizerReferenceGeometry &geometry,
    const TrajectoryPoint &ego, double ego_speed,
    const PathOptimizerReferenceGeometry &previous_geometry,
    const std::vector<PathOptimizerBound> &previous_bounds,
    double maximum_fixing_time, double interval,
    double nearest_distance = 3.0, double nearest_yaw = 1.046);

// 차량 외곽 원별 경계와 스플라인 자세 차이 beta를 QP 기준점에 채웁니다.
void attachPathOptimizerVehicleBounds(
    const PathOptimizerReferenceGeometry &geometry,
    const std::vector<PathOptimizerBound> &center_bounds,
    const std::vector<PathOptimizerCircle> &circles,
    std::vector<PathOptimizerReference> &references);

}  // planning 네임스페이스
