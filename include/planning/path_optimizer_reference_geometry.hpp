// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 경로 최적화의 기준 경로 재표본화와 곡률 계산입니다.

#pragma once

#include "planning/trajectory.hpp"

#include <array>
#include <optional>
#include <vector>

namespace planning {

struct PathOptimizerReferenceGeometry {
  Trajectory points;
  std::vector<double> curvature;
  std::vector<double> arc_length_to_next;
  std::vector<std::optional<std::array<double, 2>>> optimized_states;
  std::vector<std::optional<std::array<double, 2>>> fixed_states;
};

struct PathOptimizerReferenceGeometryParameters {
  double interval = 1.0;
  double forward_length = 100.0;
  double backward_length = 5.0;
  double margin = 10.0;
  std::size_t maximum_points = 100;
  double nearest_distance = 3.0;
  double nearest_yaw = 1.046;
};

struct PathOptimizerSplinePose {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

// 기준점마다 지정한 전후방 거리의 스플라인 자세를 계산합니다.
std::vector<std::vector<PathOptimizerSplinePose>> samplePathOptimizerSplineOffsets(
    const Trajectory &points, const std::vector<double> &offsets);

// 이전 최적화 기준점의 고정 처리 전까지의 형상 계산입니다.
PathOptimizerReferenceGeometry preparePathOptimizerReferenceGeometry(
    const Trajectory &input, const TrajectoryPoint &ego,
    const PathOptimizerReferenceGeometryParameters &parameters = {},
    const PathOptimizerReferenceGeometry *previous = nullptr);

}  // planning 네임스페이스
