// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 경로 최적화 기준점의 방향 보정과 회피 비용을 계산합니다.

#pragma once

#include "planning/path_optimizer_qp.hpp"
#include "planning/path_optimizer_reference_geometry.hpp"

#include <vector>

namespace planning {

struct PathOptimizerAvoidanceParameters {
  double wheelbase = 2.79;
  double maximum_cost_distance = 0.5;
  double cost_margin = 0.0;
  double band_length = 5.0;
  double decrease_per_point = 0.05;
  double interval = 1.0;
};

// 원본 updateExtraPoints가 계산하는 alpha와 회피 비용까지 채웁니다.
// 차량 원별 경계와 beta는 별도 경계 계산 단계에서 채워야 합니다.
std::vector<PathOptimizerReference> buildPathOptimizerReferenceData(
    const PathOptimizerReferenceGeometry &geometry,
    const std::vector<PathOptimizerBound> &center_bounds,
    const PathOptimizerAvoidanceParameters &parameters = {},
    const PathOptimizerReferenceGeometry *previous_geometry = nullptr,
    const std::vector<PathOptimizerReference> *previous_references = nullptr);

}  // planning 네임스페이스
