// Copyright 2021, 2022 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 검사 결과를 ROS와 독립적인 진단 항목으로 변환합니다.

#pragma once

#include "planning/trajectory_precheck.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace planning {

enum class ValidationDiagnosticLevel { Ok, Warn, Error };

struct ValidationDiagnostic {
  std::string name;
  ValidationDiagnosticLevel level = ValidationDiagnosticLevel::Ok;
  std::string message;
};

std::vector<ValidationDiagnostic> makeValidationDiagnostics(
    const TrajectoryValidationResult &checks,
    std::size_t consecutive_invalid_count,
    std::size_t error_count_threshold);

}  // planning 네임스페이스
