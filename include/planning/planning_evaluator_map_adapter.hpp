// SPDX-License-Identifier: Apache-2.0
// Planning evaluator의 route/map 기반 자차 차로 지표 입력을 구성합니다.

#pragma once

#include "planning/lanelet_bin_adapter.hpp"
#include "planning/planning_evaluator_core.hpp"

#include <cstdint>
#include <vector>

namespace planning {

class PlanningEvaluatorMapAdapter {
 public:
  explicit PlanningEvaluatorMapAdapter(
      const std::vector<std::uint8_t> &map_data,
      PlanningEvaluatorParameters parameters = {});

  PlanningEvaluatorResult tick(
      PlanningEvaluatorInput input, const MissionRoute &route);
  PlanningEvaluatorCore &core() { return core_; }

 private:
  LaneletBinAdapter map_;
  PlanningEvaluatorCore core_;
};

}  // namespace planning
