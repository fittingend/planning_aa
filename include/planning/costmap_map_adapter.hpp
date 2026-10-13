// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "planning/costmap_generator_core.hpp"
#include "planning/lanelet_bin_adapter.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace planning {

class CostmapMapAdapter {
 public:
  explicit CostmapMapAdapter(
      const std::vector<std::uint8_t> &map_data,
      CostmapGeneratorParameters parameters = {});

  // caller가 입력 점·객체를 map 좌표계로 변환해야 한다.
  std::optional<CostmapGeneratorResult> tick(CostmapGeneratorInput input);

 private:
  LaneletBinAdapter map_;
  CostmapGeneratorCore core_;
};

}  // namespace planning
