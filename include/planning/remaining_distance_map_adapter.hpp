// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "planning/lanelet_bin_adapter.hpp"
#include "planning/remaining_distance_time_core.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace planning {

// 지도 최단 경로의 선호 차로 중심선 호길이를 거리 계산기에 공급합니다.
class RemainingDistanceMapAdapter {
 public:
  explicit RemainingDistanceMapAdapter(
      const std::vector<std::uint8_t> &map_data);

  RemainingDistanceTimeCore &core() { return core_; }
  const RemainingDistanceTimeCore &core() const { return core_; }
  std::optional<double> routeDistance(RouteCheckpoint ego,
                                      RouteCheckpoint goal) const;

 private:
  LaneletBinAdapter map_;
  RemainingDistanceTimeCore core_;
};

}  // namespace planning
