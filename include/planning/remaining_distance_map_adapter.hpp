// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "planning/lanelet_bin_adapter.hpp"
#include "planning/remaining_distance_time_core.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace planning {

// 지도 최단 경로의 선호 차로 중심선 호길이를 거리 계산기에 공급합니다.
class RemainingDistanceMapAdapter {
 public:
  explicit RemainingDistanceMapAdapter(
      const std::vector<std::uint8_t> &map_data);
  explicit RemainingDistanceMapAdapter(const LaneletBinAdapter &map);

  RemainingDistanceTimeCore &core() { return core_; }
  const RemainingDistanceTimeCore &core() const { return core_; }
  void setRoute(const MissionRoute &route) { active_route_ = route; }
  std::optional<double> routeDistance(RouteCheckpoint ego,
                                      RouteCheckpoint goal) const;

 private:
  std::unique_ptr<LaneletBinAdapter> owned_map_;
  const LaneletBinAdapter *map_;
  std::optional<MissionRoute> active_route_;
  RemainingDistanceTimeCore core_;
};

}  // namespace planning
