// SPDX-License-Identifier: Apache-2.0

#include "planning/costmap_map_adapter.hpp"

#include <utility>

namespace planning {

CostmapMapAdapter::CostmapMapAdapter(
    const std::vector<std::uint8_t> &map_data,
    CostmapGeneratorParameters parameters)
    : map_(map_data), core_(parameters) {
  auto areas = map_.costmapAreaPolygons();
  // 원본은 Lanelet 다각형을 geometry_msgs/Point32로 옮긴 뒤 rasterize한다.
  const auto point32 = [](auto &polygons) {
    for (auto &polygon : polygons)
      for (auto &[x, y] : polygon) {
        x = static_cast<float>(x);
        y = static_cast<float>(y);
      }
  };
  point32(areas.road);
  point32(areas.parking);
  core_.onMapAreas(std::move(areas.road), std::move(areas.parking));
}

std::optional<CostmapGeneratorResult> CostmapMapAdapter::tick(
    CostmapGeneratorInput input) {
  // 기본 launch에서 activate_by_scenario=false: 주차장 내부에 있을 때만 실행.
  input.ego_in_parking_lot = map_.scenarioPosition(
      input.ego_x, input.ego_y, 0.0,
      input.ego_x, input.ego_y, {}).in_parking_lot;
  return core_.tick(input);
}

}  // namespace planning
