// SPDX-License-Identifier: Apache-2.0

#include "planning/planning_evaluator_map_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace planning {

PlanningEvaluatorMapAdapter::PlanningEvaluatorMapAdapter(
    const std::vector<std::uint8_t> &map_data,
    PlanningEvaluatorParameters parameters)
    : owned_map_(std::make_unique<LaneletBinAdapter>(map_data)),
      map_(owned_map_.get()), core_(parameters) {}

PlanningEvaluatorMapAdapter::PlanningEvaluatorMapAdapter(
    const LaneletBinAdapter &map, PlanningEvaluatorParameters parameters)
    : map_(&map), core_(parameters) {}

PlanningEvaluatorResult PlanningEvaluatorMapAdapter::tick(
    PlanningEvaluatorInput input, const MissionRoute &route) {
  if (input.ego) {
    double best_distance = std::numeric_limits<double>::infinity();
    std::unordered_set<std::int64_t> seen;
    for (const auto &section : route.sections) {
      for (const auto lane_id : section.primitive_lanelet_ids) {
        if (!seen.insert(lane_id).second) continue;
        const auto centerline = map_->laneCenterline(lane_id);
        double traveled = 0.0;
        for (std::size_t i = 1; i < centerline.size(); ++i) {
          const auto &a = centerline[i - 1];
          const auto &b = centerline[i];
          const double dx = b.x - a.x;
          const double dy = b.y - a.y;
          const double length = std::hypot(dx, dy);
          if (length < 1e-9) continue;
          const double ratio = std::clamp(
              ((input.ego->x - a.x) * dx +
               (input.ego->y - a.y) * dy) / (length * length),
              0.0, 1.0);
          const double error_x = input.ego->x - a.x - ratio * dx;
          const double error_y = input.ego->y - a.y - ratio * dy;
          const double distance = std::hypot(error_x, error_y);
          if (distance < best_distance) {
            best_distance = distance;
            input.ego_lane_id = lane_id;
            input.ego_lane_arc_m = traveled + ratio * length;
            input.ego_lane_lateral_m =
                (dx * error_y - dy * error_x) / length;
          }
          traveled += length;
        }
      }
    }
  }
  return core_.tick(input);
}

}  // namespace planning
