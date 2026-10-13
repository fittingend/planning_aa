// SPDX-License-Identifier: Apache-2.0

#include "planning/planning_final_pipeline_core.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}
}  // namespace

int main() {
  using namespace planning;
  try {
    PlanningFinalPipelineCore same_route;
    PlanningFinalPipelineCore changed_route;
    PlanningFinalFrame frame;
    frame.route_id = "route-a";
    frame.position.in_lane = true;
    frame.position.goal_in_lane = true;
    frame.maximum_velocity_mps = 11.11111068725586;
    frame.node_maximum_velocity_mps = frame.maximum_velocity_mps;
    frame.external_limit_active = false;
    frame.ego.x = 5.0;
    frame.ego_speed_mps = 5.0;
    frame.now_s = 1.0;
    frame.odometry_stamp_s = 1.0;
    for (int index = 0; index <= 90; ++index) {
      TrajectoryPoint point;
      point.x = index;
      point.longitudinal_velocity_mps = index < 70 ? 8.0F : 0.0F;
      frame.lane_driving.push_back(point);
    }
    const auto first_a = same_route.onFrame(frame);
    const auto first_b = changed_route.onFrame(frame);
    check(first_a.smoothed && first_b.smoothed,
          "first route must produce a trajectory");

    frame.now_s = 1.1;
    frame.odometry_stamp_s = 1.1;
    frame.ego.x = 6.0;
    frame.ego_speed_mps = 4.8;
    frame.lane_driving[50].longitudinal_velocity_mps = 4.0F;
    const auto second_a = same_route.onFrame(frame);
    frame.route_id = "route-b";
    const auto second_b = changed_route.onFrame(frame);
    check(second_a.smoothed && second_b.smoothed &&
              second_a.smoothed->size() == second_b.smoothed->size(),
          "route update must preserve the smoother output size");
    for (std::size_t index = 0; index < second_a.smoothed->size(); ++index) {
      check(std::abs((*second_a.smoothed)[index].longitudinal_velocity_mps -
                     (*second_b.smoothed)[index].longitudinal_velocity_mps) < 1.0e-5,
            "route UUID change must not cold-start the independent smoother");
    }
    check(second_b.scenario &&
              second_b.scenario->current == Scenario::LaneDriving,
          "route UUID change must still reach scenario selector");

    // The source smoother extracts 5 m behind and 200 m ahead of ego before
    // applying the external limit. A long route must not enter the QP whole.
    PlanningFinalPipelineCore long_route;
    frame.route_id = "long-route";
    frame.now_s = 2.0;
    frame.odometry_stamp_s = 2.0;
    frame.ego.x = 250.0;
    frame.lane_driving.clear();
    for (int index = 0; index <= 500; ++index) {
      TrajectoryPoint point;
      point.x = index;
      point.longitudinal_velocity_mps = 8.0F;
      frame.lane_driving.push_back(point);
    }
    const auto long_result = long_route.onFrame(frame);
    check(long_result.smoothed && !long_result.smoothed->empty(),
          "long route must produce a local smoother trajectory");
    check(std::abs(long_result.smoothed->front().x - 244.0) < 1.0e-3,
          "smoother must retain only the source 5 m backward window");
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
