// SPDX-License-Identifier: Apache-2.0

#include "planning/planning_pipeline_map_adapter.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>

int main(int argc, char **argv) {
  if (argc != 2) return EXIT_FAILURE;
  try {
    std::ifstream file(argv[1], std::ios::binary);
    if (!file) throw std::runtime_error("map file unavailable");
    const std::vector<std::uint8_t> map{
        std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
    planning::PlanningPipelineMapAdapter planner(map, {});
    planning::MissionRoute route;
    route.sections.push_back({24747, {24747}});
    planning::PlanningPipelineFrame frame;
    frame.timestamp_s = 1.0;
    frame.ego.x = 2585.016995171108;
    frame.ego.y = 23942.35769895078;
    frame.ego_speed_mps = 0.0;
    planning::BehaviorRouteActionInput action;
    action.pull_over = planning::BehaviorRoutePullOverRequest{};
    action.pull_over->goal = {2587.0, 23944.0, 0.0};
    action.pull_over->goal_lanelet_ids = {24747};
    action.pull_over->use_freespace_fallback = true;
    const auto result = planner.onRouteFrame(
        frame, route, {}, {}, {}, action.pull_over->goal,
        std::nullopt, 5.0, action);
    if (!result.generated_parking_grid ||
        result.generated_parking_grid->values.empty())
      throw std::runtime_error("planning pipeline did not generate parking grid");
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
