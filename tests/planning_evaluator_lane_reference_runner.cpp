// SPDX-License-Identifier: Apache-2.0
// Compare route/map lane metrics with the original evaluator node.
#include "planning/planning_evaluator_map_adapter.hpp"

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <vector>

int main(int argc, char **argv) {
  if (argc < 5) return EXIT_FAILURE;
  try {
    std::ifstream map_file(argv[1], std::ios::binary);
    if (!map_file) throw std::runtime_error("map file unavailable");
    const std::vector<std::uint8_t> map_bytes{
        std::istreambuf_iterator<char>(map_file),
        std::istreambuf_iterator<char>()};
    planning::MissionRoute route;
    for (int i = 4; i < argc; ++i) {
      const auto lane_id = std::stoll(argv[i]);
      route.sections.push_back({lane_id, {lane_id}});
    }
    planning::PlanningEvaluatorInput input;
    planning::TrajectoryPoint ego{};
    ego.x = std::stod(argv[2]);
    ego.y = std::stod(argv[3]);
    ego.orientation_w = 1.0;
    input.ego = ego;
    planning::PlanningEvaluatorMapAdapter evaluator(map_bytes);
    const auto metrics = evaluator.tick(input, route).scalar_metrics;
    std::cout << std::setprecision(17)
              << metrics.at("ego_lane_info/lane_id") << ' '
              << metrics.at("ego_lane_info/s") << ' '
              << metrics.at("ego_lane_info/t") << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
