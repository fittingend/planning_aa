// SPDX-License-Identifier: Apache-2.0

#include "planning/costmap_generator_core.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}

std::size_t cell(const planning::CrosswalkOccupancyGrid &grid,
                 double x, double y) {
  const auto ix = static_cast<std::size_t>(
      (x - grid.origin_x) / grid.resolution_m);
  const auto iy = static_cast<std::size_t>(
      (y - grid.origin_y) / grid.resolution_m);
  return iy * grid.width + ix;
}
}  // namespace

int main() {
  using namespace planning;
  try {
    CostmapGeneratorParameters parameters;
    parameters.length_x_m = 10.0;
    parameters.length_y_m = 10.0;
    parameters.resolution_m = 1.0;
    parameters.object_blur_kernel_cells = 3;
    CostmapGeneratorCore generator(parameters);
    generator.onMapAreas({{{-3.0, -3.0}, {3.0, -3.0},
                           {3.0, 3.0}, {-3.0, 3.0}}}, {});
    CostmapGeneratorInput input;
    input.ego_in_parking_lot = false;
    check(!generator.tick(input),
          "default node must stay inactive outside parking lot");
    input.ego_in_parking_lot = true;
    input.stamp_s = 12.5;
    input.points_map_frame.push_back({0.0, 0.0, 1.0});
    input.points_map_frame.push_back({1.0, 0.0, 3.0});
    input.objects_map_frame.push_back({
        {{1.5, -0.5}, {2.5, -0.5}, {2.5, 0.5}, {1.5, 0.5}}, 0.8});
    const auto output = generator.tick(input);
    check(output && output->occupancy.width == 10 &&
              output->occupancy.height == 10 &&
              output->stamp_s == 12.5 && output->frame_id == "map",
          "grid dimensions must follow configured geometry");
    const auto &grid = output->occupancy;
    check(output->primitives[cell(grid, 0.0, 0.0)] == 0.0f &&
              output->primitives[cell(grid, 4.0, 4.0)] == 1.0f,
          "road polygon must clear occupied background");
    check(output->points[cell(grid, -1.0, -1.0)] == 1.0f &&
              output->points[cell(grid, 0.0, -1.0)] == 0.0f,
          "point height filter and source grid-map index must match");
    check(output->objects[cell(grid, 2.0, 0.0)] >= 0.8f &&
              output->occupancy.values[cell(grid, 2.0, 0.0)] >= 80,
          "object probability must enter combined occupancy");

    CostmapGeneratorParameters scenario_parameters = parameters;
    scenario_parameters.activate_by_scenario = true;
    CostmapGeneratorCore scenario_generator(scenario_parameters);
    scenario_generator.onMapAreas({}, {});
    CostmapGeneratorInput scenario_input;
    scenario_input.ego_in_parking_lot = true;
    check(!scenario_generator.tick(scenario_input),
          "lane driving scenario must suppress grid output even inside parking lot");
    scenario_input.parking_activated = true;
    check(scenario_generator.tick(scenario_input).has_value(),
          "parking scenario must activate grid output");
    scenario_input.parking_activated = false;
    check(!scenario_generator.tick(scenario_input),
          "leaving parking scenario must suppress grid output");
    scenario_input.parking_activated = true;
    check(scenario_generator.tick(scenario_input).has_value(),
          "returning to parking scenario must resume grid output");

    CostmapGeneratorParameters shape_parameters = parameters;
    shape_parameters.resolution_m = 0.1;
    shape_parameters.object_blur_kernel_cells = 1;
    CostmapGeneratorCore shape_generator(shape_parameters);
    shape_generator.onMapAreas({{{-5.0, -5.0}, {5.0, -5.0},
                                 {5.0, 5.0}, {-5.0, 5.0}}}, {});
    CostmapGeneratorInput shape_input;
    shape_input.ego_in_parking_lot = true;
    shape_input.objects_map_frame.push_back({
        {{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}},
        1.0, 0, 0.0, 0.0, 0.0, 2.0, 2.0});
    const auto expanded = shape_generator.tick(shape_input);
    check(expanded &&
              expanded->objects[cell(expanded->occupancy, 1.1, 0.0)] == 1.0f &&
              expanded->objects[cell(expanded->occupancy, 1.3, 0.0)] == 0.0f,
          "source box expansion grows each side by half the configured size");
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
