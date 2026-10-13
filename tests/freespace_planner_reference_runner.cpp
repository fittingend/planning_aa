// SPDX-License-Identifier: Apache-2.0
// Replay the original node's synthetic free-grid input through the core.
#include "planning/freespace_planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>

int main(int argc, char **argv) {
  if (argc != 7 && argc != 8 && argc != 9 && argc != 10) return 2;
  const double ego_x = std::atof(argv[1]);
  const double ego_y = std::atof(argv[2]);
  const double ego_yaw = std::atof(argv[3]);
  const double goal_x = std::atof(argv[4]);
  const double goal_y = std::atof(argv[5]);
  const double goal_yaw = std::atof(argv[6]);
  planning::TrajectoryPoint ego;
  ego.x = ego_x;
  ego.y = ego_y;
  ego.orientation_z = std::sin(ego_yaw * 0.5);
  ego.orientation_w = std::cos(ego_yaw * 0.5);
  planning::CrosswalkOccupancyGrid grid;
  grid.width = grid.height = 233;
  grid.resolution_m = 0.3;
  grid.origin_x = ego_x - 34.95;
  grid.origin_y = ego_y - 34.95;
  grid.values.assign(grid.width * grid.height, 0);
  if (argc == 8 || argc == 9) {
    std::ifstream generated_grid(argv[7]);
    if (!generated_grid) return 3;
    generated_grid >> grid.width >> grid.height >> grid.resolution_m
                   >> grid.origin_x >> grid.origin_y;
    if (!generated_grid || grid.width == 0 || grid.height == 0) return 3;
    grid.values.resize(grid.width * grid.height);
    for (auto &value : grid.values) {
      int cell = 0;
      generated_grid >> cell;
      if (!generated_grid) return 3;
      value = static_cast<std::int8_t>(cell);
    }
  }
  if (argc == 10) {
    const double obstacle_forward = std::atof(argv[7]);
    const double half_length = std::atof(argv[8]);
    const double half_width = std::atof(argv[9]);
    for (std::size_t iy = 0; iy < grid.height; ++iy) {
      for (std::size_t ix = 0; ix < grid.width; ++ix) {
        const double x = grid.origin_x +
            (static_cast<double>(ix) + 0.5) * grid.resolution_m;
        const double y = grid.origin_y +
            (static_cast<double>(iy) + 0.5) * grid.resolution_m;
        const double dx = x - ego_x;
        const double dy = y - ego_y;
        const double forward = dx * std::cos(ego_yaw) + dy * std::sin(ego_yaw);
        const double lateral = -dx * std::sin(ego_yaw) + dy * std::cos(ego_yaw);
        if (std::abs(forward - obstacle_forward) <= half_length &&
            std::abs(lateral) <= half_width)
          grid.values[iy * grid.width + ix] = 100;
      }
    }
  }

  planning::FreespacePlannerCoreParameters options;
  if (argc == 9) options.search.adapt_expansion_distance = false;
  planning::FreespacePlannerCore planner(options);
  planner.onRoute({goal_x, goal_y, goal_yaw});
  planning::FreespacePlannerCoreInput input;
  input.now_s = 1.0;
  input.parking_active = true;
  input.ego = ego;
  input.occupancy_grid = std::move(grid);
  const auto result = planner.tick(input);
  if (!result.planned || !result.trajectory) return 1;
  if (result.full_trajectory) {
    std::cerr << std::setprecision(17);
    for (const auto &point : *result.full_trajectory)
      std::cerr << "FULLPOINT " << point.x << ' ' << point.y << '\n';
  }
  std::cout << std::setprecision(17);
  for (const auto &point : *result.trajectory)
    std::cout << point.x << ' ' << point.y << ' '
              << point.longitudinal_velocity_mps << '\n';
  input.now_s = 2.0;
  input.ego = result.trajectory->back();
  input.ego->longitudinal_velocity_mps = 0.0F;
  const auto completed = planner.tick(input);
  if (argc == 10 && completed.trajectory) {
    for (const auto &point : *completed.trajectory)
      std::cerr << "NEXTPOINT " << point.x << ' ' << point.y << ' '
                << point.longitudinal_velocity_mps << '\n';
  }
  input.now_s = 2.1;
  const auto stopped = planner.tick(input);
  const bool all_zero = stopped.trajectory &&
      std::all_of(stopped.trajectory->begin(), stopped.trajectory->end(),
                  [](const auto &point) {
                    return std::abs(point.longitudinal_velocity_mps) < 1e-6;
                  });
  std::cerr << "COMPLETION " << completed.completed << ' '
            << (completed.completed_status == true) << ' '
            << all_zero << '\n';
}
