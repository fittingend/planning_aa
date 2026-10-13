#include "planning/freespace_planner_core.hpp"
#include "../src/behavior_path_planner/behavior_freespace_grid.hpp"

#include <cassert>

using namespace planning;

int main() {
  {
    CrosswalkOccupancyGrid margin_grid;
    margin_grid.width = margin_grid.height = 200;
    margin_grid.resolution_m = 0.1;
    margin_grid.origin_x = margin_grid.origin_y = -10.0;
    margin_grid.values.assign(40000, 0);
    FreespacePullOverParameters margin_parameters;
    margin_parameters.vehicle_shape_margin_m = 0.4;
    const auto outside = detail::cellOf(margin_grid, 0.0, 1.25);
    assert(outside);
    margin_grid.values[static_cast<std::size_t>(outside->second) *
                            margin_grid.width + outside->first] = 100;
    assert(detail::footprintFree(
        margin_grid, margin_parameters, 0.0, 0.0, 0.0));
    const auto inside = detail::cellOf(margin_grid, 0.0, 1.05);
    assert(inside);
    margin_grid.values[static_cast<std::size_t>(inside->second) *
                            margin_grid.width + inside->first] = 100;
    assert(!detail::footprintFree(
        margin_grid, margin_parameters, 0.0, 0.0, 0.0));
  }
  FreespacePlannerCoreParameters parameters;
  parameters.replan_when_obstacle_found = false;
  int searches = 0;
  FreespacePlannerCore planner(parameters,
      [&](const TrajectoryPoint &ego, BehaviorGoalPose,
          const CrosswalkOccupancyGrid &,
          const FreespacePullOverParameters &)
          -> std::optional<ShiftPullOverCandidate> {
        ++searches;
        ShiftPullOverCandidate candidate;
        candidate.path.resize(4, ego);
        for (std::size_t i = 0; i < 4; ++i) {
          candidate.path[i].x = static_cast<double>(i);
          candidate.path[i].longitudinal_velocity_mps =
              i < 2 ? 1.0F : -1.0F;
        }
        candidate.phase_end_indices = {1, 3};
        return candidate;
      });
  CrosswalkOccupancyGrid grid;
  grid.width = grid.height = 100;
  grid.resolution_m = 1.0;
  grid.origin_x = grid.origin_y = -50.0;
  grid.values.assign(10000, 0);
  FreespacePlannerCoreInput input;
  input.parking_active = true;
  input.occupancy_grid = grid;
  input.ego = TrajectoryPoint{};
  planner.onRoute({3.0, 0.0, 0.0});
  auto output = planner.tick(input);
  assert(output.completed_status == false);
  assert(output.planned && searches == 1);
  assert(output.full_trajectory && output.full_trajectory->size() == 4);
  assert(output.target_index == 1 && output.trajectory->size() == 2);
  assert(output.trajectory->back().longitudinal_velocity_mps == 0.0F);

  input.now_s = 1.0;
  input.ego->x = 1.0;
  output = planner.tick(input);
  assert(!output.full_trajectory);
  assert(output.target_index == 3 && output.trajectory->size() == 3);
  assert(output.trajectory->front().x == 1.0);
  input.now_s = 2.0;
  input.ego->x = 3.0;
  output = planner.tick(input);
  assert(output.completed_status == true && output.completed);
  input.now_s = 2.1;
  output = planner.tick(input);
  assert(output.completed && output.trajectory->back().longitudinal_velocity_mps == 0.0F);

  planner.onRoute({4.0, 0.0, 0.0});
  input.now_s = 3.0;
  input.ego->x = 0.0;
  output = planner.tick(input);
  assert(output.planned && searches == 2);
  input.now_s = 3.1;
  input.ego->x = 5.0;
  input.ego->longitudinal_velocity_mps = 1.0F;
  output = planner.tick(input);
  assert(output.waiting_for_stop && output.completed_status == false);
  assert(output.trajectory && output.trajectory->front().longitudinal_velocity_mps == 0.0F);
  input.now_s = 4.2;
  input.ego->x = 0.0;
  input.ego->longitudinal_velocity_mps = 0.0F;
  output = planner.tick(input);
  assert(output.planned && searches == 3);

  input.parking_active = false;
  output = planner.tick(input);
  assert(output.completed_status == false && !output.trajectory);
}
