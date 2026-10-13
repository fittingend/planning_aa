#include "planning/behavior_path_postprocess.hpp"
#include "planning/crosswalk_core.hpp"
#include "planning/crosswalk_occlusion.hpp"
#include "planning/lanelet_osm_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  const auto polygon = planning::loadCrosswalkPolygon(argv[1], std::stoll(argv[2]));
  std::size_t count = 0;
  std::cin >> count;
  planning::Trajectory path(count);
  for (auto &point : path) {
    std::cin >> point.x >> point.y >> point.z >> point.longitudinal_velocity_mps;
  }
  double ego_x, ego_y, ego_speed, ego_acceleration;
  std::cin >> ego_x >> ego_y >> ego_speed >> ego_acceleration;
  planning::CrosswalkOccupancyGrid grid;
  std::cin >> grid.width >> grid.height >> grid.resolution_m >>
      grid.origin_x >> grid.origin_y >> grid.origin_yaw_rad;
  if (!std::cin) return 3;
  grid.values.assign(grid.width * grid.height, 50);
  int input_mode = 0;
  planning::CrosswalkOcclusionMaskObject mask_object;
  std::vector<planning::CrosswalkOcclusionMaskObject> mask_objects;
  if (std::cin >> input_mode) {
    if (input_mode != 1) return 6;
    double patch_x, patch_y, patch_size;
    std::cin >> patch_x >> patch_y >> patch_size;
    double mask_x, mask_y, mask_speed;
    std::cin >> mask_x >> mask_y >> mask_speed;
    if (!std::cin || patch_size <= 0.0) return 7;
    grid.values.assign(grid.width * grid.height, 0);
    const double c = std::cos(grid.origin_yaw_rad);
    const double s = std::sin(grid.origin_yaw_rad);
    for (std::size_t row = 0; row < grid.height; ++row) {
      for (std::size_t col = 0; col < grid.width; ++col) {
        const double local_x = (col + 0.5) * grid.resolution_m;
        const double local_y = (row + 0.5) * grid.resolution_m;
        const double x = grid.origin_x + c * local_x - s * local_y;
        const double y = grid.origin_y + s * local_x + c * local_y;
        if (std::abs(x - patch_x) <= patch_size / 2.0 &&
            std::abs(y - patch_y) <= patch_size / 2.0) {
          grid.values[row * grid.width + col] = 50;
        }
      }
    }
    if (mask_speed >= 0.0) {
      mask_object.pose = {mask_x, mask_y, 0.0};
      mask_object.length_m = mask_object.width_m = 3.0;
      mask_object.longitudinal_speed_mps = mask_speed;
      mask_objects.push_back(mask_object);
    }
  }
  planning::CrosswalkOcclusionParameters parameters;
  parameters.assumed_object_speed_mps = 2.0;
  const auto crossing = planning::planCrosswalkDefaultStop(path, polygon);
  if (!crossing.entry_arc_m || !crossing.exit_arc_m) return 4;
  double ego_arc = 0.0;
  double nearest = 1.0e100;
  double arc = 0.0;
  planning::CrosswalkPoint entry_point;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i + 1].x - path[i].x;
    const double dy = path[i + 1].y - path[i].y;
    const double step = std::hypot(dx, dy);
    const double ratio = std::clamp(
        ((ego_x - path[i].x) * dx + (ego_y - path[i].y) * dy) /
            (step * step), 0.0, 1.0);
    const double distance = std::hypot(
        ego_x - path[i].x - ratio * dx,
        ego_y - path[i].y - ratio * dy);
    if (distance < nearest) {
      nearest = distance;
      ego_arc = arc + ratio * step;
    }
    if (arc <= *crossing.entry_arc_m && *crossing.entry_arc_m <= arc + step) {
      const double crossing_ratio = (*crossing.entry_arc_m - arc) / step;
      entry_point = {
          path[i].x + crossing_ratio * dx,
          path[i].y + crossing_ratio * dy};
    }
    arc += step;
  }
  const auto bounds = planning::loadCrosswalkBounds(argv[1], std::stoll(argv[2]));
  const double detection_range = 0.948 + planning::crosswalkDetectionRange(
      parameters.assumed_object_speed_mps,
      *crossing.entry_arc_m - ego_arc, ego_speed);
  const auto detection_areas = planning::crosswalkDetectionAreas(
      bounds.left, bounds.right, entry_point, detection_range);
  const bool occluded = planning::isCrosswalkOccluded(
      grid, detection_areas, parameters, mask_objects);
  planning::CrosswalkOcclusionPlanner planner(parameters);
  planner.update(0.0, occluded, true, false, false, false);
  const bool slow_down = planner.update(
      0.6, occluded, true, false, false, false);
  if (input_mode == 0 && !slow_down) return 5;
  const auto slowed = slow_down ? planner.applySlowdown(
      path, *crossing.entry_arc_m, *crossing.exit_arc_m,
      ego_arc, ego_speed, ego_acceleration) : path;
  const auto output = planning::postprocessBehaviorPath(slowed);
  std::cout << std::setprecision(17) << output.size() << '\n';
  for (const auto &point : output) {
    std::cout << point.x << ' ' << point.y << ' ' << point.z << ' '
              << point.longitudinal_velocity_mps << ' '
              << point.orientation_x << ' ' << point.orientation_y << ' '
              << point.orientation_z << ' ' << point.orientation_w << '\n';
  }
}
