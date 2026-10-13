// SPDX-License-Identifier: Apache-2.0
// Compare the cruise candidate core on the source node's actual trajectory input.

#include "planning/planning_pipeline_cruise_bridge.hpp"

#include <cstdint>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  planning::CruiseCandidateParameters parameters;
  bool show_stop_distance = false;
  for (int index = 1; index < argc; ++index) {
    const std::string option(argv[index]);
    if (option == "--package-cruise-hysteresis") {
      parameters.inside_cruise_entry_speed_mps = 3.5;
      parameters.inside_cruise_keep_speed_mps = 3.0;
    } else if (option == "--enable-outside-stop-classes") {
      parameters.outside_stop_classes.fill(true);
      parameters.outside_stop_classes[0] = false;
    } else if (option == "--show-stop-distance") {
      show_stop_distance = true;
    } else if (option == "--outside-stop-predicted-path-count" &&
               index + 1 < argc) {
      parameters.outside_stop_predicted_path_count =
          std::stoul(argv[++index]);
    } else if (option == "--stop-collision-time-margin" && index + 1 < argc) {
      parameters.stop_collision_time_margin_s = std::stod(argv[++index]);
    } else {
      return 1;
    }
  }
  if (!std::isfinite(parameters.stop_collision_time_margin_s) ||
      parameters.stop_collision_time_margin_s < 0.0) return 1;
  std::size_t frame_count = 0;
  if (!(std::cin >> frame_count)) return 1;
  planning::PlanningCruiseStopCandidateCore core(parameters);
  std::cout << std::setprecision(17);
  for (std::size_t frame_index = 0; frame_index < frame_count; ++frame_index) {
    double callback_time_s = 0.0;
    double ego_speed_mps = 0.0;
    planning::TrajectoryPoint ego;
    planning::PlanningCruisePerceptionFrame perception;
    std::size_t point_count = 0;
    if (!(std::cin >> callback_time_s >> perception.stamp_s >>
          ego.x >> ego.y >> ego.z >> ego.orientation_x >>
          ego.orientation_y >> ego.orientation_z >> ego.orientation_w >>
          ego_speed_mps >> point_count)) return 2;
    planning::Trajectory path(point_count);
    for (auto &point : path) {
      if (!(std::cin >> point.x >> point.y >> point.z >>
            point.orientation_x >> point.orientation_y >>
            point.orientation_z >> point.orientation_w >>
            point.longitudinal_velocity_mps)) return 3;
    }
    std::size_t object_count = 0;
    if (!(std::cin >> object_count)) return 4;
    perception.objects.resize(object_count);
    for (auto &object : perception.objects) {
      unsigned classification = 0;
      std::size_t footprint_count = 0, prediction_count = 0;
      if (!(std::cin >> object.id >> classification >> object.shape_type >>
            object.x >> object.y >> object.yaw_rad >> object.length_m >>
            object.width_m >> object.velocity_x_mps >>
            object.velocity_y_mps >> footprint_count >> prediction_count))
        return 5;
      object.classification = static_cast<std::uint8_t>(classification);
      object.footprint.resize(footprint_count);
      for (auto &point : object.footprint)
        if (!(std::cin >> point.x >> point.y)) return 6;
      object.predicted_paths.resize(prediction_count);
      for (auto &prediction : object.predicted_paths) {
        std::size_t pose_count = 0;
        if (!(std::cin >> prediction.time_step_s >> prediction.confidence >>
              pose_count)) return 7;
        prediction.poses.resize(pose_count);
        for (auto &pose : prediction.poses)
          if (!(std::cin >> pose.x >> pose.y >> pose.yaw_rad)) return 8;
      }
    }
    const auto stop_candidates = core.plan(
        path, ego, ego_speed_mps, callback_time_s, perception);
    const auto &obstacles = core.pidObstacles();
    std::cout << frame_index << ' ' << obstacles.size() << ' '
              << (obstacles.size() == 1 ?
                      obstacles.front().distance_from_ego_front : -1.0)
              << ' ' << stop_candidates.size();
    if (show_stop_distance)
      std::cout << ' ' << (stop_candidates.empty() ? -1.0 :
          stop_candidates.front().collision_distance_from_start);
    std::cout << '\n';
  }
}
