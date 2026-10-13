#include "planning/behavior_velocity_map_planner.hpp"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 3) return 1;
  std::ifstream map_file(argv[1], std::ios::binary);
  if (!map_file) return 2;
  const std::vector<std::uint8_t> map_data{
      std::istreambuf_iterator<char>(map_file), std::istreambuf_iterator<char>()};
  std::vector<planning::BehaviorVelocityModule> modules;
  const std::string mode(argv[2]);
  if (mode == "combined") {
    modules = {planning::BehaviorVelocityModule::StopSign,
               planning::BehaviorVelocityModule::TrafficLight};
  } else if (mode == "crosswalk") {
    modules = {planning::BehaviorVelocityModule::CrosswalkDefault};
  } else if (mode == "crosswalk_signal") {
    modules = {planning::BehaviorVelocityModule::CrosswalkDefault};
  } else if (mode == "crosswalk_occlusion") {
    modules = {planning::BehaviorVelocityModule::CrosswalkDefault};
  } else if (mode == "crosswalk_stuck") {
    modules = {planning::BehaviorVelocityModule::CrosswalkDefault};
  } else if (mode == "crosswalk_sequence") {
    modules = {planning::BehaviorVelocityModule::CrosswalkDefault};
  } else if (mode == "walkway") {
    modules = {planning::BehaviorVelocityModule::Walkway};
  } else if (mode == "detection_area") {
    modules = {planning::BehaviorVelocityModule::DetectionArea};
  } else if (mode == "no_stopping_area") {
    modules = {planning::BehaviorVelocityModule::NoStoppingArea};
  } else if (mode == "merge_from_private" ||
             mode == "merge_from_private_sequence") {
    modules = {planning::BehaviorVelocityModule::IntersectionCollision,
               planning::BehaviorVelocityModule::MergeFromPrivate};
  } else if (mode == "speed_bump") {
    modules = {planning::BehaviorVelocityModule::SpeedBump};
  } else if (mode == "no_drivable_lane" ||
             mode == "no_drivable_sequence") {
    modules = {planning::BehaviorVelocityModule::NoDrivableLane};
  } else if (mode == "intersection_stuck") {
    modules = {planning::BehaviorVelocityModule::IntersectionStuck};
  } else if (mode == "intersection_collision" ||
             mode == "intersection_occlusion" ||
             mode == "intersection_occlusion_sequence" ||
             mode == "intersection_occlusion_release_sequence") {
    modules = {planning::BehaviorVelocityModule::IntersectionCollision};
  } else {
    return 3;
  }
  planning::BehaviorVelocityMapPlanner planner(map_data, modules);
  if (mode == "merge_from_private_sequence") {
    std::size_t frame_count = 0, path_size = 0;
    if (!(std::cin >> frame_count >> path_size)) return 20;
    planning::BehaviorVelocityMapInput input;
    input.trajectory.resize(path_size);
    input.lane_ids.resize(path_size);
    for (std::size_t index = 0; index < path_size; ++index) {
      auto &point = input.trajectory[index];
      std::size_t lane_count = 0;
      if (!(std::cin >> point.x >> point.y >> point.z >>
            point.orientation_x >> point.orientation_y >>
            point.orientation_z >> point.orientation_w >>
            point.longitudinal_velocity_mps >> point.lateral_velocity_mps >>
            point.heading_rate_rps >> lane_count)) return 21;
      input.lane_ids[index].resize(lane_count);
      for (auto &id : input.lane_ids[index]) {
        if (!(std::cin >> id)) return 22;
      }
    }
    std::cout << std::setprecision(17);
    for (std::size_t frame = 0; frame < frame_count; ++frame) {
      if (!(std::cin >> input.now_s >> input.ego_x >> input.ego_y >>
            input.ego_velocity_mps >> input.ego_acceleration_mps2)) return 23;
      const auto result = planner.apply(input);
      std::cout << result.merge_from_private_lanelet_ids.size() << ' '
                << result.trajectory.size() << '\n';
      for (const auto &point : result.trajectory) {
        std::cout << point.x << ' ' << point.y << ' ' << point.z << ' '
                  << point.orientation_x << ' ' << point.orientation_y << ' '
                  << point.orientation_z << ' ' << point.orientation_w << ' '
                  << point.longitudinal_velocity_mps << ' '
                  << point.lateral_velocity_mps << ' '
                  << point.heading_rate_rps << '\n';
      }
    }
    return 0;
  }
  if (mode == "crosswalk_sequence") {
    std::size_t frames = 0;
    if (!(std::cin >> frames)) return 13;
    std::cout << std::setprecision(17);
    for (std::size_t frame = 0; frame < frames; ++frame) {
      planning::BehaviorVelocityMapInput frame_input;
      std::size_t path_size = 0, object_count = 0;
      if (!(std::cin >> frame_input.now_s >> frame_input.ego_x >>
            frame_input.ego_y >> frame_input.ego_velocity_mps >>
            frame_input.ego_acceleration_mps2 >> path_size)) return 14;
      frame_input.crosswalk_enable_rtc = true;
      frame_input.trajectory.resize(path_size);
      frame_input.lane_ids.resize(path_size);
      for (auto &point : frame_input.trajectory) {
        if (!(std::cin >> point.x >> point.y >> point.z >>
              point.orientation_x >> point.orientation_y >>
              point.orientation_z >> point.orientation_w >>
              point.longitudinal_velocity_mps >> point.lateral_velocity_mps >>
              point.heading_rate_rps)) return 15;
      }
      if (!(std::cin >> object_count)) return 16;
      for (std::size_t index = 0; index < object_count; ++index) {
        planning::BehaviorCrosswalkObject object;
        std::size_t path_count = 0;
        int classification = 0;
        if (!(std::cin >> object.id >> object.position.x >> object.position.y >>
              classification >> object.predicted.length_m >>
              object.predicted.width_m >> object.predicted.velocity_x_mps >>
              object.predicted.velocity_y_mps >> path_count)) return 17;
        object.classification = static_cast<std::uint8_t>(classification);
        object.predicted.predicted_paths.resize(path_count);
        for (auto &predicted : object.predicted.predicted_paths) {
          std::size_t pose_count = 0;
          if (!(std::cin >> pose_count)) return 18;
          predicted.resize(pose_count);
          for (auto &pose : predicted) {
            if (!(std::cin >> pose.x >> pose.y >> pose.yaw_rad)) return 19;
          }
        }
        frame_input.crosswalk_objects.push_back(std::move(object));
      }
      const auto result = planner.apply(frame_input);
      int state = -1;
      const auto crosswalk = result.crosswalk_object_states.find(24783);
      if (crosswalk != result.crosswalk_object_states.end()) {
        for (const auto &[id, value] : crosswalk->second) {
          if (id.rfind("a9b8", 0) == 0) state = static_cast<int>(value);
        }
      }
      std::cout << state << ' ' << result.trajectory.size() << '\n';
      for (const auto &point : result.trajectory) {
        std::cout << point.x << ' ' << point.y << ' ' << point.z << ' '
                  << point.orientation_x << ' ' << point.orientation_y << ' '
                  << point.orientation_z << ' ' << point.orientation_w << ' '
                  << point.longitudinal_velocity_mps << ' '
                  << point.lateral_velocity_mps << ' '
                  << point.heading_rate_rps << '\n';
      }
    }
    return 0;
  }
  planning::BehaviorVelocityMapInput input;
  std::size_t count = 0;
  if (!(std::cin >> input.ego_x >> input.ego_y >> input.ego_velocity_mps >>
        input.ego_acceleration_mps2 >> count)) return 4;
  input.trajectory.resize(count);
  input.lane_ids.resize(count);
  for (std::size_t index = 0; index < count; ++index) {
    auto &point = input.trajectory[index];
    std::size_t lane_count = 0;
    if (!(std::cin >> point.x >> point.y >> point.z >>
          point.orientation_x >> point.orientation_y >> point.orientation_z >>
          point.orientation_w >> point.longitudinal_velocity_mps >>
          point.lateral_velocity_mps >> point.heading_rate_rps >> lane_count)) return 5;
    input.lane_ids[index].resize(lane_count);
    for (auto &id : input.lane_ids[index]) {
      if (!(std::cin >> id)) return 6;
    }
  }
  if (mode == "crosswalk_signal") {
    std::int64_t signal_id = 0;
    int signal_color = 0;
    std::size_t object_count = 0;
    if (!(std::cin >> signal_id >> signal_color >> object_count)) return 7;
    const auto color = signal_color == 1
        ? planning::TrafficLampColor::Red : planning::TrafficLampColor::Green;
    input.pedestrian_signals[signal_id] = {
        0.0, {{color, planning::TrafficLampShape::Circle}}};
    for (std::size_t index = 0; index < object_count; ++index) {
      planning::BehaviorCrosswalkObject object;
      std::size_t path_count = 0;
      int classification = 0;
      if (!(std::cin >> object.id >> object.position.x >> object.position.y >>
            classification >> object.predicted.length_m >>
            object.predicted.width_m >> object.predicted.velocity_x_mps >>
            object.predicted.velocity_y_mps >> path_count)) return 8;
      object.classification = static_cast<std::uint8_t>(classification);
      object.predicted.predicted_paths.resize(path_count);
      for (auto &path : object.predicted.predicted_paths) {
        std::size_t pose_count = 0;
        if (!(std::cin >> pose_count)) return 9;
        path.resize(pose_count);
        for (auto &pose : path) {
          if (!(std::cin >> pose.x >> pose.y >> pose.yaw_rad)) return 10;
        }
      }
      input.crosswalk_objects.push_back(std::move(object));
    }
  }
  if (mode == "crosswalk_occlusion") {
    planning::CrosswalkOccupancyGrid grid;
    int enabled = 0;
    int fill_unknown = 0;
    double patch_x = 0.0, patch_y = 0.0, patch_size = 0.0;
    double mask_x = 0.0, mask_y = 0.0, mask_speed = -1.0;
    if (!(std::cin >> grid.width >> grid.height >> grid.resolution_m >>
          grid.origin_x >> grid.origin_y >> grid.origin_yaw_rad >>
          enabled >> fill_unknown >> patch_x >> patch_y >> patch_size >>
          mask_x >> mask_y >> mask_speed)) return 11;
    grid.values.assign(grid.width * grid.height, fill_unknown ? 50 : 0);
    if (patch_size > 0.0) {
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
    }
    if (mask_speed >= 0.0) {
      planning::CrosswalkOcclusionMaskObject mask;
      mask.pose = {mask_x, mask_y, 0.0};
      mask.length_m = mask.width_m = 3.0;
      mask.longitudinal_speed_mps = mask_speed;
      input.crosswalk_occlusion_mask_objects.push_back(mask);
    }
    input.crosswalk_enable_rtc = true;
    input.crosswalk_occlusion_enabled = enabled != 0;
    input.crosswalk_occlusion_parameters.assumed_object_speed_mps = 2.0;
    input.crosswalk_occupancy_grid = std::move(grid);
  }
  if (mode == "crosswalk_stuck") {
    planning::CrosswalkStuckVehicle vehicle;
    int enable_in_intersection = 0;
    if (!(std::cin >> vehicle.position.x >> vehicle.position.y >>
          vehicle.length_m >> vehicle.speed_mps >> enable_in_intersection)) return 12;
    input.crosswalk_enable_rtc = true;
    input.crosswalk_stuck_vehicles.push_back(vehicle);
    input.crosswalk_stuck_parameters.enable_in_intersection =
        enable_in_intersection != 0;
  }
  if (mode == "detection_area") {
    std::size_t point_count = 0;
    if (!(std::cin >> point_count)) return 29;
    input.detection_area_obstacle_points.resize(point_count);
    for (auto &point : input.detection_area_obstacle_points) {
      if (!(std::cin >> point.x >> point.y)) return 30;
    }
  }
  if (mode == "intersection_stuck") {
    std::size_t vehicle_count = 0;
    if (!(std::cin >> vehicle_count)) return 20;
    input.intersection_stuck_vehicles.resize(vehicle_count);
    for (auto &vehicle : input.intersection_stuck_vehicles) {
      int classification = 0;
      if (!(std::cin >> vehicle.position.x >> vehicle.position.y >>
            vehicle.speed_mps >> classification)) return 21;
      vehicle.classification = static_cast<std::uint8_t>(classification);
    }
  }
  if (mode == "no_stopping_area") {
    std::size_t vehicle_count = 0;
    if (!(std::cin >> vehicle_count)) return 29;
    input.no_stopping_vehicles.resize(vehicle_count);
    for (auto &vehicle : input.no_stopping_vehicles) {
      int classification = 0;
      if (!(std::cin >> vehicle.position.x >> vehicle.position.y >>
            vehicle.speed_mps >> classification >> vehicle.yaw_rad >>
            vehicle.length_m >> vehicle.width_m)) return 30;
      vehicle.classification = static_cast<std::uint8_t>(classification);
    }
  }
  if (mode == "merge_from_private" ||
      mode == "intersection_collision" ||
      mode == "intersection_occlusion" ||
      mode == "intersection_occlusion_sequence" ||
      mode == "intersection_occlusion_release_sequence") {
    std::size_t vehicle_count = 0;
    if (!(std::cin >> vehicle_count)) return 22;
    for (std::size_t index = 0; index < vehicle_count; ++index) {
      double x = 0.0, y = 0.0, yaw = 0.0, speed = 0.0, duration = 0.0;
      int classification = 0;
      double confidence = 0.0;
      double path_y_shift = 0.0;
      double second_confidence = -1.0;
      if (!(std::cin >> x >> y >> yaw >> speed >> duration >>
            classification >> confidence >> path_y_shift >>
            second_confidence)) return 23;
      planning::IntersectionPredictedVehicle vehicle;
      vehicle.classification = static_cast<std::uint8_t>(classification);
      vehicle.confidence = confidence;
      vehicle.initial_pose = planning::CrosswalkPose2d{x, y, yaw};
      vehicle.initial_speed_mps = speed;
      if (duration >= 0.0) {
        for (std::size_t step = 0;
             step <= static_cast<std::size_t>(duration / 0.5); ++step) {
          const double distance = speed * 0.5 * static_cast<double>(step);
          vehicle.predicted_poses.push_back({
              x + std::cos(yaw) * distance,
              y + std::sin(yaw) * distance + path_y_shift, yaw});
        }
      }
      if (second_confidence >= 0.0) {
        planning::IntersectionPredictedPath second;
        second.confidence = second_confidence;
        second.poses = vehicle.predicted_poses;
        for (auto &pose : second.poses) pose.y -= path_y_shift;
        vehicle.additional_predicted_paths.push_back(std::move(second));
      }
      input.intersection_predicted_vehicles.push_back(std::move(vehicle));
    }
    std::size_t signal_count = 0;
    if (!(std::cin >> signal_count)) return 24;
    for (std::size_t index = 0; index < signal_count; ++index) {
      std::int64_t id = 0;
      int color = 0;
      if (!(std::cin >> id >> color)) return 25;
      input.intersection_signal_colors[id] = color == 1
          ? planning::TrafficLampColor::Red
          : color == 3 ? planning::TrafficLampColor::Yellow
                       : planning::TrafficLampColor::Green;
    }
    if (mode == "intersection_occlusion") {
      planning::CrosswalkOccupancyGrid grid;
      int enabled = 0, fill_unknown = 0;
      double patch_x = 0.0, patch_y = 0.0, patch_size = 0.0;
      if (!(std::cin >> grid.width >> grid.height >> grid.resolution_m >>
            grid.origin_x >> grid.origin_y >> grid.origin_yaw_rad >>
            enabled >> fill_unknown >> patch_x >> patch_y >> patch_size)) {
        return 26;
      }
      grid.values.assign(grid.width * grid.height, fill_unknown ? 50 : 0);
      if (patch_size > 0.0) {
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
      }
      input.intersection_occlusion_enabled = enabled != 0;
      input.intersection_occupancy_grid = std::move(grid);
    }
  }
  if (mode == "intersection_occlusion_sequence" ||
      mode == "intersection_occlusion_release_sequence") {
    planning::CrosswalkOccupancyGrid grid;
    std::size_t frame_count = 0;
    if (!(std::cin >> grid.width >> grid.height >> grid.resolution_m >>
          grid.origin_x >> grid.origin_y >> grid.origin_yaw_rad >>
          frame_count)) return 27;
    grid.values.resize(grid.width * grid.height);
    input.intersection_occlusion_enabled = true;
    if (mode == "intersection_occlusion_release_sequence") {
      planning::IntersectionOcclusionStopLines lines;
      if (!(std::cin >> lines.default_arc_m >> lines.peeking_arc_m)) return 31;
      input.intersection_occlusion_stoplines[25001] = lines;
    }
    for (std::size_t frame = 0; frame < frame_count; ++frame) {
      int unknown = 0, color = 0;
      if (!(std::cin >> input.now_s >> unknown >> color)) return 28;
      if (color == 0) {
        input.intersection_signal_colors.clear();
      } else {
        input.intersection_signal_colors[25004] = color == 1
            ? planning::TrafficLampColor::Red
            : color == 3 ? planning::TrafficLampColor::Yellow
                         : planning::TrafficLampColor::Green;
      }
      std::fill(grid.values.begin(), grid.values.end(), unknown ? 50 : 0);
      input.intersection_occupancy_grid = grid;
      const auto result = planner.apply(input);
      const auto first_zero = std::find_if(
          result.trajectory.begin(), result.trajectory.end(),
          [](const auto &point) {
            return point.longitudinal_velocity_mps == 0.0;
          });
      std::cout << result.intersection_occlusion_lanelet_ids.size() << ' '
                << std::distance(result.trajectory.begin(), first_zero) << '\n';
    }
    return 0;
  }
  planning::BehaviorVelocityMapResult result;
  if (mode == "no_drivable_sequence") {
    std::cout << std::setprecision(17);
    for (int frame = 0; frame < 8; ++frame) {
      input.now_s = frame * 0.3;
      result = planner.apply(input);
      std::cout << result.no_drivable_lanelet_ids.size() << ' '
                << result.trajectory.size() << '\n';
      for (const auto &point : result.trajectory) {
        std::cout << point.x << ' ' << point.y << ' ' << point.z << ' '
                  << point.orientation_x << ' ' << point.orientation_y << ' '
                  << point.orientation_z << ' ' << point.orientation_w << ' '
                  << point.longitudinal_velocity_mps << ' '
                  << point.lateral_velocity_mps << ' '
                  << point.heading_rate_rps << '\n';
      }
    }
    return 0;
  }
  for (int frame = 0; frame < 5; ++frame) {
    input.now_s = frame * 0.2;
    result = planner.apply(input);
  }
  std::cout << result.stop_sign_regulatory_ids.size();
  for (const auto id : result.stop_sign_regulatory_ids) std::cout << ' ' << id;
  std::cout << '\n' << result.traffic_light_regulatory_ids.size();
  for (const auto id : result.traffic_light_regulatory_ids) std::cout << ' ' << id;
  const auto &pedestrian_ids = mode == "walkway"
      ? result.walkway_lanelet_ids
      : mode == "detection_area"
          ? result.detection_area_regulatory_ids
      : mode == "no_stopping_area"
          ? result.no_stopping_area_regulatory_ids
      : mode == "merge_from_private"
          ? result.merge_from_private_lanelet_ids
      : mode == "speed_bump"
          ? result.speed_bump_regulatory_ids
      : mode == "no_drivable_lane"
          ? result.no_drivable_lanelet_ids
      : mode == "intersection_stuck"
          ? result.intersection_stuck_lanelet_ids
          : mode == "intersection_collision"
              ? result.intersection_collision_lanelet_ids
          : mode == "intersection_occlusion"
              ? result.intersection_occlusion_lanelet_ids
          : result.crosswalk_lanelet_ids;
  std::cout << '\n' << pedestrian_ids.size();
  for (const auto id : pedestrian_ids) std::cout << ' ' << id;
  std::cout << '\n' << result.trajectory.size() << '\n' << std::setprecision(17);
  for (const auto &point : result.trajectory) {
    std::cout << point.x << ' ' << point.y << ' ' << point.z << ' '
              << point.orientation_x << ' ' << point.orientation_y << ' '
              << point.orientation_z << ' ' << point.orientation_w << ' '
              << point.longitudinal_velocity_mps << ' '
              << point.lateral_velocity_mps << ' '
              << point.heading_rate_rps << '\n';
  }
}
