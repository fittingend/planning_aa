#include "planning/intersection_collision_core.hpp"
#include "planning/lanelet_bin_adapter.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>

int main(int argc, char **argv) {
  if (argc != 2) return 6;
  std::ifstream map_stream(argv[1], std::ios::binary);
  if (!map_stream) return 7;
  const std::vector<std::uint8_t> map_data{
      std::istreambuf_iterator<char>(map_stream),
      std::istreambuf_iterator<char>()};
  planning::LaneletBinAdapter map(map_data);
  double ego_x = 0.0, ego_y = 0.0;
  std::size_t path_size = 0, entry_index = 0, vehicle_count = 0;
  if (!(std::cin >> ego_x >> ego_y >> path_size >> entry_index)) return 1;
  planning::Trajectory path(path_size);
  for (auto &point : path) {
    if (!(std::cin >> point.x >> point.y >> point.z >>
          point.orientation_x >> point.orientation_y >>
          point.orientation_z >> point.orientation_w >>
          point.longitudinal_velocity_mps >>
          point.lateral_velocity_mps >> point.heading_rate_rps)) return 2;
  }
  if (entry_index >= path.size()) return 3;
  double entry_arc = 0.0;
  for (std::size_t i = 1; i <= entry_index; ++i) {
    entry_arc += std::hypot(path[i].x - path[i - 1].x,
                            path[i].y - path[i - 1].y);
  }
  if (!(std::cin >> vehicle_count)) return 4;
  std::vector<planning::IntersectionPredictedVehicle> vehicles;
  for (std::size_t index = 0; index < vehicle_count; ++index) {
    double x = 0.0, y = 0.0, yaw = 0.0, speed = 0.0, duration = 0.0;
    int classification = 0;
    double confidence = 0.0;
    double path_y_shift = 0.0;
    double second_confidence = -1.0;
    if (!(std::cin >> x >> y >> yaw >> speed >> duration >>
          classification >> confidence >> path_y_shift >>
          second_confidence)) return 5;
    planning::IntersectionPredictedVehicle vehicle;
    vehicle.classification = static_cast<std::uint8_t>(classification);
    vehicle.confidence = confidence;
    vehicle.initial_pose = planning::CrosswalkPose2d{x, y, yaw};
    vehicle.initial_speed_mps = speed;
    for (std::size_t i = 0; i <= static_cast<std::size_t>(duration / 0.5); ++i) {
      const double distance = speed * 0.5 * static_cast<double>(i);
      vehicle.predicted_poses.push_back({
          x + std::cos(yaw) * distance,
          y + std::sin(yaw) * distance + path_y_shift, yaw});
    }
    if (second_confidence >= 0.0) {
      planning::IntersectionPredictedPath second;
      second.confidence = second_confidence;
      second.poses = vehicle.predicted_poses;
      for (auto &pose : second.poses) pose.y -= path_y_shift;
      vehicle.additional_predicted_paths.push_back(std::move(second));
    }
    vehicle.target_vehicle = map.isVehicleOnIntersectionConflictLane(
        25001, x, y, yaw);
    vehicles.push_back(std::move(vehicle));
  }
  const auto collision = planning::detectIntersectionCollision(
      path, ego_x, ego_y, entry_arc, vehicles);
  std::cout << std::setprecision(17) << collision.collision << ' '
            << (collision.collision_time_s ? *collision.collision_time_s : -1.0)
            << ' ' << (collision.ego_arc_m ? *collision.ego_arc_m : -1.0)
            << '\n';
}
