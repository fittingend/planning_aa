#include "planning/crosswalk_core.hpp"
#include "planning/crosswalk_object_state.hpp"
#include "planning/lanelet_osm_adapter.hpp"
#include "planning/behavior_path_postprocess.hpp"

#include <iomanip>
#include <iostream>

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  const auto polygon = planning::loadCrosswalkPolygon(argv[1], std::stoll(argv[2]));
  std::size_t size = 0;
  std::cin >> size;
  planning::Trajectory path(size);
  for (auto &point : path) {
    std::cin >> point.x >> point.y >> point.z >> point.longitudinal_velocity_mps;
  }
  if (!std::cin) return 3;
  const auto result = planning::planCrosswalkDefaultStop(path, polygon);
  if (!result.stop_point) return 4;
  std::cout << std::setprecision(17) << *result.entry_arc_m << ' '
            << *result.stop_arc_m << ' ' << result.stop_point->x << ' '
            << result.stop_point->y << '\n';
  const auto output = planning::postprocessBehaviorPath(result.trajectory);
  std::cout << output.size() << '\n';
  for (const auto &point : output) {
    std::cout << point.x << ' ' << point.y << ' ' << point.z << ' '
              << point.longitudinal_velocity_mps << ' '
              << point.orientation_x << ' ' << point.orientation_y << ' '
              << point.orientation_z << ' ' << point.orientation_w << '\n';
  }
  double ego_x, ego_y, ego_speed, ego_acceleration;
  if (std::cin >> ego_x >> ego_y >> ego_speed >> ego_acceleration) {
    planning::CrosswalkPredictedObject object;
    std::size_t path_count = 0;
    std::cin >> object.length_m >> object.width_m >>
        object.velocity_x_mps >> object.velocity_y_mps >> path_count;
    object.predicted_paths.resize(path_count);
    for (auto &predicted : object.predicted_paths) {
      std::size_t pose_count = 0;
      std::cin >> pose_count;
      predicted.resize(pose_count);
      for (auto &pose : predicted) {
        std::cin >> pose.x >> pose.y >> pose.yaw_rad;
      }
    }
    if (!std::cin) return 5;
    auto selected_output = planning::postprocessBehaviorPath(path);
    const auto collision = planning::estimateCrosswalkCollision(
        path, {ego_x, ego_y}, *result.entry_arc_m, *result.exit_arc_m,
        ego_speed, object);
    if (collision) {
      std::cout << "충돌 " << collision->point.x << ' ' << collision->point.y << ' '
                << collision->ego_distance_m << ' ' << collision->object_distance_m << ' '
                << collision->time_to_collision_s << ' '
                << collision->time_to_vehicle_s << '\n';
      const auto object_stop = planning::planCrosswalkObjectStopWithDynamics(
          path, polygon, {ego_x, ego_y}, *collision,
          ego_speed, ego_acceleration);
      const auto object_output = planning::postprocessBehaviorPath(object_stop.trajectory);
      selected_output = object_output;
      std::cout << "객체정지 " << (object_stop.stop_point ? object_stop.stop_point->x : 0.0)
                << ' ' << (object_stop.stop_point ? object_stop.stop_point->y : 0.0)
                << ' ' << object_output.size() << '\n';
      for (const auto &point : object_output) {
        std::cout << point.x << ' ' << point.y << ' ' << point.z << ' '
                  << point.longitudinal_velocity_mps << ' '
                  << point.orientation_x << ' ' << point.orientation_y << ' '
                  << point.orientation_z << ' ' << point.orientation_w << '\n';
      }
    } else {
      std::cout << "충돌 없음\n";
    }
    int signal_color = 0;
    if (std::cin >> signal_color) {
      const auto color = signal_color == 1
          ? planning::TrafficLampColor::Red
          : planning::TrafficLampColor::Green;
      const bool red = planning::pedestriansHaveRedSignal(
          {{0.0, {{color, planning::TrafficLampShape::Circle}}}}, 1.0);
      if (red) selected_output = planning::postprocessBehaviorPath(path);
      std::cout << "신호경로 " << selected_output.size() << '\n';
      for (const auto &point : selected_output) {
        std::cout << point.x << ' ' << point.y << ' ' << point.z << ' '
                  << point.longitudinal_velocity_mps << ' '
                  << point.orientation_x << ' ' << point.orientation_y << ' '
                  << point.orientation_z << ' ' << point.orientation_w << '\n';
      }
    }
  }
}
