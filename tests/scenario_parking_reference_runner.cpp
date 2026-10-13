#include "planning/lanelet_bin_adapter.hpp"
#include "planning/scenario_selector.hpp"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 2 && argc != 3) return 1;
  std::ifstream map_file(argv[1], std::ios::binary);
  if (!map_file) return 2;
  const std::vector<std::uint8_t> map_data{
      std::istreambuf_iterator<char>(map_file), std::istreambuf_iterator<char>()};
  planning::LaneletBinAdapter map(map_data);
  double x = 0.0, y = 0.0, yaw = 0.0;
  std::size_t count = 0;
  if (!(std::cin >> x >> y >> yaw >> count)) return 3;
  std::vector<std::int64_t> ids(count);
  for (auto &id : ids) {
    if (!(std::cin >> id)) return 4;
  }
  if (argc == 3 && std::string(argv[2]) == "timeout") {
    const auto initial_position = map.scenarioPosition(
        x, y, yaw, 2555.150390625, 24006.923828125, ids);
    const auto parking_position = map.scenarioPosition(
        x, y, yaw, 0.0, 0.0, ids);
    planning::ScenarioSelectorCore selector;
    selector.onMap(true);
    selector.onRoute("synthetic-parking");
    selector.onOperationMode(true, true);
    selector.onOdometry(0.0, x, y, 0.0);
    const auto initial = selector.tick(0.0, initial_position);
    if (!initial) return 7;
    planning::ScenarioTrajectory lane;
    lane.stamp_s = 0.0;
    planning::TrajectoryPoint endpoint;
    endpoint.x = x + 100.0;
    endpoint.y = y;
    lane.points.push_back(endpoint);
    selector.onLaneTrajectory(lane, 0.1);
    selector.onRoute("synthetic-parking");
    selector.tick(0.2, parking_position);
    const auto boundary = selector.tick(5.2, parking_position);
    const auto transitioned = selector.tick(5.3, parking_position);
    if (!boundary || !transitioned) return 8;
    std::cout << initial_position.in_lane << ' '
              << initial_position.goal_in_lane << ' '
              << initial_position.in_parking_lot << '\n'
              << static_cast<int>(initial->current) << ' '
              << static_cast<int>(boundary->current) << ' '
              << static_cast<int>(transitioned->current) << '\n';
    return 0;
  }
  if (argc == 3 && std::string(argv[2]) == "empty_timeout") {
    const auto position = map.scenarioPosition(x, y, yaw, 0.0, 0.0, ids);
    planning::ScenarioSelectorCore selector;
    selector.onMap(true);
    selector.onRoute("synthetic-parking");
    selector.onOperationMode(true, true);
    selector.onOdometry(0.0, x, y, 0.0);
    const auto initial = selector.tick(0.0, position);
    if (!initial) return 9;
    planning::ScenarioTrajectory parking;
    parking.stamp_s = 0.0;
    planning::TrajectoryPoint endpoint;
    endpoint.x = x;
    endpoint.y = y;
    parking.points.push_back(endpoint);
    const bool published = selector.onParkingTrajectory(parking, 0.1).has_value();
    selector.tick(0.2, position);
    const auto boundary = selector.tick(3.2, position);
    const auto transitioned = selector.tick(3.3, position);
    if (!boundary || !transitioned) return 10;
    std::cout << static_cast<int>(initial->current) << ' '
              << static_cast<int>(boundary->current) << ' '
              << static_cast<int>(transitioned->current) << ' '
              << published << '\n';
    return 0;
  }
  const auto position = map.scenarioPosition(x, y, yaw, 0.0, 0.0, ids);
  planning::ScenarioSelectorCore selector;
  selector.onMap(true);
  selector.onRoute("synthetic-parking");
  selector.onOperationMode(true, true);
  selector.onOdometry(0.0, x, y, 0.0);
  const auto initial = selector.tick(0.0, position);
  if (!initial) return 5;
  planning::ScenarioTrajectory parking;
  parking.stamp_s = 0.0;
  planning::TrajectoryPoint endpoint;
  endpoint.x = x;
  endpoint.y = y;
  endpoint.longitudinal_velocity_mps = 0.0;
  parking.points.push_back(endpoint);
  const bool parking_published = selector.onParkingTrajectory(parking, 0.1).has_value();
  selector.onParkingCompleted(true);
  selector.onOdometry(0.2, x, y, 0.0);
  const auto completed = selector.tick(0.2, position);
  if (!completed) return 6;
  planning::ScenarioTrajectory lane;
  lane.stamp_s = 0.2;
  lane.points = {endpoint};
  const bool lane_published = selector.onLaneTrajectory(lane, 0.3).has_value();
  std::cout << position.in_lane << ' ' << position.goal_in_lane << ' '
            << position.in_parking_lot << ' ' << position.along_lane << '\n'
            << static_cast<int>(initial->current) << ' '
            << static_cast<int>(completed->current) << ' '
            << parking_published << ' ' << lane_published << '\n';
}
