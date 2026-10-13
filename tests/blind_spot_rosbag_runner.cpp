// K-City rosbag에서 추출한 경로·객체를 사각지대 지도 계획기에 연속 입력합니다.

#include "planning/behavior_velocity_map_planner.hpp"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  std::ifstream map_stream(argv[1], std::ios::binary);
  if (!map_stream) return 2;
  const std::vector<std::uint8_t> map_data{
      std::istreambuf_iterator<char>(map_stream),
      std::istreambuf_iterator<char>()};
  planning::BehaviorVelocityMapPlanner planner(
      map_data, {planning::BehaviorVelocityModule::BlindSpot});
  std::size_t frame_index = 0;
  while (true) {
    char kind = 0;
    if (!(std::cin >> kind)) break;
    if (kind != 'F') return 3;
    planning::BehaviorVelocityMapInput input;
    input.blind_spot_auto_enabled = true;
    std::size_t point_count = 0;
    std::size_t object_count = 0;
    std::cin >> input.now_s >> input.ego_x >> input.ego_y >>
        input.ego_velocity_mps >> point_count >> object_count;
    if (!std::cin || point_count > 100000 || object_count > 10000) return 3;
    for (std::size_t index = 0; index < point_count; ++index) {
      if (!(std::cin >> kind) || kind != 'P') return 3;
      planning::TrajectoryPoint point;
      std::size_t lane_count = 0;
      std::cin >> point.x >> point.y >> point.z >>
          point.orientation_x >> point.orientation_y >>
          point.orientation_z >> point.orientation_w >>
          point.longitudinal_velocity_mps >> lane_count;
      if (!std::cin || lane_count > 1000) return 3;
      std::vector<std::int64_t> lane_ids(lane_count);
      for (auto &lane_id : lane_ids) std::cin >> lane_id;
      if (!std::cin) return 3;
      input.trajectory.push_back(point);
      input.lane_ids.push_back(std::move(lane_ids));
    }
    for (std::size_t index = 0; index < object_count; ++index) {
      if (!(std::cin >> kind) || kind != 'O') return 3;
      planning::BlindSpotMapObject object;
      std::cin >> object.id >> object.classification >>
          object.position.x >> object.position.y >>
          object.longitudinal_velocity_mps;
      if (!std::cin) return 3;
      input.blind_spot_map_objects.push_back(std::move(object));
    }
    try {
      const auto result = planner.apply(input);
      std::size_t unsafe = 0;
      for (const auto &[lane_id, decision] : result.blind_spot_decisions) {
        (void)lane_id;
        unsafe += decision.kind == planning::BlindSpotDecisionKind::Unsafe;
      }
      std::cout << frame_index << ' ' << result.blind_spot_decisions.size()
                << ' ' << unsafe << '\n';
    } catch (const std::exception &error) {
      std::cerr << "프레임 " << frame_index << ": " << error.what() << '\n';
      return 4;
    }
    ++frame_index;
  }
  return 0;
}
