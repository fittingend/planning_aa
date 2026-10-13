#include "planning/behavior_path_object_safety.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
  planning::BehaviorSafetyRssParameters parameters;
  if (argc > 2) return EXIT_FAILURE;
  if (argc == 2) parameters.ego_velocity_limit_mps = std::stod(argv[1]);
  std::size_t ego_count = 0;
  std::size_t object_count = 0;
  std::size_t region_count = 0;
  while (std::cin >> ego_count >> object_count >> region_count) {
    planning::BehaviorSafetyObject object;
    std::cin >> object.shape_type >> object.length_m >> object.width_m;
    std::vector<planning::BehaviorSafetyPose> ego(ego_count);
    object.predicted.resize(object_count);
    for (auto &pose : ego) {
      std::cin >> pose.time_s >> pose.x >> pose.y >> pose.yaw_rad >>
          pose.velocity_mps;
    }
    for (auto &pose : object.predicted) {
      std::cin >> pose.time_s >> pose.x >> pose.y >> pose.yaw_rad >>
          pose.velocity_mps;
    }
    planning::BehaviorSafetyRegions regions(region_count);
    for (auto &region : regions) {
      region.resize(3);
      for (auto &[x, y] : region) std::cin >> x >> y;
    }
    if (!std::cin) return EXIT_FAILURE;
    const auto collision = planning::firstBehaviorPathObjectCollision(
        ego, {}, object, parameters);
    const auto filtered = regions.empty() ? collision :
        planning::firstBehaviorPathObjectCollision(
            ego, {}, object, parameters, regions);
    const auto print = [](const auto &result) {
      if (!result) {
        std::cout << "safe -1";
        return;
      }
      std::cout << (result->kind ==
          planning::BehaviorSafetyCollisionKind::DirectOverlap ?
          "overlap_polygon" : "overlap_extended_polygon") << ' '
          << result->object_sample_index;
    };
    print(collision);
    std::cout << ' ';
    print(filtered);
    if (collision) {
      std::cout << ' ' << collision->rss_distance_m
                << ' ' << collision->longitudinal_extension_m
                << ' ' << collision->lateral_extension_m
                << ' ' << collision->object_in_front;
    } else {
      std::cout << " 0 0 0 0";
    }
    std::cout << '\n';
  }
  return EXIT_SUCCESS;
}
