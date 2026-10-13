// SPDX-License-Identifier: Apache-2.0
// 기록된 출발 자세 주변의 지도 차로 중심선을 독립 어댑터에서 읽습니다.

#include "planning/lanelet_bin_adapter.hpp"
#include "planning/behavior_path_lane_safety.hpp"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  if (argc < 4) return 2;
  std::ifstream input(argv[1], std::ios::binary);
  if (!input) return 2;
  const std::vector<std::uint8_t> map_data(
      std::istreambuf_iterator<char>{input},
      std::istreambuf_iterator<char>{});
  const planning::LaneletBinAdapter map(map_data);
  const auto lane_id = map.nearestRoadLaneletId(
      std::stod(argv[2]), std::stod(argv[3]));
  if (!lane_id) {
    std::cout << "-1 0\n";
    return 0;
  }
  const auto centerline = map.laneCenterline(*lane_id);
  std::cout << *lane_id << ' ' << centerline.size() << '\n';
  for (const auto &point : centerline) {
    std::cout.precision(17);
    std::cout << point.x << ' ' << point.y << '\n';
  }
  if (argc > 4) {
    std::vector<planning::BehaviorPathPoint2d> points;
    points.reserve(centerline.size());
    for (const auto &point : centerline) points.push_back({point.x, point.y});
    std::vector<double> distances;
    for (int index = 4; index < argc; ++index) {
      distances.push_back(std::stod(argv[index]));
    }
    const auto samples = planning::sampleStartPullOutBackwardPoses(
        points, {0.0, std::stod(argv[2]), std::stod(argv[3]), 0.0, 0.0},
        distances);
    std::cout << "S " << samples.size() << '\n';
    for (const auto &sample : samples) {
      if (!sample) {
        std::cout << "N\n";
      } else {
        std::cout << sample->x << ' ' << sample->y << ' '
                  << sample->yaw_rad << '\n';
      }
    }
  }
  return 0;
}
