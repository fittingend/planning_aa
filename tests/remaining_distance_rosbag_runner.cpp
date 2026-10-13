// SPDX-License-Identifier: Apache-2.0
// 저장된 LaneletMapBin 내부 자료에서 주기별 남은 거리·시간을 계산합니다.

#include "planning/remaining_distance_map_adapter.hpp"

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <stdexcept>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 4) return 2;
  std::ifstream map_file(argv[1], std::ios::binary);
  std::ifstream input(argv[2]);
  std::ofstream output(argv[3]);
  if (!map_file || !input || !output) return 2;
  const std::vector<std::uint8_t> map{
      std::istreambuf_iterator<char>(map_file),
      std::istreambuf_iterator<char>()};
  planning::RemainingDistanceMapAdapter adapter(map);
  planning::RouteCheckpoint goal;
  input >> goal.x >> goal.y >> goal.yaw_rad >> goal.z;
  if (!input) return 2;
  adapter.core().onGoal(goal);
  adapter.core().onScenario(planning::RemainingDistanceScenario::LaneDriving);
  output << std::setprecision(17);
  std::size_t count = 0;
  input >> count;
  for (std::size_t i = 0; i < count; ++i) {
    planning::RouteCheckpoint ego;
    double velocity_limit = 0.0;
    input >> ego.x >> ego.y >> ego.yaw_rad >> ego.z >> velocity_limit;
    if (!input) throw std::runtime_error("remaining distance input truncated");
    adapter.core().onEgo(ego);
    adapter.core().onVelocityLimit(velocity_limit);
    const auto result = adapter.core().tick();
    if (!result) return 3;
    output << result->distance_m << ' ' << result->time_s << '\n';
  }
}
