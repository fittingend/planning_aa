// 외부 속도 제한 변경 시 원본과 같은 제동 거리 계산을 호출합니다.

#include "planning/jerk_external_limit.hpp"

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
  if (argc != 9) return EXIT_FAILURE;
  const planning::JerkMotion motion{
      std::stod(argv[1]), std::stod(argv[2])};
  planning::JerkExternalLimitParameters parameters;
  parameters.min_deceleration = std::stod(argv[4]);
  parameters.max_jerk = std::stod(argv[5]);
  parameters.min_jerk = std::stod(argv[6]);
  parameters.insertion_margin = std::stod(argv[7]);
  auto plan = planning::planJerkExternalLimit(
      motion, std::stod(argv[3]), parameters);
  planning::advanceJerkExternalLimit(plan, std::stod(argv[8]));
  std::cout << std::setprecision(17) << plan.velocity_before_limit << ','
            << plan.target_velocity << ',' << plan.insertion_distance << ','
            << plan.distance_valid << '\n';
}
