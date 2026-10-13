#include "planning/intersection_collision_core.hpp"

#include <iostream>

int main() {
  std::size_t count = 0;
  if (!(std::cin >> count)) return 1;
  planning::IntersectionCollisionState state;
  for (std::size_t index = 0; index < count; ++index) {
    double now_s = 0.0;
    bool collision = false;
    if (!(std::cin >> now_s >> collision)) return 2;
    std::cout << state.update(collision, now_s) << '\n';
  }
}
