// SPDX-License-Identifier: Apache-2.0
// 원본 감속 디버그의 구간·속도를 독립 궤적 삽입에 적용합니다.

#include "planning/motion_velocity_core.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <optional>
#include <stdexcept>

namespace {
std::optional<planning::MotionVelocityPoint2D> pointAtArc(
    const planning::Trajectory &path, double arc) {
  if (arc < 0.0) return std::nullopt;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double length = std::hypot(
        path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
    if (arc > length) { arc -= length; continue; }
    const double ratio = length < 1e-9 ? 0.0 : arc / length;
    return planning::MotionVelocityPoint2D{
        path[i].x + ratio * (path[i + 1].x - path[i].x),
        path[i].y + ratio * (path[i + 1].y - path[i].y)};
  }
  return std::nullopt;
}
}  // 익명 네임스페이스

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  std::ifstream input(argv[1]);
  std::ofstream output(argv[2]);
  if (!input || !output) return 2;
  output << std::setprecision(17);
  std::size_t frame_count = 0;
  input >> frame_count;
  for (std::size_t frame = 0; frame < frame_count; ++frame) {
    std::size_t count = 0;
    double start_arc = 0.0, end_arc = 0.0, velocity = 0.0;
    input >> count >> start_arc >> end_arc >> velocity;
    planning::Trajectory path(count);
    for (auto &point : path)
      input >> point.x >> point.y >> point.z >> point.orientation_x >>
          point.orientation_y >> point.orientation_z >> point.orientation_w >>
          point.longitudinal_velocity_mps;
    if (!input || count < 2)
      throw std::runtime_error("감속 구간 재생 입력이 잘못되었습니다");
    planning::MotionVelocityPlan plan;
    const auto start = pointAtArc(path, start_arc);
    const auto end = pointAtArc(path, end_arc);
    if (start && end) plan.slowdowns.push_back({*start, *end, velocity});
    const auto result = planning::applyMotionVelocityPlans(path, {plan});
    output << result.size() << '\n';
    for (const auto &point : result)
      output << point.x << ' ' << point.y << ' '
             << point.longitudinal_velocity_mps << '\n';
  }
}
