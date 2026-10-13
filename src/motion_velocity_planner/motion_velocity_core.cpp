// SPDX-License-Identifier: Apache-2.0
// 원본 MotionVelocityPlannerNode의 궤적 점 삽입과 속도 제한입니다.

#include "planning/motion_velocity_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {
namespace {

std::size_t nearestSegment(
    const Trajectory &trajectory, const MotionVelocityPoint2D &target) {
  std::size_t closest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < trajectory.size(); ++i) {
    const double dx = trajectory[i + 1].x - trajectory[i].x;
    const double dy = trajectory[i + 1].y - trajectory[i].y;
    const double norm = dx * dx + dy * dy;
    const double ratio = norm < 1.0e-12 ? 0.0 : std::clamp(
        ((target.x - trajectory[i].x) * dx +
         (target.y - trajectory[i].y) * dy) / norm, 0.0, 1.0);
    const double distance = std::hypot(
        trajectory[i].x + ratio * dx - target.x,
        trajectory[i].y + ratio * dy - target.y);
    if (distance < best) { best = distance; closest = i; }
  }
  return closest;
}

std::optional<std::size_t> insertPoint(
    Trajectory &trajectory, const MotionVelocityPoint2D &target) {
  if (trajectory.size() < 2 || !std::isfinite(target.x) ||
      !std::isfinite(target.y)) return std::nullopt;
  const std::size_t segment = nearestSegment(trajectory, target);
  auto &front = trajectory[segment];
  const auto &back = trajectory[segment + 1];
  const double dx = back.x - front.x;
  const double dy = back.y - front.y;
  const double norm = dx * dx + dy * dy;
  if (norm < 1.0e-12) return std::nullopt;
  const double ratio = ((target.x - front.x) * dx +
                        (target.y - front.y) * dy) / norm;
  if (ratio < 0.0 || ratio > 1.0) return std::nullopt;
  if (std::hypot(target.x - front.x, target.y - front.y) < 1.0e-3)
    return segment;
  if (std::hypot(target.x - back.x, target.y - back.y) < 1.0e-3)
    return segment + 1;
  TrajectoryPoint inserted = front;
  inserted.x = target.x;
  inserted.y = target.y;
  inserted.z = front.z + ratio * (back.z - front.z);
  const double yaw = std::atan2(back.y - target.y, back.x - target.x);
  inserted.orientation_x = 0.0;
  inserted.orientation_y = 0.0;
  inserted.orientation_z = std::sin(yaw / 2.0);
  inserted.orientation_w = std::cos(yaw / 2.0);
  front.orientation_z = inserted.orientation_z;
  front.orientation_w = inserted.orientation_w;
  trajectory.insert(trajectory.begin() + segment + 1, inserted);
  return segment + 1;
}

}  // 익명 네임스페이스

Trajectory resampleMotionVelocityPlanningInput(const Trajectory &input) {
  Trajectory result;
  for (const auto &point : input) {
    if (result.empty() ||
        std::hypot(point.x - result.back().x, point.y - result.back().y) > 0.5)
      result.push_back(point);
  }
  return result;
}

Trajectory applyMotionVelocityPlans(
    const Trajectory &input, const std::vector<MotionVelocityPlan> &plans) {
  Trajectory output = input;
  for (const auto &plan : plans) {
    for (const auto &stop : plan.stop_points) {
      const auto index = insertPoint(output, stop);
      if (!index) continue;
      for (std::size_t i = *index; i < output.size(); ++i)
        output[i].longitudinal_velocity_mps = 0.0F;
    }
    for (const auto &slowdown : plan.slowdowns) {
      if (!std::isfinite(slowdown.velocity) || slowdown.velocity < 0.0)
        throw std::invalid_argument("동작 속도 계획 감속 속도가 유효하지 않습니다");
      const auto first = insertPoint(output, slowdown.from);
      const auto last = insertPoint(output, slowdown.to);
      if (!first || !last || *first > *last) continue;
      for (std::size_t i = *first; i <= *last; ++i)
        output[i].longitudinal_velocity_mps = static_cast<float>(
            std::min<double>(output[i].longitudinal_velocity_mps,
                             slowdown.velocity));
    }
  }
  return output;
}

}  // planning 네임스페이스
