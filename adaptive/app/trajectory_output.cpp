// SPDX-License-Identifier: Apache-2.0
#include "trajectory_output.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace adaptive {
namespace {
bool finite(const planning::TrajectoryPoint &p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(p.orientation_x) && std::isfinite(p.orientation_y) &&
         std::isfinite(p.orientation_z) && std::isfinite(p.orientation_w) &&
         std::isfinite(p.longitudinal_velocity_mps) &&
         std::isfinite(p.lateral_velocity_mps) &&
         std::isfinite(p.acceleration_mps2) &&
         std::isfinite(p.heading_rate_rps) &&
         std::isfinite(p.front_wheel_angle_rad) &&
         std::isfinite(p.rear_wheel_angle_rad);
}

template <typename Time>
bool setTime(Time &target, double seconds) {
  if (!std::isfinite(seconds) || seconds < 0.0 ||
      seconds >= static_cast<double>(std::numeric_limits<std::int32_t>::max()))
    return false;
  const auto whole = static_cast<std::int32_t>(seconds);
  const auto nanos = static_cast<std::uint32_t>(
      std::min(999999999.0, std::floor((seconds - whole) * 1.0e9)));
  target.sec = whole;
  target.nanosec = nanos;
  return true;
}
}  // namespace

std::optional<oss::type::autoware_planning_msgs::msg::Trajectory>
makeTrajectoryEvent(const planning::PlanningFinalResult &result,
                    double stamp_s, const std::string &frame_id) {
  if (!result.validation || !result.validation->data_ready ||
      !result.validation->checks.allValid() ||
      !result.validation->selected_trajectory ||
      result.validation->selected_trajectory->size() < 2 ||
      frame_id.empty()) return std::nullopt;

  oss::type::autoware_planning_msgs::msg::Trajectory event{};
  if (!setTime(event.header.stamp, stamp_s)) return std::nullopt;
  event.header.frame_id = frame_id;
  const auto &points = *result.validation->selected_trajectory;
  double elapsed_s = 0.0;
  event.points.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto &source = points[i];
    if (!finite(source)) return std::nullopt;
    if (i != 0) {
      const auto &previous = points[i - 1];
      const double distance = std::hypot(source.x - previous.x,
                                         source.y - previous.y);
      const double mean_speed = 0.5 * (std::abs(source.longitudinal_velocity_mps) +
                                       std::abs(previous.longitudinal_velocity_mps));
      // The core has no point timestamps. Keep time monotonic at stops.
      elapsed_s += std::max(0.1, distance / std::max(0.1, mean_speed));
    }
    oss::type::autoware_planning_msgs::msg::TrajectoryPoint target{};
    if (!setTime(target.time_from_start, elapsed_s)) return std::nullopt;
    target.pose.position.x = source.x;
    target.pose.position.y = source.y;
    target.pose.position.z = source.z;
    target.pose.orientation.x = source.orientation_x;
    target.pose.orientation.y = source.orientation_y;
    target.pose.orientation.z = source.orientation_z;
    target.pose.orientation.w = source.orientation_w;
    target.longitudinal_velocity_mps = source.longitudinal_velocity_mps;
    target.lateral_velocity_mps = source.lateral_velocity_mps;
    target.acceleration_mps2 = source.acceleration_mps2;
    target.heading_rate_rps = source.heading_rate_rps;
    target.front_wheel_angle_rad = source.front_wheel_angle_rad;
    target.rear_wheel_angle_rad = source.rear_wheel_angle_rad;
    event.points.push_back(std::move(target));
  }
  return event;
}
}  // namespace adaptive
