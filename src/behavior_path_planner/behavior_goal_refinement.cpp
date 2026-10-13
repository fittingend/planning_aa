// SPDX-License-Identifier: Apache-2.0
// 행동 경로의 고정 목표 접속 계산입니다.

#include "planning/behavior_goal_refinement.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {
namespace {

double distance(const TrajectoryPoint &point, BehaviorGoalPose pose) {
  return std::hypot(point.x - pose.x, point.y - pose.y);
}

BehaviorGoalPath removeOverlaps(const BehaviorGoalPath &input) {
  BehaviorGoalPath result;
  result.left_bound = input.left_bound;
  result.right_bound = input.right_bound;
  result.points.reserve(input.points.size());
  result.lane_ids.reserve(input.lane_ids.size());
  for (std::size_t index = 0; index < input.points.size(); ++index) {
    if (!result.points.empty() &&
        std::hypot(input.points[index].x - result.points.back().x,
                   input.points[index].y - result.points.back().y) < 1.0e-8) {
      continue;
    }
    result.points.push_back(input.points[index]);
    result.lane_ids.push_back(input.lane_ids[index]);
  }
  if (!result.points.empty()) {
    result.points.back().longitudinal_velocity_mps = 0.0F;
  }
  return result;
}

std::optional<std::pair<std::size_t, double>> nearestSegment(
    const Trajectory &path, BehaviorGoalPose pose) {
  constexpr double pi = 3.14159265358979323846;
  double best = 9.0;
  std::optional<std::size_t> nearest;
  for (std::size_t index = 0; index < path.size(); ++index) {
    const double distance_squared =
        std::pow(path[index].x - pose.x, 2) +
        std::pow(path[index].y - pose.y, 2);
    if (distance_squared > 9.0 || distance_squared >= best) continue;
    const double heading = 2.0 * std::atan2(
        path[index].orientation_z, path[index].orientation_w);
    if (std::abs(std::remainder(
            heading - pose.yaw_rad, 2.0 * pi)) > pi / 4.0) continue;
    best = distance_squared;
    nearest = index;
  }
  if (!nearest) return std::nullopt;
  std::size_t segment = *nearest;
  if (segment == path.size() - 1) {
    --segment;
  } else if (segment > 0) {
    const double dx = path[segment + 1].x - path[segment].x;
    const double dy = path[segment + 1].y - path[segment].y;
    if (dx * (pose.x - path[segment].x) +
        dy * (pose.y - path[segment].y) <= 0.0) --segment;
  }
  const double dx = path[segment + 1].x - path[segment].x;
  const double dy = path[segment + 1].y - path[segment].y;
  const double squared = dx * dx + dy * dy;
  const double ratio = squared > 1.0e-12
      ? ((pose.x - path[segment].x) * dx +
         (pose.y - path[segment].y) * dy) / squared : 1.0;
  return std::make_pair(segment, ratio);
}

std::optional<BehaviorGoalPath> refineFilteredGoalPath(
    const BehaviorGoalPath &filtered, BehaviorGoalPose goal,
    std::int64_t goal_lane_id, double search_radius_m) {
  if (filtered.points.size() < 2) return std::nullopt;
  const auto goal_segment = nearestSegment(filtered.points, goal);
  if (!goal_segment) return std::nullopt;
  const BehaviorGoalPose pre_goal{
      goal.x - std::cos(goal.yaw_rad),
      goal.y - std::sin(goal.yaw_rad), goal.yaw_rad};
  const auto pre_segment = nearestSegment(filtered.points, pre_goal);
  if (!pre_segment) return std::nullopt;
  std::optional<std::size_t> nearest_goal_index;
  double nearest_goal_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < filtered.points.size(); ++index) {
    if (std::find(filtered.lane_ids[index].begin(),
                  filtered.lane_ids[index].end(), goal_lane_id) ==
        filtered.lane_ids[index].end()) continue;
    const double current_distance = distance(filtered.points[index], goal);
    if (current_distance < search_radius_m &&
        current_distance < nearest_goal_distance) {
      nearest_goal_index = index;
      nearest_goal_distance = current_distance;
    }
  }
  if (!nearest_goal_index) return std::nullopt;
  std::size_t retained_last = *nearest_goal_index;
  for (std::size_t index = *nearest_goal_index + 1; index > 0; --index) {
    retained_last = index - 1;
    if (distance(filtered.points[retained_last], goal) >
        search_radius_m) break;
  }
  const auto interpolate = [&](std::pair<std::size_t, double> segment,
                               BehaviorGoalPose pose, bool zero_velocity) {
    auto point = filtered.points[segment.first];
    const auto &next = filtered.points[segment.first + 1];
    point.x = pose.x;
    point.y = pose.y;
    point.z += segment.second * (next.z - point.z);
    point.orientation_z = std::sin(pose.yaw_rad * 0.5);
    point.orientation_w = std::cos(pose.yaw_rad * 0.5);
    point.orientation_x = 0.0;
    point.orientation_y = 0.0;
    if (zero_velocity) {
      point.longitudinal_velocity_mps = 0.0F;
    } else {
      if (std::hypot(
              next.x - filtered.points[segment.first].x,
              next.y - filtered.points[segment.first].y) < 1.0e-6) {
        point.longitudinal_velocity_mps = next.longitudinal_velocity_mps;
      }
    }
    return point;
  };
  BehaviorGoalPath output;
  output.points.assign(filtered.points.begin(),
                       filtered.points.begin() +
                           static_cast<std::ptrdiff_t>(retained_last + 1));
  output.lane_ids.assign(filtered.lane_ids.begin(),
                         filtered.lane_ids.begin() +
                             static_cast<std::ptrdiff_t>(retained_last + 1));
  output.points.push_back(interpolate(*pre_segment, pre_goal, false));
  output.points.push_back(interpolate(*goal_segment, goal, true));
  std::vector<std::int64_t> skipped_ids;
  for (std::size_t index = retained_last + 1;
       index < filtered.lane_ids.size(); ++index) {
    for (const auto id : filtered.lane_ids[index]) {
      if (std::find(skipped_ids.begin(), skipped_ids.end(), id) ==
          skipped_ids.end()) skipped_ids.push_back(id);
    }
  }
  output.lane_ids.push_back(std::move(skipped_ids));
  output.lane_ids.push_back(filtered.lane_ids.back());
  output.left_bound = filtered.left_bound;
  output.right_bound = filtered.right_bound;
  return output;
}

}  // 익명 네임스페이스

std::optional<BehaviorGoalPath> refineBehaviorGoalPath(
    const BehaviorGoalPath &input, BehaviorGoalPose goal,
    std::int64_t goal_lane_id, double search_radius_m) {
  if (input.points.size() != input.lane_ids.size() ||
      !std::isfinite(goal.x) || !std::isfinite(goal.y) ||
      !std::isfinite(goal.yaw_rad) || !std::isfinite(search_radius_m) ||
      search_radius_m < 0.0) {
    throw std::invalid_argument("고정 목표 경로 입력이 유효하지 않습니다");
  }
  return refineFilteredGoalPath(
      removeOverlaps(input), goal, goal_lane_id, search_radius_m);
}

BehaviorGoalPath refineFixedBehaviorGoalPath(
    const BehaviorGoalPath &input, BehaviorGoalPose goal,
    std::int64_t goal_lane_id, double initial_search_radius_m,
    const std::function<bool(const BehaviorGoalPath &)> &is_valid) {
  if (input.points.size() != input.lane_ids.size() ||
      !std::isfinite(goal.x) || !std::isfinite(goal.y) ||
      !std::isfinite(goal.yaw_rad) ||
      !std::isfinite(initial_search_radius_m) ||
      initial_search_radius_m < 0.0) {
    throw std::invalid_argument("고정 목표 검색 반경이 유효하지 않습니다");
  }
  const auto filtered = removeOverlaps(input);
  BehaviorGoalPath last = filtered;
  for (double radius = initial_search_radius_m; radius >= 0.0;
       radius -= 1.0) {
    const auto refined = refineFilteredGoalPath(
        filtered, goal, goal_lane_id, radius);
    last = refined ? *refined : filtered;
    if (!is_valid || is_valid(last)) break;
  }
  return last;
}

BehaviorGoalDecisionState advanceBehaviorGoalDecisionState(
    const BehaviorGoalDecisionState &current,
    const BehaviorGoalDecisionInput &input) {
  if (!std::isfinite(input.now_s) ||
      !std::isfinite(input.keep_unsafe_time_s) ||
      !std::isfinite(input.distance_to_parking_start_m) ||
      !std::isfinite(input.decide_path_distance_m) ||
      input.keep_unsafe_time_s < 0.0 ||
      input.decide_path_distance_m < 0.0 ||
      (current.safe_start_time_s &&
       !std::isfinite(*current.safe_start_time_s)) ||
      (current.deciding_start_time_s &&
       !std::isfinite(*current.deciding_start_time_s))) {
    throw std::invalid_argument("목표 경로 결정 상태 입력이 유효하지 않습니다");
  }
  auto next = current;
  if (!input.enable_safety_check) {
    next.is_stable_safe = true;
  } else if (input.is_current_safe) {
    if (!next.safe_start_time_s) {
      next.safe_start_time_s = input.now_s;
      next.is_stable_safe = false;
    } else {
      next.is_stable_safe =
          input.now_s - *next.safe_start_time_s >
          input.keep_unsafe_time_s;
    }
  } else {
    next.safe_start_time_s.reset();
    next.is_stable_safe = false;
  }

  if (current.kind == BehaviorGoalDecisionKind::Decided) return next;
  if (!input.found_pull_over_path || !input.has_pull_over_path ||
      (input.enable_safety_check && !next.is_stable_safe &&
       !input.is_activated)) {
    next.kind = BehaviorGoalDecisionKind::NotDecided;
    return next;
  }

  if (current.kind == BehaviorGoalDecisionKind::Deciding) {
    if ((input.has_modified_goal && !input.is_goal_safe_with_margin) ||
        input.parking_path_collides) {
      next.kind = BehaviorGoalDecisionKind::NotDecided;
      next.deciding_start_time_s.reset();
      return next;
    }
    if (input.enable_safety_check && !next.is_stable_safe) {
      next.kind = BehaviorGoalDecisionKind::NotDecided;
      return next;
    }
    if (!current.deciding_start_time_s) {
      throw std::invalid_argument("주차 경로 검사 시작 시각이 없습니다");
    }
    if (input.now_s - *current.deciding_start_time_s > 1.0) {
      next.kind = BehaviorGoalDecisionKind::Decided;
      next.deciding_start_time_s.reset();
    }
    return next;
  }

  if (input.distance_to_parking_start_m > input.decide_path_distance_m) {
    next.kind = BehaviorGoalDecisionKind::NotDecided;
    return next;
  }
  if (input.use_object_recognition) {
    next.kind = BehaviorGoalDecisionKind::Deciding;
    next.deciding_start_time_s = input.now_s;
    return next;
  }
  next.kind = BehaviorGoalDecisionKind::Decided;
  next.deciding_start_time_s.reset();
  next.safe_start_time_s.reset();
  next.is_stable_safe = false;
  return next;
}

}  // planning 네임스페이스
