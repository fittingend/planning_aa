// SPDX-License-Identifier: Apache-2.0
// 원본 목표 검색기의 후보 갱신과 승인 전 재검사 분기를 계산합니다.

#include "planning/behavior_goal_safety.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {
namespace {

BehaviorGoalPose along(const BehaviorGoalPose &pose, double offset_m) {
  return {pose.x + offset_m * std::cos(pose.yaw_rad),
          pose.y + offset_m * std::sin(pose.yaw_rad),
          pose.yaw_rad};
}

double distanceToInterval(double value, double minimum, double maximum) {
  return value < minimum ? minimum - value :
         value > maximum ? value - maximum : 0.0;
}

}  // 익명 네임스페이스

BehaviorGoalSafetyReason evaluateBehaviorGoalSafety(
    const BehaviorGoalSafetyInput &input,
    const std::function<bool(BehaviorGoalPose)> &occupancy_collision) {
  if (!std::isfinite(input.goal.x) || !std::isfinite(input.goal.y) ||
      !std::isfinite(input.goal.yaw_rad) ||
      !std::isfinite(input.vehicle.front_m) ||
      !std::isfinite(input.vehicle.rear_m) ||
      !std::isfinite(input.vehicle.half_width_m) ||
      input.vehicle.front_m <= 0.0 || input.vehicle.rear_m < 0.0 ||
      input.vehicle.half_width_m <= 0.0 ||
      !std::isfinite(input.object_hard_margin_m) ||
      !std::isfinite(input.longitudinal_margin_m) ||
      !std::isfinite(input.occupancy_grid_margin_m) ||
      !std::isfinite(input.margin_scale_factor) ||
      input.object_hard_margin_m < 0.0 ||
      input.longitudinal_margin_m < 0.0 ||
      input.occupancy_grid_margin_m < 0.0 ||
      input.margin_scale_factor < 0.0 ||
      (input.use_occupancy_grid && !occupancy_collision &&
       !(input.mode == BehaviorGoalSafetyMode::Recheck &&
         !input.use_object_recognition))) {
    throw std::invalid_argument("목표 후보 안전 입력이 유효하지 않습니다");
  }
  if (input.mode == BehaviorGoalSafetyMode::Recheck &&
      !input.use_object_recognition) {
    return BehaviorGoalSafetyReason::Safe;
  }
  const double margin = input.object_hard_margin_m *
                        input.margin_scale_factor;
  const BehaviorSafetyPose vehicle_pose{
      0.0, input.goal.x, input.goal.y, input.goal.yaw_rad, 0.0};

  if (input.mode == BehaviorGoalSafetyMode::CandidateUpdate &&
      input.use_occupancy_grid && occupancy_collision(input.goal)) {
    return BehaviorGoalSafetyReason::GoalGridCollision;
  }
  if (input.use_object_recognition &&
      behaviorFootprintCollidesWithObjects(
          vehicle_pose, input.vehicle, input.object_polygons, margin)) {
    return BehaviorGoalSafetyReason::FootprintObjectCollision;
  }

  if (input.use_occupancy_grid &&
      input.use_occupancy_for_longitudinal) {
    const double offset = std::max(
        input.longitudinal_margin_m - input.occupancy_grid_margin_m,
        0.0);
    if (occupancy_collision(along(input.goal, offset)) ||
        occupancy_collision(along(input.goal, -offset))) {
      return BehaviorGoalSafetyReason::LongitudinalGridCollision;
    }
  }

  if (input.use_object_recognition) {
    double minimum_longitudinal = std::numeric_limits<double>::infinity();
    const double cosine = std::cos(input.goal.yaw_rad);
    const double sine = std::sin(input.goal.yaw_rad);
    for (const auto &polygon : input.object_polygons) {
      if (polygon.size() < 3) {
        throw std::invalid_argument("목표 후보 객체 다각형이 유효하지 않습니다");
      }
      double lateral_distance = std::numeric_limits<double>::infinity();
      double longitudinal_distance =
          std::numeric_limits<double>::infinity();
      for (const auto &[x, y] : polygon) {
        if (!std::isfinite(x) || !std::isfinite(y)) {
          throw std::invalid_argument("목표 후보 객체 좌표가 유효하지 않습니다");
        }
        const double dx = x - input.goal.x;
        const double dy = y - input.goal.y;
        const double longitudinal = dx * cosine + dy * sine;
        const double lateral = -dx * sine + dy * cosine;
        lateral_distance = std::min(lateral_distance,
            distanceToInterval(
                lateral, -input.vehicle.half_width_m,
                input.vehicle.half_width_m));
        longitudinal_distance = std::min(longitudinal_distance,
            distanceToInterval(
                longitudinal, -input.vehicle.rear_m,
                input.vehicle.front_m));
      }
      if (lateral_distance < margin) {
        minimum_longitudinal = std::min(
            minimum_longitudinal, longitudinal_distance);
      }
    }
    if (minimum_longitudinal < input.longitudinal_margin_m) {
      return BehaviorGoalSafetyReason::LongitudinalObjectTooClose;
    }
  }
  return BehaviorGoalSafetyReason::Safe;
}

}  // planning 네임스페이스
