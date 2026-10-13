// SPDX-License-Identifier: Apache-2.0
// 고정 목표 경로의 종점 연결과 검색 반경 축소를 ROS 없이 계산합니다.

#pragma once

#include "planning/trajectory.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace planning {

struct BehaviorGoalBoundaryPoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct BehaviorGoalPath {
  Trajectory points;
  std::vector<std::vector<std::int64_t>> lane_ids;
  std::vector<BehaviorGoalBoundaryPoint> left_bound;
  std::vector<BehaviorGoalBoundaryPoint> right_bound;
};

struct BehaviorGoalPose {
  double x = 0.0;
  double y = 0.0;
  double yaw_rad = 0.0;
};

// 목표는 지도 안팎 판정을 거쳐 보정한 자세로 전달합니다.
std::optional<BehaviorGoalPath> refineBehaviorGoalPath(
    const BehaviorGoalPath &input, BehaviorGoalPose goal,
    std::int64_t goal_lane_id, double search_radius_m);

// 원본처럼 반경을 1m씩 줄이고, 모두 무효이면 마지막 계산 경로를 반환합니다.
BehaviorGoalPath refineFixedBehaviorGoalPath(
    const BehaviorGoalPath &input, BehaviorGoalPose goal,
    std::int64_t goal_lane_id, double initial_search_radius_m,
    const std::function<bool(const BehaviorGoalPath &)> &is_valid = {});

enum class BehaviorGoalDecisionKind { NotDecided, Deciding, Decided };

struct BehaviorGoalDecisionState {
  BehaviorGoalDecisionKind kind = BehaviorGoalDecisionKind::NotDecided;
  std::optional<double> deciding_start_time_s;
  bool is_stable_safe = false;
  std::optional<double> safe_start_time_s;
};

struct BehaviorGoalDecisionInput {
  double now_s = 0.0;
  bool found_pull_over_path = false;
  bool has_pull_over_path = false;
  bool enable_safety_check = false;
  bool is_current_safe = false;
  double keep_unsafe_time_s = 0.0;
  bool is_activated = false;
  bool has_modified_goal = false;
  bool is_goal_safe_with_margin = true;
  bool parking_path_collides = false;
  double distance_to_parking_start_m = 0.0;
  double decide_path_distance_m = 0.0;
  bool use_object_recognition = false;
};

// 목표·주차 후보의 안전 검사는 앞단에서 수행하고 상태 전환만 계산합니다.
BehaviorGoalDecisionState advanceBehaviorGoalDecisionState(
    const BehaviorGoalDecisionState &current,
    const BehaviorGoalDecisionInput &input);

}  // planning 네임스페이스
