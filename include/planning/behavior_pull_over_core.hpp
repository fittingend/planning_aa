// SPDX-License-Identifier: Apache-2.0
// 목표 자세 앞에서 횡이동하고 두 점 스플라인으로 진입하는 주차 경로를 생성합니다.

#pragma once

#include "planning/behavior_goal_refinement.hpp"
#include "planning/behavior_path_shift.hpp"
#include "planning/crosswalk_occlusion.hpp"

#include <optional>
#include <cstdint>
#include <vector>

namespace planning {

struct ShiftPullOverParameters {
  double minimum_lateral_jerk_mps3 = 0.5;
  double maximum_lateral_jerk_mps3 = 1.0;
  int jerk_samples = 3;
  double pull_over_velocity_mps = 2.0;
  double after_shift_straight_distance_m = 5.0;
  double interpolation_interval_m = 0.5;
};

struct ShiftPullOverCandidate {
  Trajectory path;
  std::size_t shift_start_index = 0;
  std::size_t shift_end_index = 0;
  double lateral_shift_m = 0.0;
  double lateral_jerk_mps3 = 0.0;
  // 전진·후진 전환 전에 정지해야 하는 점과 마지막 정지점을 기록합니다.
  std::vector<std::size_t> phase_end_indices;
};

struct GeometricPullOverParameters {
  double wheelbase_m = 2.79;
  double maximum_steering_angle_rad = 0.4;
  // 0이면 가까운 원호와 같은 최소 회전 반경을 사용합니다.
  double far_turn_radius_m = 0.0;
  double arc_interval_m = 1.0;
  double after_straight_distance_m = 2.0;
  double forward_velocity_mps = 1.38;
  double backward_velocity_mps = -1.38;
  bool forward = true;
};

struct FreespacePullOverParameters {
  double wheelbase_m = 2.79;
  double maximum_steering_angle_rad = 0.4;
  double expansion_distance_m = 0.5;
  bool adapt_expansion_distance = false;
  bool append_exact_goal = true;
  // A* appends a laterally shifted goal only for the source node's goal exit.
  bool source_goal_shift_policy = false;
  bool stop_at_phase_changes = true;
  double velocity_mps = 1.0;
  double front_m = 3.79;
  double rear_m = 1.10;
  double half_width_m = 0.948;
  double vehicle_shape_margin_m = 1.0;
  double goal_position_tolerance_m = 0.5;
  double goal_yaw_tolerance_rad = 0.12;
  double final_straight_distance_m = 1.0;
  bool use_backward_goal_approach = false;
  double reverse_weight = 1.0;
  double curve_weight = 0.0;
  double smoothness_weight = 0.1;
  double near_goal_distance_m = 0.0;
  double goal_lateral_distance_weight = 0.0;
  double direction_change_weight = 1.5;
  double obstacle_distance_weight = 0.0;
  int theta_bins = 120;
  int obstacle_threshold = 30;
  int maximum_expansions = 200000;
  bool use_reverse = true;
  bool use_reeds_shepp_heuristic = false;
  bool use_obstacle_distance_heuristic = true;
  bool use_source_obstacle_edt = false;
  bool use_rrt_star = false;
  int rrt_iterations = 2000;
  double rrt_step_m = 1.0;
  double rrt_time_limit_ms = 0.0;
  std::uint32_t rrt_seed = 1;
};

// 원본의 저크 표본, 경로 횡이동, 목표 앞 곡선 연결 순서로 첫 유효 후보를 만듭니다.
std::optional<ShiftPullOverCandidate> generateShiftPullOverCandidate(
    const Trajectory &reference, const TrajectoryPoint &ego,
    BehaviorGoalPose goal,
    const ShiftPullOverParameters &parameters = {});

// 횡이동 경로의 시작·끝 자세 사이에서 원본의 48개 5차 베지어 표본을 만듭니다.
std::vector<ShiftPullOverCandidate> generateBezierPullOverCandidates(
    const Trajectory &reference, const TrajectoryPoint &ego,
    BehaviorGoalPose goal,
    const ShiftPullOverParameters &parameters = {});

// 평행한 도로·목표 자세 사이에 반경을 달리 지정할 수 있는 두 원호를 연결합니다.
std::optional<ShiftPullOverCandidate> generateGeometricPullOverCandidate(
    const Trajectory &reference, const TrajectoryPoint &ego,
    BehaviorGoalPose goal,
    const GeometricPullOverParameters &parameters = {});

// 곡선 도로의 시작 접선과 목표 접선 사이에 접하는 두 원호를 찾습니다.
std::optional<ShiftPullOverCandidate> generateCurvedGeometricPullOverCandidate(
    const Trajectory &reference, const TrajectoryPoint &ego,
    BehaviorGoalPose goal,
    const GeometricPullOverParameters &parameters = {});

// 점유 격자에서 차량 운동 원호를 확장해 목표 자세까지 주차 경로를 찾습니다.
std::optional<ShiftPullOverCandidate> generateFreespacePullOverCandidate(
    const TrajectoryPoint &ego, BehaviorGoalPose goal,
    const CrosswalkOccupancyGrid &grid,
    const FreespacePullOverParameters &parameters = {});

// 원본 RRT*와 Reeds–Shepp 연결 경로로 자유공간 주차 후보를 만듭니다.
std::optional<ShiftPullOverCandidate> generateRrtStarPullOverCandidate(
    const TrajectoryPoint &ego, BehaviorGoalPose goal,
    const CrosswalkOccupancyGrid &grid,
    const FreespacePullOverParameters &parameters = {});

}  // planning 네임스페이스
