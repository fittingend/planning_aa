// SPDX-License-Identifier: Apache-2.0
// 지도 경로에서 출발·회피·차로 변경·주차 행동을 선택합니다.

#pragma once

#include "planning/behavior_goal_refinement.hpp"
#include "planning/behavior_path_lane_safety.hpp"
#include "planning/behavior_path_shift.hpp"
#include "planning/behavior_static_avoidance.hpp"
#include "planning/behavior_lane_change_safety.hpp"
#include "planning/behavior_lane_change_route.hpp"
#include "planning/behavior_pull_over_core.hpp"
#include "planning/behavior_frenet_sampling.hpp"
#include "planning/behavior_dynamic_avoidance.hpp"

#include <array>
#include <optional>

namespace planning {

struct BehaviorRouteStartRequest {
  BehaviorPathShiftAnchor start;
  std::vector<BehaviorSafetyPose> start_pose_candidates;
  std::array<bool, 8> object_types_to_check{
      true, true, true, true, true, true, true, true};
  double lateral_offset_m = 0.0;
  bool use_ego_pose = true;
  bool terminal_is_goal = false;
  StartPullOutParameters parameters;
  BehaviorPathVehicleFootprint vehicle;
  bool use_geometric = false;
  bool use_geometric_fallback = false;
  GeometricPullOverParameters geometric;
  bool use_freespace = false;
  bool use_freespace_fallback = false;
  std::optional<CrosswalkOccupancyGrid> freespace_grid;
  FreespacePullOverParameters freespace;
  double freespace_join_distance_m = 10.0;
  std::vector<BehaviorSafetyPolygon> path_stop_objects;
  double object_hard_margin_m = 0.6;
};

struct BehaviorRouteSideShiftRequest {
  double requested_lateral_offset_m = 0.0;
  double current_lateral_offset_m = 0.0;
  bool use_ego_offset = true;
  bool approved = false;
  BehaviorSideShiftParameters parameters;
  double forward_path_length_m = 300.0;
  double backward_path_length_m = 5.0;
  double input_path_interval_m = 2.0;
};

struct BehaviorRouteObservedObject {
  double x = 0.0;
  double y = 0.0;
  double yaw_rad = 0.0;
  double length_m = 0.0;
  double width_m = 0.0;
  double speed_mps = 0.0;
  double stationary_duration_s = 0.0;
  std::vector<std::pair<double, double>> footprint;
  std::uint8_t classification = 1;
  bool within_crosswalk = false;
  bool within_intersection = false;
  bool on_current_road_lane = false;
  bool has_side_road_lane = false;
  double initial_displacement_m = 0.0;
};

struct BehaviorRouteAvoidancePerceptionParameters {
  double stationary_speed_mps = 1.0;
  double stationary_time_s = 0.0;
  double ambiguous_vehicle_wait_s = 3.0;
  double ambiguous_vehicle_maximum_movement_m = 1.0;
  double intersection_yaw_deviation_rad = 0.349;
  double parked_center_offset_m = 1.0;
  double parked_shiftable_ratio = 0.8;
  double minimum_road_shoulder_width_m = 0.5;
  double parked_lateral_hard_margin_m = 0.7;
  double lost_object_compensation_s = 2.0;
  double maximum_forward_distance_m = 150.0;
  double maximum_lateral_distance_m = 5.0;
  double maximum_backward_distance_m = 10.0;
  std::optional<double> goal_distance_from_ego_m;
  double goal_clearance_m = 20.0;
  bool allow_goal_modification = false;
  double vehicle_width_m = 1.896;
  double front_overhang_m = 3.79;
  double rear_overhang_m = 1.10;
  double lateral_soft_margin_m = 0.5;
  double lateral_hard_margin_m = 0.2;
  double drivable_bound_margin_m = 0.3;
  double maximum_lateral_jerk_mps3 = 3.0;
  double nominal_lateral_jerk_mps3 = 0.2;
  double minimum_distance_jerk_mps3 = 3.0;
  double nominal_avoidance_speed_mps = 8.33;
  double minimum_prepare_distance_m = 1.0;
  double maximum_prepare_time_s = 2.0;
};

// 원본 실행 설정의 객체 종류별 정차 속도·시간 임계값을 반환합니다.
double behaviorRouteAvoidanceSpeedThreshold(
    const BehaviorRouteAvoidancePerceptionParameters &parameters,
    std::uint8_t classification);
double behaviorRouteAvoidanceStopThreshold(
    const BehaviorRouteAvoidancePerceptionParameters &parameters,
    std::uint8_t classification);

struct BehaviorRouteAvoidanceRequest {
  std::vector<StaticAvoidanceSingleObjectInput> objects;
  bool approved = false;
  double ego_linear_shift_m = 0.0;
  bool use_ego_pose = true;
  bool use_perception_objects = false;
  BehaviorRouteAvoidancePerceptionParameters perception_parameters;
  BehaviorPathShiftParameters shift_parameters;
  BehaviorPathVehicleFootprint vehicle;
};

// 정차 이력이 확인된 객체와 차로 경계를 회피 코어의 입력으로 바꿉니다.
std::vector<StaticAvoidanceSingleObjectInput>
makeBehaviorRouteAvoidanceObjects(
    const Trajectory &reference, const TrajectoryPoint &ego,
    const std::vector<std::pair<double, double>> &left_boundary,
    const std::vector<std::pair<double, double>> &right_boundary,
    const std::vector<BehaviorRouteObservedObject> &objects,
    const BehaviorRouteAvoidancePerceptionParameters &parameters = {});

struct BehaviorRouteLaneChangeRequest {
  enum class ExternalDirection { None, Left, Right };
  ExternalDirection external_direction = ExternalDirection::None;
  struct AvoidanceObject {
    double longitudinal_distance_m = 0.0;
    double minimum_avoidance_distance_m = 0.0;
    std::uint8_t classification = 0;
    bool avoid_required = false;
    bool object_on_right = false;
  };
  struct AvoidanceActivation {
    std::vector<AvoidanceObject> objects;
    std::array<bool, 8> target_types{
        true, true, true, true, true, true, true, true};
    double minimum_lane_change_buffer_m = 0.0;
    double finish_judge_buffer_m = 2.0;
  };
  std::optional<AvoidanceActivation> avoidance_activation;
  std::vector<std::int64_t> target_lanelet_ids;
  Trajectory target_reference;
  std::vector<std::vector<std::int64_t>> target_reference_lane_ids;
  double prepare_length_m = 0.0;
  double shift_length_m = 0.0;
  double lateral_acceleration_mps2 = 0.5;
  struct Regulation {
    LaneChangeRegulatoryDistances distances;
    bool intersection = true;
    bool crosswalk = true;
    bool traffic_light = true;
    bool near_current_terminal_start = false;
    std::optional<double> stopped_time_s;
    double stop_time_threshold_s = 3.0;
  } regulation;
  struct Sampling {
    bool enabled = false;
    double minimum_prepare_duration_s = 2.0;
    double maximum_prepare_duration_s = 4.0;
    double maximum_vehicle_velocity_mps = 15.0;
    double minimum_change_velocity_mps = 2.78;
    double minimum_longitudinal_acceleration_mps2 = -1.0;
    double maximum_longitudinal_acceleration_mps2 = 1.0;
    int longitudinal_acceleration_samples = 5;
    double minimum_lateral_acceleration_mps2 = 0.4;
    double maximum_lateral_acceleration_mps2 = 0.65;
    int lateral_acceleration_samples = 3;
    double lateral_jerk_mps3 = 0.5;
    double deceleration_factor = 0.5;
    double signal_activation_length_m = 0.0;
    double front_overhang_m = 3.79;
    double active_signal_duration_s = 0.0;
    bool force_acceleration_sampling = false;
  } sampling;
  bool use_perception_objects = false;
  BehaviorLaneChangeCandidateSafetyInput safety;
  BehaviorLaneChangeExecutionInput execution;
  BehaviorPathVehicleFootprint vehicle;
};

// 정적 장애물 회피 대신 차로 변경을 실행할 원본의 거리 조건을 검사합니다.
bool behaviorAvoidanceByLaneChangeRequested(
    const BehaviorRouteLaneChangeRequest::AvoidanceActivation &input);

// 목표 차로까지 한 번 변경할 때 원본의 최소 저크 이동 길이와 종료 여유를 계산합니다.
double behaviorMinimumLaneChangeDistanceBuffer(
    double lateral_shift_m, double minimum_velocity_mps,
    double lateral_jerk_mps3, double maximum_lateral_acceleration_mps2,
    double finish_judge_buffer_m);

struct BehaviorRoutePullOverRequest {
  BehaviorGoalPose goal;
  std::vector<std::int64_t> goal_lanelet_ids;
  ShiftPullOverParameters parameters;
  BehaviorPathVehicleFootprint vehicle;
  bool approved = false;
  bool safe = true;
  bool use_bezier = false;
  bool use_bezier_fallback = true;
  bool use_geometric = false;
  bool use_geometric_fallback = true;
  GeometricPullOverParameters geometric;
  bool use_freespace = false;
  bool use_freespace_fallback = true;
  std::optional<CrosswalkOccupancyGrid> freespace_grid;
  FreespacePullOverParameters freespace;
  std::vector<BehaviorSafetyPolygon> path_stop_objects;
  std::vector<BehaviorSafetyPolygon> path_moving_objects;
  double object_hard_margin_m = 0.6;
  double maximum_deceleration_mps2 = 1.0;
  double maximum_extra_stopping_margin_m = 1.0;
};

struct BehaviorRouteSamplingRequest {
  BehaviorFrenetSamplingInput sampling;
  Trajectory previous_path;
  int previous_path_reuse_points = 2;
  bool require_unsafe_reference = true;
  BehaviorPathVehicleFootprint vehicle;
  std::vector<BehaviorSafetyPolygon> obstacle_polygons;
  double maximum_curvature_per_m = 3.0;
  double object_hard_margin_m = 0.0;
  std::array<double, 3> cost_weights{0.5, 1.0, 20.0};
};

struct BehaviorRouteDynamicAvoidanceRequest {
  BehaviorDynamicAvoidanceParameters parameters;
};

struct BehaviorRouteActionInput {
  std::optional<BehaviorRouteStartRequest> start;
  std::optional<BehaviorRouteSideShiftRequest> side_shift;
  std::optional<BehaviorRouteAvoidanceRequest> avoidance;
  std::optional<BehaviorRouteLaneChangeRequest> lane_change;
  std::optional<BehaviorRoutePullOverRequest> pull_over;
  std::optional<BehaviorRouteSamplingRequest> sampling;
  std::optional<BehaviorRouteDynamicAvoidanceRequest> dynamic_avoidance;
};

enum class BehaviorRouteActionKind {
  LaneFollow,
  StartPullOut,
  SideShift,
  StaticAvoidance,
  LaneChange,
  LaneChangeAbort,
  LaneChangeStop,
  PullOver,
  Sampling,
};

struct BehaviorRouteActionResult {
  BehaviorGoalPath path;
  BehaviorRouteActionKind kind = BehaviorRouteActionKind::LaneFollow;
  std::size_t start_candidate_count = 0;
  std::size_t side_shift_candidate_count = 0;
  std::optional<std::size_t> side_shift_end_index;
  std::optional<BehaviorSafetyPose> selected_start_pose;
  // 전체 자유공간 출발 경로 기준 인덱스입니다.
  std::vector<std::size_t> start_phase_end_indices;
  std::size_t start_current_phase = 0;
  std::size_t avoidance_candidate_count = 0;
  std::size_t lane_change_candidate_count = 0;
  std::size_t lane_change_collision_count = 0;
  std::size_t lane_change_other_unsafe_count = 0;
  std::size_t pull_over_candidate_count = 0;
  std::size_t sampling_candidate_count = 0;
  // 전체 승인 경로 기준 인덱스입니다. 단계 출력 경로의 지역 인덱스와 다릅니다.
  std::vector<std::size_t> pull_over_phase_end_indices;
  std::size_t pull_over_current_phase = 0;
  std::optional<std::size_t> lane_change_first_safe_index;
  std::optional<std::size_t> lane_change_shift_end_index;
  std::optional<BehaviorLaneChangeFailureDecision> lane_change_decision;
  bool lane_change_candidate_outside_lanes = false;
  std::vector<StaticAvoidanceAnchoredShiftArc> registered_shift_arcs;
};

struct BehaviorPullOverPhaseState {
  std::size_t index = 0;
  std::optional<double> phase_start_s;
  std::optional<BehaviorGoalPose> goal;
};

using BehaviorRoutePhaseState = BehaviorPullOverPhaseState;

// 정차 상태와 구간 끝 도착을 확인한 뒤 다음 주차 구간을 출력합니다.
BehaviorGoalPath selectBehaviorPullOverPhase(
    const BehaviorGoalPath &full_path,
    const std::vector<std::size_t> &phase_end_indices,
    const TrajectoryPoint &ego, double ego_speed_mps,
    double timestamp_s, BehaviorGoalPose goal,
    BehaviorPullOverPhaseState &state,
    double arrival_distance_m = 1.0,
    double minimum_phase_duration_s = 4.0);

// 자유공간 출발 경로의 후진·전진 구간을 정차 조건에 따라 선택합니다.
BehaviorGoalPath selectBehaviorStartPhase(
    const BehaviorGoalPath &full_path,
    const std::vector<std::size_t> &phase_end_indices,
    const TrajectoryPoint &ego, double ego_speed_mps,
    double timestamp_s, BehaviorGoalPose goal,
    BehaviorRoutePhaseState &state,
    bool terminal_is_goal = false,
    double arrival_distance_m = 1.0,
    double minimum_phase_duration_s = 4.0);

// 자차 앞에 남은 주차 경로와 정지 객체 외곽의 충돌 여부를 검사합니다.
bool behaviorPullOverPathCollides(
    const Trajectory &path, const TrajectoryPoint &ego,
    const BehaviorPathVehicleFootprint &vehicle,
    const std::vector<BehaviorSafetyPolygon> &objects,
    double margin_m, double maximum_deceleration_mps2 = 1.0,
    double maximum_extra_stopping_margin_m = 0.0);

// 새 회피 이동선은 승인된 경우에만 등록합니다. 이전 등록선은 매 콜백 재투영합니다.
BehaviorRouteActionResult composeBehaviorRouteAction(
    const BehaviorGoalPath &reference, const TrajectoryPoint &ego,
    const std::vector<std::vector<BehaviorPathPoint2d>> &lane_polygons,
    const BehaviorRouteActionInput &input,
    const std::vector<StaticAvoidanceAnchoredShiftArc> &registered = {});

}  // planning 네임스페이스
