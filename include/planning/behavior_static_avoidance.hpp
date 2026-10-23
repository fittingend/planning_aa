// SPDX-License-Identifier: Apache-2.0
// 정적 장애물 회피의 측방 여유 거리와 이동 방향을 계산합니다.

#pragma once

#include "planning/behavior_path_shift.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace planning {

struct StaticAvoidanceOverhangPoint {
  double lateral_m = 0.0;
  double x = 0.0;
  double y = 0.0;
};

// 기준 경로 방향의 객체 외접 직사각형에 여유를 더합니다.
std::vector<std::pair<double, double>> staticAvoidanceEnvelopePolygon(
    const std::vector<std::pair<double, double>> &object_polygon,
    const TrajectoryPoint &closest_pose,
    double buffer_m);

struct StaticAvoidanceLongitudinalSpan {
  double start_from_ego_m = 0.0;
  double length_m = 0.0;
};

// 자차와 객체 외피를 경로 선분에 투영해 시작 거리와 길이를 구합니다.
std::optional<StaticAvoidanceLongitudinalSpan>
staticAvoidanceEnvelopeLongitudinalSpan(
    const Trajectory &reference,
    double ego_x, double ego_y,
    const std::vector<std::pair<double, double>> &envelope);

// 객체 외피 각 점의 최근접 기준 경로 자세에서 측방 거리를 계산합니다.
std::vector<StaticAvoidanceOverhangPoint>
staticAvoidanceEnvelopeOverhangDistances(
    const Trajectory &reference,
    const std::vector<std::pair<double, double>> &envelope,
    bool object_on_right);

struct StaticAvoidanceBoundaryDistance {
  double distance_m = 0.0;
  std::optional<std::pair<double, double>> envelope_point;
  std::optional<std::pair<double, double>> boundary_point;
};

struct StaticAvoidanceBoundaryInput {
  // 오른쪽 객체면 왼쪽, 왼쪽 객체면 오른쪽 주행 가능 경계를 넣습니다.
  std::vector<std::pair<double, double>> opposite_boundary;
  double centerline_yaw_rad = 0.0;
};

struct StaticAvoidanceObjectGeometry {
  std::vector<std::pair<double, double>> envelope;
  StaticAvoidanceLongitudinalSpan longitudinal;
  std::vector<StaticAvoidanceOverhangPoint> overhang_points;
  StaticAvoidanceOverhangPoint nearest_overhang;
  std::optional<StaticAvoidanceBoundaryDistance> boundary_distance;
  double lateral_margin_m = 0.0;
  double desired_shift_m = 0.0;
};

// 외피 돌출점에서 회피 방향의 주행 가능 경계까지 최좁은 거리를 구합니다.
StaticAvoidanceBoundaryDistance staticAvoidanceRoadShoulderDistance(
    const std::vector<StaticAvoidanceOverhangPoint> &overhang_points,
    double envelope_area_m2, double object_length_m,
    const std::vector<std::pair<double, double>> &opposite_boundary,
    double centerline_yaw_rad, bool object_on_right);

struct StaticAvoidanceMarginInput {
  double vehicle_width_m = 0.0;
  double road_shoulder_distance_m = 0.0;
  double distance_factor = 1.0;
  double lateral_hard_margin_m = 0.0;
  double parked_lateral_hard_margin_m = 0.0;
  double lateral_soft_margin_m = 0.0;
  double soft_drivable_bound_margin_m = 0.0;
  double hard_drivable_bound_margin_m = 0.0;
  bool parked = false;
};

// 차로 경계의 강제 여유가 부족하면 회피 후보를 만들지 않습니다.
std::optional<double> staticAvoidanceLateralMargin(
    const StaticAvoidanceMarginInput &input);

// 한 객체의 외피·호길이·돌출점과 요구 횡이동량을 순서대로 계산합니다.
std::optional<StaticAvoidanceObjectGeometry>
staticAvoidanceObjectGeometry(
    const Trajectory &reference, double ego_x, double ego_y,
    const std::vector<std::pair<double, double>> &object_polygon,
    const TrajectoryPoint &closest_reference_pose,
    double envelope_buffer_m, bool object_on_right,
    const StaticAvoidanceMarginInput &margin_input,
    const std::optional<StaticAvoidanceBoundaryInput> &boundary = std::nullopt);

// 장애물 오른쪽은 양수, 왼쪽은 음수 이동을 요구합니다.
double staticAvoidanceShiftLength(
    bool object_on_right, double overhang_m, double lateral_margin_m);
bool staticAvoidanceShiftNecessary(
    bool object_on_right, double shift_length_m);

struct StaticAvoidanceFeasibilityInput {
  double desired_shift_m = 0.0;
  double current_ego_shift_m = 0.0;
  double object_longitudinal_m = 0.0;
  double front_constant_distance_m = 0.0;
  double prepare_distance_m = 0.0;
  double nominal_avoidance_distance_m = 0.0;
  double avoidance_speed_mps = 0.0;
  double maximum_lateral_jerk_mps3 = 0.0;
  double minimum_distance_jerk_mps3 = 0.0;
  double minimum_distance_speed_mps = 0.0;
  double lateral_execution_threshold_m = 0.0;
  double vehicle_width_m = 0.0;
  double lateral_hard_margin_m = 0.0;
  double overhang_m = 0.0;
  bool object_on_right = false;
  bool best_effort_lateral_margin = false;
  bool best_effort_deceleration = false;
  bool stoppable = false;
  bool shorten_margin_immediately = false;
  bool already_approved = false;
};

enum class StaticAvoidanceFeasibilityReason {
  Feasible,
  InsufficientLongitudinalDistance,
  NeedsDeceleration,
  BelowExecutionThreshold,
};

struct StaticAvoidanceFeasibilityResult {
  StaticAvoidanceFeasibilityReason reason =
      StaticAvoidanceFeasibilityReason::Feasible;
  double shift_m = 0.0;
  double avoidance_distance_m = 0.0;
};

// 객체별 횡이동량을 준비 거리·저크·측방 강제 여유에 맞춰 결정합니다.
StaticAvoidanceFeasibilityResult staticAvoidanceFeasibleShift(
    const StaticAvoidanceFeasibilityInput &input);

struct StaticAvoidanceOutlineInput {
  double object_longitudinal_m = 0.0;
  double front_constant_distance_m = 0.0;
  double rear_constant_distance_m = 0.0;
  double avoidance_distance_m = 0.0;
  double minimum_avoidance_distance_m = 0.0;
  double maximum_return_distance_m = 0.0;
  double to_start_point_m = 0.0;
  double to_return_point_m = 0.0;
  double current_start_shift_m = 0.0;
  double target_shift_m = 0.0;
  bool skip_return = false;
};

struct StaticAvoidanceShiftArc {
  double start_m = 0.0;
  double end_m = 0.0;
  double start_shift_m = 0.0;
  double end_shift_m = 0.0;
};

struct StaticAvoidanceAnchoredShiftArc {
  double start_x = 0.0;
  double start_y = 0.0;
  double end_x = 0.0;
  double end_y = 0.0;
  double start_shift_m = 0.0;
  double end_shift_m = 0.0;
};

// 등록된 회피 이동선에서 현재 자차 위치의 횡이동량을 복원합니다.
double staticAvoidanceRegisteredShiftAtEgo(
    const Trajectory &reference, std::size_t ego_index,
    const std::vector<StaticAvoidanceAnchoredShiftArc> &registered);

// 이전 경로의 등록 이동선을 현재 경로의 최근접 점에 다시 배치하고 검사합니다.
std::vector<StaticAvoidanceShiftArc>
reprojectStaticAvoidanceRegisteredShiftArcs(
    const Trajectory &reference, std::size_t ego_index,
    double ego_linear_shift_m,
    const std::vector<StaticAvoidanceAnchoredShiftArc> &registered);

struct StaticAvoidanceRegisteredPlan {
  Trajectory path;
  std::vector<StaticAvoidanceShiftArc> active_shift_arcs;
  std::vector<StaticAvoidanceAnchoredShiftArc> registered_shift_arcs;
};

// 이전 승인 이동선을 현재 경로에 재투영하고 새 이동선은 승인 후 등록합니다.
std::optional<StaticAvoidanceRegisteredPlan>
planStaticAvoidanceWithRegisteredShifts(
    const Trajectory &reference, std::size_t ego_index,
    double ego_linear_shift_m,
    const std::vector<StaticAvoidanceAnchoredShiftArc> &registered,
    const std::vector<StaticAvoidanceShiftArc> &proposed,
    bool approved,
    const BehaviorPathShiftParameters &shift_parameters = {});

struct StaticAvoidanceOutline {
  StaticAvoidanceShiftArc avoid;
  std::optional<StaticAvoidanceShiftArc> return_to_center;
};

struct StaticAvoidanceMergedShiftProfile {
  std::vector<double> positive_m;
  std::vector<double> negative_m;
  std::vector<double> combined_m;
};

// 여러 객체 이동선의 같은 방향 최대값과 반대 방향 최소값을 점별로 합칩니다.
StaticAvoidanceMergedShiftProfile mergeStaticAvoidanceShiftArcs(
    const std::vector<double> &arcs_from_ego_m,
    std::size_t ego_index, double current_shift_m,
    const std::vector<StaticAvoidanceShiftArc> &shift_arcs);

// 점별 횡이동량에서 앞뒤 기울기의 변화점을 찾아 이동선 구간을 다시 만듭니다.
std::vector<StaticAvoidanceShiftArc> extractStaticAvoidanceShiftArcs(
    const std::vector<double> &arcs_from_ego_m,
    std::size_t ego_index,
    const std::vector<double> &shift_profile_m,
    double gradient_change_threshold = 0.001);

struct StaticAvoidanceTrimParameters {
  // 자차 속도×최소 준비 시간과 최소 준비 거리 중 큰 값을 호출자가 제공합니다.
  double minimum_prepare_distance_m = 0.0;
  double minimum_arc_length_m = 1.0;
  double quantize_size_m = 0.1;
  double similar_gradient_first_m = 0.1;
  double similar_gradient_second_m = 0.2;
  double similar_gradient_third_m = 0.5;
};

// 짧은 구간 제거·기울기 병합·종점 양자화를 원본 순서로 적용합니다.
std::vector<StaticAvoidanceShiftArc> trimStaticAvoidanceShiftArcs(
    std::vector<StaticAvoidanceShiftArc> shift_arcs,
    const StaticAvoidanceTrimParameters &parameters = {});

// 같은 기준 경로 위의 새 이동선이 이전 이동선과 겹치면 원본 규칙으로 교체합니다.
std::vector<StaticAvoidanceShiftArc> mergeStaticAvoidanceRegisteredShiftArcs(
    const std::vector<StaticAvoidanceShiftArc> &registered,
    const std::vector<StaticAvoidanceShiftArc> &proposed);

// 객체별 계산을 마친 거리를 받아 회피·복귀 이동선의 호길이를 정합니다.
std::optional<StaticAvoidanceOutline> staticAvoidanceShiftOutline(
    const StaticAvoidanceOutlineInput &input);

// 경로 시작점에서 자차까지의 호길이를 받아 회피·복귀 곡선을 적용합니다.
std::optional<Trajectory> generateStaticAvoidancePath(
    const Trajectory &reference, double ego_arc_m,
    const StaticAvoidanceOutline &outline,
    const BehaviorPathShiftParameters &parameters = {});

// 여러 회피·복귀 이동선을 공통 경로의 점 인덱스로 바꿔 적용합니다.
std::optional<Trajectory> generateStaticAvoidanceMergedPath(
    const Trajectory &reference, double ego_arc_m,
    const std::vector<StaticAvoidanceShiftArc> &shift_arcs,
    const BehaviorPathShiftParameters &parameters = {});

struct StaticAvoidanceMultiObjectPlan {
  StaticAvoidanceMergedShiftProfile profile;
  std::vector<StaticAvoidanceShiftArc> shift_arcs;
  Trajectory path;
};

// 제공된 객체별 이동선을 병합·정리한 뒤 공통 경로에 적용합니다.
std::optional<StaticAvoidanceMultiObjectPlan> planStaticAvoidanceMultiObject(
    const Trajectory &reference, std::size_t ego_index,
    double ego_arc_m, double current_shift_m,
    const std::vector<StaticAvoidanceShiftArc> &raw_shift_arcs,
    const StaticAvoidanceTrimParameters &trim_parameters = {},
    const BehaviorPathShiftParameters &shift_parameters = {});

struct StaticAvoidanceSingleObjectInput {
  double ego_x = 0.0;
  double ego_y = 0.0;
  std::vector<std::pair<double, double>> object_polygon;
  TrajectoryPoint closest_reference_pose;
  double envelope_buffer_m = 0.0;
  bool object_on_right = false;
  StaticAvoidanceMarginInput margin;
  std::optional<StaticAvoidanceBoundaryInput> boundary;
  // 객체 거리·위치·돌출량·요구 이동량과 차량 폭·강제 여유는 앞단 결과로 채웁니다.
  StaticAvoidanceFeasibilityInput feasibility;
  // 객체 거리·회피 가능 거리·최소 거리·목표 이동량은 앞단 결과로 채웁니다.
  StaticAvoidanceOutlineInput outline;
  BehaviorPathShiftParameters path_shift;
  std::uint8_t classification = 0;
};

struct StaticAvoidanceSingleObjectPlan {
  StaticAvoidanceObjectGeometry geometry;
  StaticAvoidanceFeasibilityResult feasibility;
  StaticAvoidanceOutline outline;
  Trajectory path;
};

// 호출자가 지도 경계와 객체별 속도·저크·거리 제약을 채운 한 객체의 경로입니다.
std::optional<StaticAvoidanceSingleObjectPlan>
planStaticAvoidanceSingleObject(
    const Trajectory &reference,
    const StaticAvoidanceSingleObjectInput &input);

}  // planning 네임스페이스
