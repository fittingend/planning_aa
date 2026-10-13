#include "planning/behavior_static_avoidance.hpp"
#include "planning/behavior_path_object_safety.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

void check(bool value, const char *message) {
  if (!value) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

bool near(double first, double second) {
  return std::abs(first - second) < 1.0e-9;
}

}  // 익명 네임스페이스

int main() {
  planning::StaticAvoidanceMarginInput input;
  input.vehicle_width_m = 2.0;
  input.distance_factor = 1.0;
  input.lateral_hard_margin_m = 0.2;
  input.parked_lateral_hard_margin_m = 0.7;
  input.lateral_soft_margin_m = 0.7;
  input.soft_drivable_bound_margin_m = 0.5;
  input.hard_drivable_bound_margin_m = 0.1;
  input.road_shoulder_distance_m = 5.0;
  const auto wide = planning::staticAvoidanceLateralMargin(input);
  input.road_shoulder_distance_m = 3.0;
  const auto narrow = planning::staticAvoidanceLateralMargin(input);
  input.road_shoulder_distance_m = 2.5;
  const auto constrained = planning::staticAvoidanceLateralMargin(input);
  input.parked = true;
  const auto blocked = planning::staticAvoidanceLateralMargin(input);
  check(wide && narrow && constrained &&
            near(*wide, 1.9) && near(*narrow, 1.5) &&
            near(*constrained, 1.2) && !blocked,
        "원본의 넓은 차로·연한 경계·단단한 경계·주차 차량 사례를 재현합니다");
  input.parked = false;
  input.lateral_hard_margin_m = -0.2;
  input.parked_lateral_hard_margin_m = -0.2;
  input.road_shoulder_distance_m = 5.0;
  const auto unknown_margin = planning::staticAvoidanceLateralMargin(input);
  check(unknown_margin && near(*unknown_margin, 1.5),
        "원본 설정의 미분류 객체 음수 하드 여유를 계산합니다");
  input.lateral_hard_margin_m = 0.2;
  input.parked_lateral_hard_margin_m = 0.7;
  check(near(planning::staticAvoidanceShiftLength(true, -1.0, 1.5), 0.5) &&
            near(planning::staticAvoidanceShiftLength(false, -1.0, 1.5),
                 -2.5) &&
            planning::staticAvoidanceShiftNecessary(true, 0.5) &&
            !planning::staticAvoidanceShiftNecessary(true, -0.5) &&
            planning::staticAvoidanceShiftNecessary(false, -0.5) &&
            !planning::staticAvoidanceShiftNecessary(false, 0.5),
        "장애물 좌우 위치에 맞는 이동량과 방향을 선택합니다");
  planning::StaticAvoidanceFeasibilityInput feasibility;
  feasibility.desired_shift_m = 2.0;
  feasibility.object_longitudinal_m = 30.0;
  feasibility.front_constant_distance_m = 5.0;
  feasibility.prepare_distance_m = 5.0;
  feasibility.nominal_avoidance_distance_m = 10.0;
  feasibility.avoidance_speed_mps = 5.0;
  feasibility.maximum_lateral_jerk_mps3 = 0.5;
  feasibility.minimum_distance_jerk_mps3 = 1.0;
  feasibility.minimum_distance_speed_mps = 1.0;
  feasibility.lateral_execution_threshold_m = 0.01;
  feasibility.vehicle_width_m = 2.0;
  feasibility.lateral_hard_margin_m = 0.05;
  feasibility.overhang_m = -1.0;
  feasibility.object_on_right = true;
  feasibility.best_effort_lateral_margin = true;
  auto shift_profile = planning::staticAvoidanceFeasibleShift(feasibility);
  check(shift_profile.reason ==
            planning::StaticAvoidanceFeasibilityReason::Feasible &&
            near(shift_profile.shift_m, 2.0) &&
            near(shift_profile.avoidance_distance_m, 10.0),
        "명목 이동 거리가 충분하면 희망 이동량을 유지합니다");
  feasibility.object_longitudinal_m = 18.0;
  shift_profile = planning::staticAvoidanceFeasibleShift(feasibility);
  check(shift_profile.reason ==
            planning::StaticAvoidanceFeasibilityReason::NeedsDeceleration,
        "짧은 거리에서 저크가 넘으면 감속을 요청합니다");
  feasibility.best_effort_deceleration = true;
  shift_profile = planning::staticAvoidanceFeasibleShift(feasibility);
  check(shift_profile.reason ==
            planning::StaticAvoidanceFeasibilityReason::Feasible &&
            near(shift_profile.shift_m, 0.063) &&
            near(shift_profile.avoidance_distance_m, 8.0),
        "최선 노력 분기에서 저크에 맞는 작은 횡이동량을 계산합니다");
  feasibility.lateral_hard_margin_m = 0.1;
  shift_profile = planning::staticAvoidanceFeasibleShift(feasibility);
  check(shift_profile.reason ==
            planning::StaticAvoidanceFeasibilityReason::NeedsDeceleration,
        "줄인 횡이동량이 강제 여유보다 작으면 원본처럼 감속을 요청합니다");
  feasibility.lateral_hard_margin_m = 0.05;
  feasibility.object_longitudinal_m = 9.0;
  shift_profile = planning::staticAvoidanceFeasibleShift(feasibility);
  check(shift_profile.reason ==
            planning::StaticAvoidanceFeasibilityReason::InsufficientLongitudinalDistance,
        "준비 거리가 없으면 이동선을 만들지 않습니다");
  feasibility.object_longitudinal_m = 18.0;
  feasibility.stoppable = true;
  feasibility.best_effort_deceleration = false;
  shift_profile = planning::staticAvoidanceFeasibleShift(feasibility);
  check(shift_profile.reason ==
            planning::StaticAvoidanceFeasibilityReason::Feasible &&
            near(shift_profile.shift_m, 2.0),
        "정지 가능한 객체의 여유를 즉시 줄이지 않습니다");
  planning::StaticAvoidanceOutlineInput outline_input;
  outline_input.object_longitudinal_m = 30.0;
  outline_input.front_constant_distance_m = 5.0;
  outline_input.rear_constant_distance_m = 5.0;
  outline_input.avoidance_distance_m = 10.0;
  outline_input.minimum_avoidance_distance_m = 5.0;
  outline_input.maximum_return_distance_m = 10.0;
  outline_input.to_start_point_m = 12.0;
  outline_input.to_return_point_m = 50.0;
  outline_input.target_shift_m = 2.0;
  const auto outline = planning::staticAvoidanceShiftOutline(outline_input);
  check(outline && near(outline->avoid.start_m, 15.0) &&
            near(outline->avoid.end_m, 25.0) &&
            near(outline->avoid.end_shift_m, 2.0) &&
            outline->return_to_center &&
            near(outline->return_to_center->start_m, 35.0) &&
            near(outline->return_to_center->end_m, 45.0),
        "객체 앞에서 횡이동을 끝내고 뒤에서 최대 복귀 거리까지 돌아옵니다");
  planning::Trajectory reference(61);
  for (std::size_t index = 0; index < reference.size(); ++index) {
    reference[index].x = static_cast<double>(index);
  }
  const auto envelope = planning::staticAvoidanceEnvelopePolygon(
      {{10.0, -2.0}, {12.0, -2.0}, {11.0, -1.0}},
      reference[10], 0.5);
  check(envelope.size() == 5 &&
            near(envelope[0].first, 9.5) &&
            near(envelope[0].second, -2.5) &&
            near(envelope[2].first, 12.5) &&
            near(envelope[2].second, -0.5),
        "경로 방향 외접 직사각형에 객체 여유를 더합니다");
  const auto buffered_overhang =
      planning::staticAvoidanceEnvelopeOverhangDistances(
          reference, envelope, true);
  check(near(buffered_overhang.front().lateral_m, -0.5),
        "객체 외피 여유를 돌출량 계산에 이어 적용합니다");
  auto turned_pose = reference[10];
  turned_pose.orientation_z = std::sin(std::acos(-1.0) * 0.25);
  turned_pose.orientation_w = std::cos(std::acos(-1.0) * 0.25);
  const auto turned_envelope = planning::staticAvoidanceEnvelopePolygon(
      {{8.0, 1.0}, {8.0, 3.0}, {9.0, 2.0}}, turned_pose, 0.0);
  check(turned_envelope.size() == 5 &&
            near(turned_envelope[0].first, 9.0) &&
            near(turned_envelope[0].second, 1.0) &&
            near(turned_envelope[2].first, 8.0) &&
            near(turned_envelope[2].second, 3.0),
        "회전한 기준 경로 방향에서 객체 외접 직사각형을 만듭니다");
  const auto longitudinal =
      planning::staticAvoidanceEnvelopeLongitudinalSpan(
          reference, 5.0, 0.0,
          {{20.0, -2.0}, {24.0, -2.0},
           {24.0, -1.0}, {20.0, -1.0}});
  check(longitudinal &&
            near(longitudinal->start_from_ego_m, 15.0) &&
            near(longitudinal->length_m, 4.0),
        "객체 외피의 앞·뒤 끝을 자차 기준 경로 호길이로 계산합니다");
  const auto beyond_path =
      planning::staticAvoidanceEnvelopeLongitudinalSpan(
          reference, -2.0, 0.0,
          {{62.0, -1.0}, {64.0, -1.0}});
  check(beyond_path &&
            near(beyond_path->start_from_ego_m, 64.0) &&
            near(beyond_path->length_m, 2.0),
        "경로 앞뒤 바깥 객체도 원본처럼 끝 선분을 연장해 투영합니다");
  input.parked = false;
  input.road_shoulder_distance_m = 5.0;
  const auto object_geometry = planning::staticAvoidanceObjectGeometry(
      reference, 5.0, 0.0,
      {{20.0, -2.0}, {24.0, -2.0},
       {24.0, -1.0}, {20.0, -1.0}},
      reference[20], 0.5, true, input);
  check(object_geometry && object_geometry->envelope.size() == 5 &&
            near(object_geometry->longitudinal.start_from_ego_m, 14.5) &&
            near(object_geometry->longitudinal.length_m, 5.0) &&
            near(object_geometry->nearest_overhang.lateral_m, -0.5) &&
            near(object_geometry->lateral_margin_m, 1.9) &&
            near(object_geometry->desired_shift_m, 1.4),
        "객체 외피에서 회피 요구 이동량까지 한 번에 계산합니다");
  const auto clear_boundary =
      planning::staticAvoidanceRoadShoulderDistance(
          object_geometry->overhang_points, 10.0,
          object_geometry->longitudinal.length_m,
          {{0.0, 3.0}, {60.0, 3.0}}, 0.0, true);
  check(near(clear_boundary.distance_m, 3.5) &&
            clear_boundary.envelope_point && clear_boundary.boundary_point &&
            near(clear_boundary.envelope_point->second, -0.5) &&
            near(clear_boundary.boundary_point->second, 3.0),
        "객체 반대쪽 차로 경계까지 가장 좁은 양의 거리를 고릅니다");
  input.road_shoulder_distance_m = 0.0;
  const auto mapped_geometry = planning::staticAvoidanceObjectGeometry(
      reference, 5.0, 0.0,
      {{20.0, -2.0}, {24.0, -2.0},
       {24.0, -1.0}, {20.0, -1.0}},
      reference[20], 0.5, true, input,
      planning::StaticAvoidanceBoundaryInput{
          {{0.0, 3.0}, {60.0, 3.0}}, 0.0});
  check(mapped_geometry && mapped_geometry->boundary_distance &&
            near(mapped_geometry->boundary_distance->distance_m, 3.5) &&
            near(mapped_geometry->lateral_margin_m, 1.9) &&
            near(mapped_geometry->desired_shift_m, 1.4),
        "지도 경계와 객체 외피를 연결해 회피 여유를 계산합니다");
  const auto intruding_boundary =
      planning::staticAvoidanceRoadShoulderDistance(
          object_geometry->overhang_points, 10.0,
          object_geometry->longitudinal.length_m,
          {{0.0, -1.5}, {60.0, -1.5}}, 0.0, true);
  check(near(intruding_boundary.distance_m, -1.0),
        "경계가 객체 외피 안으로 들어오면 음의 거리를 반환합니다");
  check(!planning::staticAvoidanceObjectGeometry(
            reference, 5.0, 0.0,
            {{20.0, -2.0}, {24.0, -2.0},
             {24.0, -1.0}, {20.0, -1.0}},
            reference[20], 0.5, true, input,
            planning::StaticAvoidanceBoundaryInput{
                {{0.0, -1.5}, {60.0, -1.5}}, 0.0}),
        "지도 경계가 객체 외피 안에 들어오면 회피 후보를 버립니다");
  const auto missing_boundary =
      planning::staticAvoidanceRoadShoulderDistance(
          object_geometry->overhang_points, 10.0,
          object_geometry->longitudinal.length_m,
          {{70.0, 3.0}, {80.0, 3.0}}, 0.0, true);
  check(near(missing_boundary.distance_m, 0.0) &&
            !missing_boundary.boundary_point,
        "대응하는 경계 교점이 없으면 원본의 기본 거리 0을 반환합니다");
  const std::vector<planning::StaticAvoidanceOverhangPoint> left_points{
      {1.0, 10.0, 1.0}, {2.0, 11.0, 2.0}};
  const auto left_clearance =
      planning::staticAvoidanceRoadShoulderDistance(
          left_points, 2.0, 2.0,
          {{0.0, -3.0}, {20.0, -3.0}}, 0.0, false);
  const auto left_intrusion =
      planning::staticAvoidanceRoadShoulderDistance(
          left_points, 2.0, 2.0,
          {{0.0, 1.5}, {20.0, 1.5}}, 0.0, false);
  check(near(left_clearance.distance_m, 4.0) &&
            near(left_intrusion.distance_m, -0.5),
        "왼쪽 객체는 오른쪽 경계 방향의 교점과 역방향 침범을 구분합니다");
  planning::BehaviorSafetyObject detected_shape;
  detected_shape.shape_type = 0;
  detected_shape.length_m = 4.0;
  detected_shape.width_m = 2.0;
  const auto world_polygon = planning::behaviorSafetyWorldPolygon(
      {0.0, 20.0, -1.5, 0.0, 0.0}, detected_shape);
  input.road_shoulder_distance_m = 5.0;
  const auto shape_geometry = planning::staticAvoidanceObjectGeometry(
      reference, 5.0, 0.0, world_polygon,
      reference[20], 0.0, true, input);
  check(world_polygon.size() == 5 && shape_geometry &&
            near(shape_geometry->longitudinal.start_from_ego_m, 13.0) &&
            near(shape_geometry->desired_shift_m, 1.4),
        "인식 직사각형 외형을 세계 좌표 외피와 회피량 계산에 연결합니다");
  input.parked = true;
  input.road_shoulder_distance_m = 2.5;
  check(!planning::staticAvoidanceObjectGeometry(
            reference, 5.0, 0.0,
            {{20.0, -2.0}, {24.0, -2.0},
             {24.0, -1.0}, {20.0, -1.0}},
            reference[20], 0.5, true, input),
        "차로 경계 여유가 모자라면 객체별 계산을 중단합니다");
  const auto right_overhang =
      planning::staticAvoidanceEnvelopeOverhangDistances(
          reference, {{10.0, -2.0}, {11.0, -2.0},
                      {11.0, -1.0}, {10.0, -1.0}}, true);
  const auto left_overhang =
      planning::staticAvoidanceEnvelopeOverhangDistances(
          reference, {{10.0, 1.0}, {11.0, 1.0},
                      {11.0, 2.0}, {10.0, 2.0}}, false);
  check(right_overhang.size() == 4 &&
            near(right_overhang.front().lateral_m, -1.0) &&
            left_overhang.size() == 4 &&
            near(left_overhang.front().lateral_m, 1.0),
        "객체 외피에서 경로에 가장 가까운 좌우 돌출점을 고릅니다");
  const auto shifted = planning::generateStaticAvoidancePath(
      reference, 0.0, *outline);
  check(shifted && shifted->size() == reference.size() &&
            near((*shifted)[10].y, 0.0) &&
            std::abs((*shifted)[30].y - 2.0) < 0.01 &&
            std::abs(shifted->back().y) < 0.01,
        "회피와 복귀 이동선을 직선 기준 경로에 이어 적용합니다");
  const auto merged_path = planning::generateStaticAvoidanceMergedPath(
      reference, 0.0,
      {{10.0, 20.0, 0.0, 2.0}, {30.0, 40.0, 2.0, 0.0}});
  check(merged_path && merged_path->size() == reference.size() &&
            std::abs((*merged_path)[25].y - 2.0) < 0.01 &&
            std::abs((*merged_path)[45].y) < 0.01,
        "여러 이동선을 한 기준 경로의 PathShifter 계산에 연결합니다");
  const auto multi_plan = planning::planStaticAvoidanceMultiObject(
      reference, 0, 0.0, 0.0,
      {{10.0, 20.0, 0.0, 2.0},
       {20.0, 30.0, 2.0, 2.0},
       {30.0, 40.0, 2.0, 0.0}});
  check(multi_plan && !multi_plan->shift_arcs.empty() &&
            multi_plan->path.size() == reference.size() &&
            std::abs(multi_plan->path[25].y - 2.0) < 0.01 &&
            std::abs(multi_plan->path[45].y) < 0.01,
        "다중 객체 이동선의 병합·추출·정리·경로 적용을 한 호출로 잇습니다");
  const auto merged = planning::mergeStaticAvoidanceShiftArcs(
      {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0}, 1, 0.0,
      {{2.0, 6.0, 0.0, 2.0},
       {3.0, 5.0, 0.0, 3.0},
       {4.0, 7.0, 0.0, -1.0}});
  check(near(merged.positive_m[4], 1.5) &&
            near(merged.negative_m[5], -1.0 / 3.0) &&
            near(merged.combined_m[5], 1.5 - 1.0 / 3.0) &&
            near(merged.combined_m[6], -2.0 / 3.0) &&
            near(merged.combined_m[8], 0.0),
        "여러 객체 이동선의 같은 방향 최대값과 반대 방향 최소값을 더합니다");
  const auto held = planning::mergeStaticAvoidanceShiftArcs(
      {0.0, 1.0, 2.0, 3.0, 3.5, 4.5, 6.0}, 0, 0.0,
      {{2.0, 4.0, 0.0, 1.0}});
  check(near(held.combined_m[4], 0.75) &&
            near(held.combined_m[5], 0.75) &&
            near(held.combined_m[6], 0.75),
        "활성 이동선이 없는 점에는 마지막 이동량을 유지합니다");
  const auto extracted = planning::extractStaticAvoidanceShiftArcs(
      {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0}, 0,
      {0.0, 0.0, 1.0, 2.0, 2.0, 1.0, 0.0, 0.0});
  check(extracted.size() == 3 &&
            near(extracted[0].start_m, 1.0) &&
            near(extracted[0].end_m, 3.0) &&
            near(extracted[0].end_shift_m, 2.0) &&
            near(extracted[1].start_m, 3.0) &&
            near(extracted[1].end_m, 4.0) &&
            near(extracted[2].start_m, 4.0) &&
            near(extracted[2].end_m, 6.0) &&
            near(extracted[2].end_shift_m, 0.0),
        "점별 횡이동량의 기울기 변화점에서 이동선을 다시 만듭니다");
  const auto trimmed = planning::trimStaticAvoidanceShiftArcs(
      {{1.0, 3.0, 0.0, 1.0}, {3.0, 5.0, 1.0, 2.02},
       {5.0, 6.0, 2.02, 2.02}, {6.0, 8.0, 2.02, 0.0},
       {8.0, 8.5, 0.0, 0.1}});
  check(trimmed.size() == 3 &&
            near(trimmed[0].start_m, 1.0) &&
            near(trimmed[0].end_m, 5.0) &&
            near(trimmed[0].end_shift_m, 2.0) &&
            near(trimmed[1].end_shift_m, 2.0) &&
            near(trimmed[2].end_m, 8.0),
        "짧은 이동선 제거와 유사 기울기 병합·양자화를 원본 순서로 적용합니다");
  planning::StaticAvoidanceTrimParameters prepare_limit;
  prepare_limit.minimum_prepare_distance_m = 2.0;
  const auto prepared = planning::trimStaticAvoidanceShiftArcs(
      {{1.0, 3.0, 0.0, 1.0}, {3.0, 5.0, 1.0, 0.0}},
      prepare_limit);
  check(prepared.size() == 1 && near(prepared[0].start_m, 3.0),
        "준비 거리가 현재 속도·설정으로 정한 하한 이하인 구간은 제거합니다");
  const auto registered = planning::mergeStaticAvoidanceRegisteredShiftArcs(
      {{0.0, 3.0, 0.0, 0.5}, {1.0, 10.0, 0.0, 1.0},
       {2.0, 9.0, 0.0, 3.0}, {5.0, 7.0, 0.0, 0.5}},
      {{4.0, 8.0, 0.0, 2.0}});
  check(registered.size() == 3 &&
            near(registered[0].start_m, 0.0) &&
            near(registered[1].start_m, 2.0) &&
            near(registered[2].start_m, 4.0),
        "새 이동선보다 뒤에서 시작하거나 약하게 겹치는 이전 이동선을 교체합니다");
  const auto negative_registered =
      planning::mergeStaticAvoidanceRegisteredShiftArcs(
          {{1.0, 10.0, 0.0, -1.0}, {2.0, 9.0, 0.0, -3.0}},
          {{4.0, 8.0, 0.0, -2.0}});
  check(negative_registered.size() == 2 &&
            near(negative_registered[0].end_shift_m, -3.0) &&
            near(negative_registered[1].end_shift_m, -2.0),
        "왼쪽 회피에서도 더 약하게 겹치는 이전 이동선만 교체합니다");
  const std::vector<planning::StaticAvoidanceAnchoredShiftArc> anchored{
      {4.2, 0.1, 8.1, -0.1, 0.0, 2.0},
      {0.1, 0.0, 3.2, 0.0, 0.0, 1.0},
      {6.1, 0.0, 9.1, 0.0, 0.0, 1.0}};
  const auto reprojected =
      planning::reprojectStaticAvoidanceRegisteredShiftArcs(
          reference, 5, 0.5, anchored);
  check(reprojected.size() == 2 &&
            near(reprojected[0].start_m, -1.0) &&
            near(reprojected[0].end_m, 3.0) &&
            near(reprojected[1].start_m, 1.0) &&
            near(reprojected[1].end_m, 4.0),
        "등록 이동선의 세계 좌표 기준점을 현재 경로점에 투영하고 자차 뒤 종료선은 제외합니다");
  const auto inconsistent =
      planning::reprojectStaticAvoidanceRegisteredShiftArcs(
          reference, 5, 0.7, anchored);
  check(inconsistent.size() == 1 &&
            near(inconsistent.front().start_m, 1.0),
        "자차 위치에서 이전 이동선의 기대 횡이동과 0.1m 넘게 다르면 제외합니다");
  const std::vector<planning::StaticAvoidanceShiftArc> proposal{
      {10.0, 20.0, 0.0, 2.0}};
  const auto unapproved = planning::planStaticAvoidanceWithRegisteredShifts(
      reference, 5, 0.0, {}, proposal, false);
  check(unapproved && unapproved->active_shift_arcs.empty() &&
            unapproved->path.size() == reference.size(),
        "승인 전 새 이동선은 현재 경로와 등록 상태에 반영하지 않습니다");
  const auto approved = planning::planStaticAvoidanceWithRegisteredShifts(
      reference, 5, 0.0, {}, proposal, true);
  check(approved && approved->active_shift_arcs.size() == 1 &&
            approved->registered_shift_arcs.size() == 1 &&
            near(approved->registered_shift_arcs.front().start_x, 15.0) &&
            near(approved->registered_shift_arcs.front().end_x, 25.0),
        "승인된 이동선의 세계 좌표 기준점을 다음 콜백용으로 등록합니다");
  const auto next_frame = planning::planStaticAvoidanceWithRegisteredShifts(
      reference, 6, 0.0, approved->registered_shift_arcs, {}, false);
  check(next_frame && next_frame->active_shift_arcs.size() == 1 &&
            near(next_frame->active_shift_arcs.front().start_m, 9.0) &&
            near(next_frame->active_shift_arcs.front().end_m, 19.0),
        "다음 콜백에서는 등록된 기준점을 새 자차 위치의 호길이에 투영합니다");
  planning::StaticAvoidanceSingleObjectInput single;
  single.ego_x = 5.0;
  single.object_polygon = {
      {30.0, -2.0}, {34.0, -2.0},
      {34.0, -1.0}, {30.0, -1.0}};
  single.closest_reference_pose = reference[30];
  single.object_on_right = true;
  single.margin = input;
  single.margin.parked = false;
  single.margin.road_shoulder_distance_m = 0.0;
  single.boundary = planning::StaticAvoidanceBoundaryInput{
      {{0.0, 3.0}, {60.0, 3.0}}, 0.0};
  single.feasibility.front_constant_distance_m = 2.0;
  single.feasibility.prepare_distance_m = 2.0;
  single.feasibility.nominal_avoidance_distance_m = 6.0;
  single.feasibility.avoidance_speed_mps = 5.0;
  single.feasibility.maximum_lateral_jerk_mps3 = 0.5;
  single.feasibility.minimum_distance_jerk_mps3 = 1.0;
  single.feasibility.minimum_distance_speed_mps = 1.0;
  single.outline.front_constant_distance_m = 2.0;
  single.outline.rear_constant_distance_m = 2.0;
  single.outline.maximum_return_distance_m = 10.0;
  single.outline.to_start_point_m = 10.0;
  single.outline.to_return_point_m = 40.0;
  const auto single_plan = planning::planStaticAvoidanceSingleObject(
      reference, single);
  check(single_plan &&
            near(single_plan->geometry.longitudinal.start_from_ego_m, 25.0) &&
            near(single_plan->feasibility.shift_m, 0.9) &&
            near(single_plan->outline.avoid.start_m, 17.0) &&
            std::abs(single_plan->path[30].y - 0.9) < 0.01 &&
            std::abs(single_plan->path[50].y) < 0.01,
        "한 객체의 지도 경계부터 회피·복귀 경로까지 연결합니다");
  single.boundary = planning::StaticAvoidanceBoundaryInput{
      {{0.0, -1.5}, {60.0, -1.5}}, 0.0};
  check(!planning::planStaticAvoidanceSingleObject(reference, single),
        "경계가 객체 안에 있으면 독립 경로를 만들지 않습니다");
  check(!planning::generateStaticAvoidancePath(
            planning::Trajectory(reference.begin(),
                                 reference.begin() + 30),
            0.0, *outline),
        "복귀 구간이 경로 밖에 있으면 후보를 버립니다");
  outline_input.skip_return = true;
  const auto no_return = planning::staticAvoidanceShiftOutline(outline_input);
  check(no_return && !no_return->return_to_center,
        "목표 가까이에서는 복귀 이동선을 생략합니다");
  outline_input.avoidance_distance_m = 3.0;
  check(!planning::staticAvoidanceShiftOutline(outline_input),
        "최소 회피 거리보다 짧은 이동선은 만들지 않습니다");
  outline_input.to_start_point_m = 30.0;
  const auto emergency_start =
      planning::staticAvoidanceShiftOutline(outline_input);
  check(emergency_start && near(emergency_start->avoid.start_m, 22.0),
        "희망 시작점이 객체 앞 종료점보다 멀면 최근 회피 시작점을 사용합니다");
  return EXIT_SUCCESS;
}
