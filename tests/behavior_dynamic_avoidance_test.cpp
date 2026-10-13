#include "planning/behavior_dynamic_avoidance.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
void check(bool value, const char *message) {
  if (!value) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}
}  // 익명 네임스페이스

int main() {
  planning::Trajectory direction_path(2);
  direction_path[1].x = 1.0;
  check(planning::behaviorDynamicPathForward(direction_path),
        "전진 첫 경로 구간에서 동적 회피를 허용합니다");
  direction_path[1].x = -1.0;
  check(!planning::behaviorDynamicPathForward(direction_path),
        "후진 첫 경로 구간에서는 동적 회피를 실행하지 않습니다");
  planning::BehaviorDynamicPredictedPolygonInput predicted;
  predicted.object_length_m = 4.0;
  predicted.object_width_m = 2.0;
  predicted.paths.push_back({1.0, 1.0,
                             {{0.0, 10.0, 0.0, 0.0, 0.0},
                              {1.0, 20.0, 0.0, 0.0, 0.0}}});
  const auto predicted_polygon =
      planning::behaviorDynamicPredictedPolygon(predicted);
  check(predicted_polygon && predicted_polygon->size() > 4,
        "예측 객체 외곽의 볼록 껍질과 보행자 여유를 생성합니다");
  const auto object_path_polygon =
      planning::behaviorDynamicObjectPathPolygon(
          predicted.paths.front().poses, 4.0, 2.0);
  check(object_path_polygon && object_path_polygon->size() == 5 &&
            std::abs(object_path_polygon->front().first - 5.0) < 1.0e-8 &&
            std::abs(object_path_polygon->front().second + 2.0) < 1.0e-8 &&
            std::abs(object_path_polygon->at(1).first - 23.0) < 1.0e-8,
        "규제 객체의 예측 경로를 전후·측방 여유 다각형으로 둘러쌉니다");
  predicted.ego_reserve_polygon =
      {{0.0, -1.0}, {30.0, -1.0},
       {30.0, 1.0}, {0.0, 1.0}};
  check(!planning::behaviorDynamicPredictedPolygon(predicted),
        "자차 통로를 빼면 두 부분으로 갈라지는 예측 객체는 제외합니다");
  const auto timing = planning::behaviorDynamicCollisionTiming(
      {10.0, 5.0, 25.0, -2.0, 2.0, 4.0, 1.0});
  check(std::abs(timing.start_s - 3.8) < 1.0e-9 &&
            std::abs(timing.end_s - 5.6) < 1.0e-9,
        "같은 방향 객체의 상대 속도로 충돌 시작·종료 시각을 계산합니다");
  planning::BehaviorDynamicLongitudinalBoundsInput longitudinal;
  longitudinal.object_min_longitudinal_m = -2.0;
  longitudinal.object_max_longitudinal_m = 2.0;
  longitudinal.object_path_speed_mps = 5.0;
  longitudinal.ego_speed_mps = 10.0;
  longitudinal.collision_time = timing;
  longitudinal.valid_predicted_path_length_m = 100.0;
  const auto longitudinal_bounds =
      planning::behaviorDynamicLongitudinalBounds(longitudinal);
  check(longitudinal_bounds &&
            std::abs(longitudinal_bounds->first + 12.25) < 1.0e-9 &&
            std::abs(longitudinal_bounds->second - 22.0) < 1.0e-9,
        "같은 방향 객체의 감속·상대 속도로 전후 제외 길이를 계산합니다");
  planning::BehaviorDynamicUnregulatedLateralInput lateral;
  lateral.object_min_lateral_m = 2.0;
  lateral.object_max_lateral_m = 3.0;
  const auto left_bounds =
      planning::behaviorDynamicUnregulatedLateralBounds(lateral);
  check(left_bounds &&
            std::abs(left_bounds->near_m - 1.0) < 1.0e-9 &&
            std::abs(left_bounds->far_m - 4.0) < 1.0e-9,
        "왼쪽 비규제 객체의 가까운·먼 회피 경계를 계산합니다");
  lateral.object_lateral_speed_mps = -1.0;
  check(!planning::behaviorDynamicUnregulatedLateralBounds(lateral),
        "미래 점유가 자차 경로를 가로지르면 횡회피 경계를 만들지 않습니다");
  planning::BehaviorDynamicRegulatedLateralInput regulated;
  regulated.object_min_lateral_m = 1.0;
  regulated.object_max_lateral_m = 2.0;
  regulated.object_path_speed_mps = 3.0;
  regulated.collision_on_left = true;
  const auto regulated_bounds =
      planning::behaviorDynamicRegulatedLateralBounds(regulated);
  check(regulated_bounds &&
            std::abs(regulated_bounds->near_m - 0.448) < 1.0e-9 &&
            std::abs(regulated_bounds->far_m - 3.0) < 1.0e-9,
        "왼쪽 차량의 안쪽 경계를 최소 자차 통로 폭으로 제한합니다");
  regulated.object_min_lateral_m = -0.5;
  regulated.object_max_lateral_m = 0.5;
  check(!planning::behaviorDynamicRegulatedLateralBounds(
            regulated),
        "자차 통로를 넓게 덮는 선행 차량은 횡회피 대상에서 제외합니다");
  planning::BehaviorDynamicCutInInput cut_in;
  cut_in.predicted_path_enters_ego_corridor = true;
  cut_in.object_path_speed_mps = 2.0;
  cut_in.ego_speed_mps = 10.0;
  cut_in.ego_to_object_back_m = 12.0;
  check(planning::behaviorDynamicObjectWillCutIn(cut_in),
        "앞 객체의 미래 경로가 자차 통로에 들어오면 진입 대상으로 판단합니다");
  cut_in.ego_to_object_back_m = 4.0;
  check(!planning::behaviorDynamicObjectWillCutIn(cut_in),
        "너무 가까운 앞 객체는 진입 예측 분기에서 제외합니다");
  planning::BehaviorDynamicCutOutInput cut_out;
  cut_out.object_on_left = true;
  cut_out.object_path_speed_mps = 2.0;
  cut_out.object_lateral_speed_mps = 0.5;
  cut_out.time_since_inside_ego_path_s = 1.0;
  check(planning::behaviorDynamicObjectWillCutOut(cut_out),
        "최근 왼쪽으로 통로를 벗어난 객체를 이탈 대상으로 판단합니다");
  cut_out.time_since_inside_ego_path_s = 3.0;
  check(!planning::behaviorDynamicObjectWillCutOut(cut_out),
        "통로에서 벗어난 지 오래된 객체는 이탈 분기에서 제외합니다");
  planning::BehaviorDynamicAvoidanceCorridorInput corridor;
  for (int x = 0; x <= 100; x += 10) {
    planning::TrajectoryPoint point;
    point.x = x;
    corridor.reference.push_back(point);
  }
  const auto reserve = planning::behaviorDynamicEgoReservePolygon(
      corridor.reference, true);
  double minimum_y = 100.0;
  double maximum_y = -100.0;
  if (reserve) {
    for (const auto &point : *reserve) {
      minimum_y = std::min(minimum_y, point.second);
      maximum_y = std::max(maximum_y, point.second);
    }
  }
  check(reserve && std::abs(minimum_y + 0.448) < 1.0e-6 &&
            std::abs(maximum_y - 0.948) < 1.0e-6,
        "왼쪽 객체의 자차 통로를 좌우 비대칭 폭으로 보존합니다");
  corridor.object_x = 50.0;
  const auto offsets = planning::behaviorDynamicPathOffsets(
      corridor.reference, 50.0, 0.0,
      {{48.0, -1.0}, {52.0, -1.0},
       {52.0, 1.0}, {48.0, 1.0}});
  check(offsets &&
            std::abs(offsets->object_arc_m - 50.0) < 1.0e-9 &&
            std::abs(offsets->minimum_lateral_m + 1.0) < 1.0e-9 &&
            std::abs(offsets->maximum_lateral_m - 1.0) < 1.0e-9 &&
            std::abs(offsets->minimum_longitudinal_m + 2.0) <
                1.0e-9 &&
            std::abs(offsets->maximum_longitudinal_m - 2.0) <
                1.0e-9,
        "객체 외곽의 경로 기준 전후·좌우 거리를 투영합니다");
  planning::BehaviorDynamicRegulatedPlanInput integrated;
  integrated.reference = corridor.reference;
  integrated.ego_x = 5.0;
  integrated.ego_speed_mps = 10.0;
  integrated.object_x = 50.0;
  integrated.object_y = 2.5;
  integrated.object_path_speed_mps = 5.0;
  integrated.object_polygon =
      {{48.0, 2.0}, {52.0, 2.0},
       {52.0, 3.0}, {48.0, 3.0}};
  integrated.collision_on_left = true;
  const auto integrated_plan =
      planning::planBehaviorDynamicRegulatedObject(
          integrated);
  check(integrated_plan &&
            integrated_plan->exclusion_polygon.size() >= 5 &&
            integrated_plan->collision_time.start_s > 0.0 &&
            integrated_plan->longitudinal_bounds.first < 0.0 &&
            integrated_plan->longitudinal_bounds.second > 0.0,
        "차량 외곽부터 충돌 시각·회피 경계·제외 다각형을 이어 계산합니다");
  auto filtered_plan_input = integrated;
  filtered_plan_input.previous_near_m = 0.448;
  const auto filtered_plan =
      planning::planBehaviorDynamicRegulatedObject(
          filtered_plan_input);
  check(filtered_plan &&
            std::abs(filtered_plan->lateral_bounds.near_m -
                     0.5032) < 1.0e-9,
        "이전 콜백의 안쪽 회피 경계를 저역 통과합니다");
  filtered_plan_input.use_lateral_lowpass = false;
  const auto changed_reference_plan =
      planning::planBehaviorDynamicRegulatedObject(
          filtered_plan_input);
  check(changed_reference_plan &&
            std::abs(changed_reference_plan->lateral_bounds.near_m -
                     1.0) < 1.0e-9,
        "기준 경로가 측방 이동하면 이전 경계의 저역 통과를 끕니다");
  auto near_plan = integrated;
  near_plan.ego_x = 0.0;
  near_plan.ego_speed_mps = 5.0;
  near_plan.object_x = 10.0;
  near_plan.object_y = 1.5;
  near_plan.object_path_speed_mps = 0.0;
  near_plan.object_polygon =
      {{8.0, 1.0}, {12.0, 1.0},
       {12.0, 2.0}, {8.0, 2.0}};
  const auto feasible_plan =
      planning::planBehaviorDynamicRegulatedObject(near_plan);
  double feasible_start = std::numeric_limits<double>::infinity();
  if (feasible_plan)
    for (const auto &[x, y] : feasible_plan->exclusion_polygon) {
      feasible_start = std::min(feasible_start, x);
      (void)y;
    }
  check(feasible_plan &&
            std::abs(feasible_start - 11.875) < 1.0e-6,
        "자차 횡가속도·저크로 가능한 지점부터 회피 외곽을 시작합니다");
  near_plan.feasibility_origin =
      planning::BehaviorSafetyPose{0.0, 0.0, 0.0, 0.0, 5.0};
  near_plan.ego_x = 8.0;
  const auto retained_feasibility =
      planning::planBehaviorDynamicRegulatedObject(near_plan);
  double retained_start = std::numeric_limits<double>::infinity();
  if (retained_feasibility)
    for (const auto &[x, y] :
         retained_feasibility->exclusion_polygon) {
      retained_start = std::min(retained_start, x);
      (void)y;
    }
  check(retained_feasibility &&
            std::abs(retained_start - feasible_start) < 1.0e-6,
        "연속 회피 대상은 진입 시점의 횡이동 가능 구간을 유지합니다");
  near_plan.ego_x = 0.0;
  near_plan.feasibility_origin.reset();
  near_plan.object_x = 5.0;
  near_plan.object_polygon =
      {{3.0, 1.0}, {7.0, 1.0},
       {7.0, 2.0}, {3.0, 2.0}};
  check(!planning::planBehaviorDynamicRegulatedObject(near_plan),
        "측방 회피가 가능해지기 전에 끝나는 객체는 제외합니다");
  corridor.longitudinal_start_m = -5.0;
  corridor.longitudinal_end_m = 10.0;
  corridor.inner_lateral_m = 1.0;
  corridor.outer_lateral_m = 3.0;
  const auto polygon = planning::behaviorDynamicAvoidanceCorridor(
      corridor);
  check(polygon && polygon->size() == 7 &&
            std::abs(polygon->front().first - 45.0) < 1.0e-8 &&
            std::abs(polygon->front().second - 1.0) < 1.0e-8 &&
            std::abs(polygon->at(3).first - 60.0) < 1.0e-8 &&
            std::abs(polygon->at(3).second - 3.0) < 1.0e-8,
        "경로형 동적 장애물 제외 다각형의 전후·좌우 경계를 계산합니다");
  corridor.longitudinal_start_m = 100.0;
  corridor.longitudinal_end_m = 120.0;
  check(!planning::behaviorDynamicAvoidanceCorridor(corridor),
        "경로 밖 객체 구간에는 제외 다각형을 만들지 않습니다");
  planning::BehaviorDynamicAvoidanceParameters parameters;
  planning::BehaviorDynamicRoughAdmissionInput rough;
  rough.classification = 1;
  rough.tangent_speed_mps = 3.0;
  rough.speed_norm_mps = 3.0;
  rough.center_distance_to_path_m = 1.5;
  rough.object_maximum_radius_m = 2.0;
  check(planning::behaviorDynamicRoughAdmission(
            rough, parameters),
        "경로 가까이 나란히 움직이는 차량은 1차 선별을 통과합니다");
  rough.yaw_difference_rad = 1.57;
  check(!planning::behaviorDynamicRoughAdmission(
            rough, parameters),
        "빠르게 경로를 가로지르는 차량은 1차 선별에서 제외합니다");
  rough.yaw_difference_rad = 0.0;
  rough.center_distance_to_path_m = 10.0;
  check(!planning::behaviorDynamicRoughAdmission(
            rough, parameters),
        "외형 반경을 뺀 경로 거리도 먼 차량은 제외합니다");
  rough.classification = 7;
  rough.center_distance_to_path_m = 1.0;
  rough.normal_speed_mps = 1.0;
  check(!planning::behaviorDynamicRoughAdmission(
            rough, parameters),
        "측방으로 빠르게 이동하는 보행자는 제외합니다");
  check(planning::behaviorDynamicObjectType(1, parameters) ==
            planning::BehaviorDynamicObjectType::Regulated &&
            planning::behaviorDynamicObjectType(7, parameters) ==
                planning::BehaviorDynamicObjectType::Unregulated,
        "실행 설정은 차량을 규제 대상으로, 보행자를 비규제 대상으로 분류합니다");
  parameters.avoid_types[7] = false;
  check(planning::behaviorDynamicObjectType(7, parameters) ==
            planning::BehaviorDynamicObjectType::OutOfScope,
        "제외된 보행자는 회피 대상에서 빼도록 분류합니다");
  planning::BehaviorDynamicAvoidanceTracker tracker(parameters);
  const planning::BehaviorDynamicAvoidanceObservation car{
      "차량", 1, 3.0, 10.0, 0.0};
  for (int callback = 0; callback < 4; ++callback)
    check(tracker.advance({car}).empty(),
          "연속 관측 다섯 번 전에는 대상을 확정하지 않습니다");
  const auto entered = tracker.advance({car});
  check(entered.size() == 1 && entered.front().id == "차량",
        "다섯 번째 연속 관측에서 동적 회피 대상을 확정합니다");
  for (int callback = 0; callback < 4; ++callback)
    check(tracker.advance({}).size() == 1,
          "일시 누락된 대상의 마지막 관측을 보존합니다");
  check(tracker.advance({}).empty(),
        "이탈 카운터 아래로 떨어진 객체를 제거합니다");
  check(tracker.advance({car}).empty(),
        "제거된 객체는 새 관측 연속 횟수부터 다시 셉니다");
  tracker.reset();
  check(tracker.advance({car}).empty(),
        "초기화 뒤에는 이전 연속 관측을 재사용하지 않습니다");
}
