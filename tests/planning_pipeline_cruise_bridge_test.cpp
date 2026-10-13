// SPDX-License-Identifier: Apache-2.0
// 예측 객체를 종단 경로 정지 후보로 바꾸는 실제 충돌 분기를 검사합니다.

#include "planning/planning_pipeline_cruise_bridge.hpp"
#include "planning/obstacle_cruise_collision.hpp"
#include "planning/obstacle_cruise_slowdown_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {
void check(bool valid, const char *message) {
  if (!valid) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}
}  // 익명 네임스페이스

int main() {
  planning::Trajectory path;
  for (int index = 0; index <= 100; ++index) {
    planning::TrajectoryPoint point;
    point.x = index;
    point.longitudinal_velocity_mps = 5.0F;
    path.push_back(point);
  }
  planning::TrajectoryPoint ego;
  planning::PlanningCruisePerceptionFrame perception;
  perception.stamp_s = 10.0;
  planning::PlanningPerceivedObject object;
  object.id = "정지객체";
  object.classification = 1;
  object.x = 30.0;
  object.length_m = 4.0;
  object.width_m = 2.0;
  perception.objects.push_back(object);
  planning::PlanningCruiseStopCandidateCore core;
  const auto candidates = core.plan(path, ego, 5.0, 10.0, perception);
  check(candidates.size() == 1 &&
            candidates.front().collision_distance_from_start > 0.0 &&
            candidates.front().desired_margin > 0.0 &&
            core.claimedObjectIds() ==
                std::vector<std::string>{"정지객체"},
        "전방의 정지 객체는 충돌 위치와 정지 여유를 생성합니다");
  planning::CruiseCandidateParameters configured_margin;
  configured_margin.stop_safe_distance_margin_m = 6.0;
  planning::PlanningCruiseStopCandidateCore margin_core(configured_margin);
  const auto margin_candidates = margin_core.plan(
      path, ego, 5.0, 10.0, perception);
  check(margin_candidates.size() == 1 &&
            std::abs(margin_candidates.front().desired_margin - 6.0) < 1.0e-6,
        "원본 장애물 정지 안전거리 설정을 후보 여유에 전달합니다");
  planning::PlanningCruiseStopCandidateCore obigo(
      {}, {3.74, 1.03, 1.83});
  const auto obigo_candidates = obigo.plan(
      path, ego, 5.0, 10.0, perception);
  check(obigo_candidates.size() == 1 &&
            std::abs(obigo_candidates.front().collision_distance_from_start -
                     candidates.front().collision_distance_from_start -
                     0.05) < 1.0e-6,
        "기본 sample 차량 앞 길이와 지정한 Obigo 차량 차이가 반영됩니다");
  perception.objects.front().y = 20.0;
  perception.objects.front().velocity_x_mps = 1.4;
  perception.stamp_s = 10.5;
  check(core.plan(path, ego, 5.0, 11.4, perception).size() == 1,
        "이전 정지 객체 유지는 인식 헤더 시각과 1.5m/s 임계값으로 검사합니다");
  perception.stamp_s = 11.1;
  check(core.plan(path, ego, 5.0, 11.9, perception).empty(),
        "인식 헤더가 이전 객체 시각보다 1초 이상 지나면 유지를 끝냅니다");
  perception.objects.front().y = 20.0;
  core.reset();
  check(core.plan(path, ego, 5.0, 10.1, perception).empty() &&
            core.claimedObjectIds().empty(),
        "횡방향으로 먼 객체는 정지 후보가 아닙니다");
  perception.objects.front().y = 0.0;
  perception.objects.front().velocity_x_mps = 5.0;
  planning::CruisePredictedPath prediction;
  prediction.time_step_s = 1.0;
  prediction.confidence = 1.0;
  for (int index = 0; index < 8; ++index)
    prediction.poses.push_back({30.0 + 5.0 * index, 0.0, 0.0});
  perception.objects.front().predicted_paths = {prediction};
  core.reset();
  check(core.plan(path, ego, 5.0, 10.0, perception).empty() &&
            core.claimedObjectIds() ==
                std::vector<std::string>{"정지객체"},
        "같은 방향으로 이동하는 순항 객체는 정지 후보에서 제외합니다");
  check(core.pidObstacles().size() == 1 &&
            core.pidObstacles().front().distance_from_ego_front > 0.0 &&
            core.pidObstacles().front().distance_from_ego_front < 30.0 &&
            std::abs(core.pidObstacles().front().velocity - 5.0) < 1.0e-6,
        "순항 충돌점과 경로 방향 객체 속도를 PID 입력으로 전달합니다");
  auto reverse_path = path;
  for (auto &point : reverse_path) {
    point.orientation_z = 1.0;
    point.orientation_w = 0.0;
  }
  auto reverse_perception = perception;
  reverse_perception.objects.front().yaw_rad = std::acos(-1.0);
  planning::PlanningCruiseStopCandidateCore reverse_core;
  reverse_core.plan(reverse_path, ego, 5.0, 10.0, reverse_perception);
  check(reverse_core.pidObstacles().empty(),
        "원본처럼 후진 궤적에서는 장애물 순항 후보를 만들지 않습니다");
  auto reverse_stop_path = path;
  for (std::size_t index = 0; index < reverse_stop_path.size(); ++index)
    reverse_stop_path[index].x = 100.0 - index;
  auto reverse_ego = ego;
  reverse_ego.x = 100.0;
  auto reverse_stop_object = object;
  reverse_stop_object.x = 70.0;
  planning::PlanningCruisePerceptionFrame reverse_stop_perception;
  reverse_stop_perception.stamp_s = 10.0;
  reverse_stop_perception.objects.push_back(reverse_stop_object);
  planning::PlanningCruiseStopCandidateCore reverse_stop_core;
  check(reverse_stop_core.plan(reverse_stop_path, reverse_ego, -5.0, 10.0,
                               reverse_stop_perception).size() == 1 &&
            reverse_stop_core.pidObstacles().empty(),
        "후진 궤적은 뒤 범퍼 기준으로 정지 후보를 계산하고 순항하지 않습니다");
  const auto cruise_object = perception.objects.front();
  perception.objects.clear();
  core.plan(path, ego, 5.0, 10.1, perception);
  check(core.pidObstacles().empty(),
        "순항 대상이 사라지면 이전 PID 입력을 유지하지 않습니다");
  perception.objects.push_back(cruise_object);
  perception.objects.front().velocity_x_mps = 2.0;
  for (int index = 0; index < 8; ++index)
    perception.objects.front().predicted_paths.front().poses[index].x =
        30.0 + 2.0 * index;
  core.reset();
  const auto launch_classification = core.plan(
      path, ego, 5.0, 10.0, perception);
  planning::CruiseCandidateParameters package_parameters;
  package_parameters.inside_cruise_entry_speed_mps = 3.5;
  package_parameters.inside_cruise_keep_speed_mps = 3.0;
  planning::PlanningCruiseStopCandidateCore package_core(package_parameters);
  const auto package_classification = package_core.plan(
      path, ego, 5.0, 10.0, perception);
  check(launch_classification.empty() && package_classification.size() == 1,
        "원본 설정별 내부 객체 순항/정지 전환 속도를 전달합니다");
  planning::PlanningCruisePerceptionFrame held_object;
  held_object.stamp_s = 10.0;
  held_object.objects.push_back(object);
  planning::PlanningCruiseStopCandidateCore launch_hold;
  planning::PlanningCruiseStopCandidateCore package_hold(package_parameters);
  check(launch_hold.plan(path, ego, 5.0, 10.0, held_object).size() == 1 &&
            package_hold.plan(path, ego, 5.0, 10.0, held_object).size() == 1,
        "정지 객체를 이전 주기 상태로 저장합니다");
  held_object.stamp_s = 10.5;
  held_object.objects.front().y = 20.0;
  held_object.objects.front().velocity_x_mps = 2.0;
  check(launch_hold.plan(path, ego, 5.0, 10.5, held_object).empty() &&
            package_hold.plan(path, ego, 5.0, 10.5, held_object).size() == 1,
        "원본 stop-to-cruise 설정을 이전 정지 객체 유지에도 전달합니다");
  perception.objects.front().y = 3.0;
  perception.objects.front().velocity_x_mps = 5.0;
  perception.objects.front().velocity_y_mps = -0.5;
  auto lane_keep = prediction;
  lane_keep.confidence = 0.8;
  for (auto &pose : lane_keep.poses) pose.y = 3.0;
  auto lane_change = prediction;
  lane_change.confidence = 0.2;
  for (std::size_t index = 0; index < lane_change.poses.size(); ++index)
    lane_change.poses[index].y = std::max(0.0, 3.0 - 1.5 * index);
  perception.objects.front().predicted_paths = {lane_keep, lane_change};
  planning::CruiseCandidateParameters one_path_parameters;
  one_path_parameters.outside_predicted_path_count = 1;
  planning::PlanningCruiseStopCandidateCore one_path_core(one_path_parameters);
  planning::CruiseCandidateParameters two_path_parameters;
  two_path_parameters.outside_predicted_path_count = 2;
  planning::PlanningCruiseStopCandidateCore two_path_core(two_path_parameters);
  one_path_core.plan(path, ego, 5.0, 10.0, perception);
  two_path_core.plan(path, ego, 5.0, 10.0, perception);
  check(one_path_core.pidObstacles().empty() &&
            two_path_core.pidObstacles().size() == 1,
        "원본의 차로 밖 예측 경로 검사 개수 설정을 반영합니다");
  planning::PlanningCruisePerceptionFrame moving_away;
  moving_away.stamp_s = 10.0;
  auto transient = object;
  transient.id = "미래이탈객체";
  transient.predicted_paths = {prediction};
  for (std::size_t index = 0;
       index < transient.predicted_paths.front().poses.size(); ++index) {
    transient.predicted_paths.front().poses[index] =
        {30.0, static_cast<double>(index) * 2.0, 0.0};
  }
  moving_away.objects.push_back(transient);
  planning::CruiseCandidateParameters one_second;
  one_second.stop_collision_time_margin_s = 1.0;
  planning::CruiseCandidateParameters four_seconds;
  four_seconds.stop_collision_time_margin_s = 4.0;
  planning::PlanningCruiseStopCandidateCore short_margin(one_second);
  planning::PlanningCruiseStopCandidateCore long_margin(four_seconds);
  check(short_margin.plan(path, ego, 5.0, 10.0, moving_away).empty() &&
            long_margin.plan(path, ego, 5.0, 10.0, moving_away).size() == 1,
        "원본 내부 정지 객체의 미래 충돌 확인 여유를 설정값으로 전달합니다");

  planning::PlanningCruisePerceptionFrame outside;
  outside.stamp_s = 10.0;
  auto crossing_pedestrian = object;
  crossing_pedestrian.id = "차로밖보행자";
  crossing_pedestrian.classification = 7;
  crossing_pedestrian.x = 15.0;
  crossing_pedestrian.y = 3.0;
  crossing_pedestrian.length_m = 0.6;
  crossing_pedestrian.width_m = 0.6;
  crossing_pedestrian.velocity_x_mps = 0.0;
  crossing_pedestrian.velocity_y_mps = -2.0;
  planning::CruisePredictedPath crossing_path;
  crossing_path.confidence = 1.0;
  crossing_path.time_step_s = 1.0;
  crossing_path.poses = {{15.0, 3.0, 0.0}, {15.0, 1.0, 0.0},
                         {15.0, -1.0, 0.0}};
  crossing_pedestrian.predicted_paths = {crossing_path};
  outside.objects.push_back(crossing_pedestrian);
  planning::PlanningCruiseStopCandidateCore launch_outside;
  check(launch_outside.plan(path, ego, 5.0, 10.0, outside).empty(),
        "Obigo launch에서 꺼진 차로 밖 정지 종류는 제외합니다");
  planning::CruiseCandidateParameters enabled_outside;
  enabled_outside.outside_stop_classes[7] = true;
  planning::PlanningCruiseStopCandidateCore outside_core(enabled_outside);
  const auto outside_candidates =
      outside_core.plan(path, ego, 5.0, 10.0, outside);
  check(outside_candidates.size() == 1 &&
            std::abs(outside_candidates.front().collision_distance_from_start -
                     10.91) < 0.01,
        "원본 차로 밖 보행자의 미래 충돌과 충돌 시각을 정지 후보에 연결합니다");
  auto vanished_outside = outside;
  vanished_outside.objects.front().y = 20.0;
  vanished_outside.objects.front().velocity_y_mps = 0.0;
  for (auto &pose : vanished_outside.objects.front().predicted_paths.front().poses)
    pose.y = 20.0;
  vanished_outside.stamp_s = 10.5;
  const auto held_outside = outside_core.plan(
      path, ego, 5.0, 10.5, vanished_outside);
  check(held_outside.size() == 1 &&
            std::abs(held_outside.front().collision_distance_from_start -
                     10.91) < 0.01,
        "원본처럼 차로 밖 객체가 멀어져도 1초 이내에는 이전 정지를 유지합니다");
  vanished_outside.stamp_s = 11.0;
  check(outside_core.plan(path, ego, 5.0, 11.0,
                          vanished_outside).empty(),
        "원본 객체 헤더 시각으로 1초가 되면 이전 차로 밖 정지를 끝냅니다");
  auto far_crossing = outside;
  far_crossing.objects.front().x = 35.0;
  for (auto &pose : far_crossing.objects.front().predicted_paths.front().poses)
    pose.x = 35.0;
  planning::PlanningCruiseStopCandidateCore outside_too_late(enabled_outside);
  check(outside_too_late.plan(path, ego, 5.0, 10.0,
                              far_crossing).empty(),
        "원본처럼 자차·보행자 충돌 시각 차이가 크면 정지를 생략합니다");
  auto short_horizon = enabled_outside;
  short_horizon.pedestrian_deceleration_mps2 = 2.0;
  planning::PlanningCruiseStopCandidateCore outside_short_horizon(short_horizon);
  check(outside_short_horizon.plan(path, ego, 5.0, 10.0,
                                    outside).empty(),
        "보행자 감속률이 예측 검사 범위를 제한합니다");
  auto curved_path = path;
  for (std::size_t index = 0; index < curved_path.size(); ++index) {
    curved_path[index].y = 0.005 * index * index;
    const double angle = std::atan(0.01 * index);
    curved_path[index].orientation_z = std::sin(angle * 0.5);
    curved_path[index].orientation_w = std::cos(angle * 0.5);
  }
  auto curved_outside = outside;
  curved_outside.objects.front().y += 1.125;
  for (auto &pose : curved_outside.objects.front().predicted_paths.front().poses)
    pose.y += 1.125;
  planning::PlanningCruiseStopCandidateCore outside_curve(enabled_outside);
  const auto curved_candidates = outside_curve.plan(
      curved_path, ego, 5.0, 10.0, curved_outside);
  check(curved_candidates.size() == 1 &&
            std::abs(curved_candidates.front().collision_distance_from_start -
                     10.83567047) < 1.0e-4,
        "곡선에서도 원본 기존 주행 다각형의 충돌 거리와 일치합니다");
  curved_outside.objects.front().shape_type = 2;
  curved_outside.objects.front().length_m = 0.4;
  curved_outside.objects.front().footprint = {
      {0.0, -0.3, 0.0}, {0.4, 0.0, 0.0}, {0.0, 0.3, 0.0}};
  planning::PlanningCruiseStopCandidateCore outside_polygon(enabled_outside);
  const auto polygon_candidates = outside_polygon.plan(
      curved_path, ego, 5.0, 10.0, curved_outside);
  check(polygon_candidates.size() == 1 &&
            std::abs(polygon_candidates.front().collision_distance_from_start -
                     11.13885880) < 1.0e-4,
        "원본 다각형 외형의 곡선 충돌 거리와 일치합니다");
  curved_outside.objects.front().length_m = 0.0;
  curved_outside.objects.front().width_m = 0.0;
  planning::PlanningCruiseStopCandidateCore polygon_without_dimensions(
      enabled_outside);
  const auto dimensionless_polygon_candidates =
      polygon_without_dimensions.plan(curved_path, ego, 5.0, 10.0,
                                      curved_outside);
  check(dimensionless_polygon_candidates.size() == 1 &&
            std::abs(dimensionless_polygon_candidates.front()
                         .collision_distance_from_start -
                     polygon_candidates.front().collision_distance_from_start) <
                1.0e-6,
        "다각형 객체의 외곽 점으로 크기를 구하며 크기 필드 0은 버리지 않습니다");
  planning::CruiseSlowdownCandidateState slowdown_state;
  for (int index = 0; index < 5; ++index) {
    slowdown_state.beginFrame();
    const bool active = slowdown_state.shouldSlowDown("slowdown", 0.75);
    check(active == (index == 4),
          "감속 객체는 원본처럼 5회 연속 진입 뒤 활성화됩니다");
    if (active) slowdown_state.accepted("slowdown");
    slowdown_state.endFrame();
  }
  for (int index = 0; index < 5; ++index) {
    slowdown_state.beginFrame();
    const bool active = slowdown_state.shouldSlowDown("slowdown", 1.232);
    check(active == (index < 4),
          "원본 카운터는 5회 연속 이탈 뒤 감속 상태를 해제합니다");
    if (active) slowdown_state.accepted("slowdown");
    slowdown_state.endFrame();
  }
  const auto slowdown_path = planning::resampleCruiseCollisionTrajectory(
      path, 0, 2.0, 6.0);
  const planning::CruiseObjectShape slowdown_shape{0, 0.6, 0.6, {}};
  const auto slowdown_interval = planning::findCruiseSlowdownCollisionInterval(
      slowdown_path, {15.0, 2.0, 0.0}, slowdown_shape,
      3.79, 1.10, 1.896, 1.3, planning::CruisePredictedPose{0, 0, 0});
  check(slowdown_interval &&
            std::abs(slowdown_interval->front_arc_m - 14.7) < 1.0e-6 &&
            std::abs(slowdown_interval->back_arc_m - 15.3) < 1.0e-6,
        "원본 감속 객체의 1.3m 확장 외형 앞뒤 충돌점을 계산합니다");
  const auto exit_interval = planning::findCruiseSlowdownCollisionInterval(
      slowdown_path, {15.0, 2.48, 0.0}, slowdown_shape,
      3.79, 1.10, 1.896, 1.3, planning::CruisePredictedPose{0, 0, 0});
  check(exit_interval.has_value(),
        "이탈 계수의 1.232m 횡거리에서도 확장 외형 충돌이 유지됩니다");
  planning::ObstacleCruiseSlowdownCore slowdown_core;
  const auto slowdown_result = slowdown_core.plan(
      path, {0, 0}, 5.0, 0.0,
      {{"slowdown", slowdown_interval->front_arc_m,
        slowdown_interval->back_arc_m, 0.752, 0.0, 0.0}});
  check(slowdown_result.intervals.slowdowns.size() == 1 &&
            std::abs(slowdown_result.intervals.slowdowns.front().from.x -
                     0.77) < 1.0e-6 &&
            std::abs(slowdown_result.intervals.slowdowns.front().to.x -
                     11.51) < 1.0e-6 &&
            std::abs(slowdown_result.intervals.slowdowns.front().velocity -
                     6.76) < 1.0e-6,
        "원본 감속 디버그의 시작·끝 거리와 목표 속도와 일치합니다");
  planning::ObstacleCruiseSlowdownCore moving_slowdown_core;
  const auto moving_slowdown_result = moving_slowdown_core.plan(
      path, {0, 0}, 5.0, 0.0,
      {{"moving_slowdown", slowdown_interval->front_arc_m,
        slowdown_interval->back_arc_m, 0.752, 0.6, 0.0}});
  check(moving_slowdown_result.intervals.slowdowns.size() == 1 &&
            std::abs(moving_slowdown_result.intervals.slowdowns.front().from.x -
                     2.58772731) < 1.0e-5 &&
            std::abs(moving_slowdown_result.intervals.slowdowns.front().to.x -
                     12.39772701) < 1.0e-5 &&
            std::abs(moving_slowdown_result.intervals.slowdowns.front().velocity -
                     6.14) < 1.0e-5,
        "이동 객체의 원본 감속 구간과 목표 속도도 일치합니다");
  const auto curved_slowdown_path =
      planning::resampleCruiseCollisionTrajectory(curved_path, 0, 2.0, 6.0);
  const planning::CruisePredictedPose curved_slowdown_pose{15.0, 3.125, 0.0};
  const double curved_lateral = planning::cruiseObjectLateralDistance(
      curved_slowdown_path, curved_slowdown_pose, slowdown_shape,
      3.79, 1.10, 1.896, planning::CruisePredictedPose{0, 0, 0});
  const auto curved_interval = planning::findCruiseSlowdownCollisionInterval(
      curved_slowdown_path, curved_slowdown_pose, slowdown_shape,
      3.79, 1.10, 1.896, 1.3,
      planning::CruisePredictedPose{0, 0, 0});
  const auto curved_projection = planning::projectCruiseObjectToTrajectory(
      curved_path, 0, {15.0, 3.125, 0.0, 0.6, 0.0});
  check(curved_interval && curved_projection &&
            std::abs(curved_lateral - 0.65547746) < 1.0e-5,
        "곡선에서 원본 객체의 정밀 횡거리와 충돌점을 계산합니다");
  planning::ObstacleCruiseSlowdownCore curved_slowdown_core;
  const auto curved_slowdown_result = curved_slowdown_core.plan(
      curved_path, {0, 0}, 5.0, 0.0,
      {{"curved_slowdown",
        planning::dynamicStopArc(curved_path,
            {curved_interval->front_point.x, curved_interval->front_point.y}),
        planning::dynamicStopArc(curved_path,
            {curved_interval->back_point.x, curved_interval->back_point.y}),
        curved_lateral, curved_projection->longitudinal_velocity_mps,
        curved_projection->approach_velocity_mps}});
  const auto &curved_slowdown =
      curved_slowdown_result.intervals.slowdowns.front();
  check(std::abs(planning::dynamicStopArc(curved_path, curved_slowdown.from) -
                 4.00542831) < 1.0e-4 &&
            std::abs(planning::dynamicStopArc(curved_path, curved_slowdown.to) -
                     12.83124924) < 1.0e-4 &&
            std::abs(curved_slowdown.velocity - 5.41608095) < 1.0e-4,
        "곡선 충돌점은 원본처럼 원 궤적에 다시 투영해 감속 구간을 만듭니다");
  planning::CruiseSlowdownCandidateState exit_state;
  planning::ObstacleCruiseSlowdownCore exit_core;
  const double expected_exit_starts[] = {0.77, 0.76892, 0.211244};
  for (int frame = 240; frame < 256; ++frame) {
    const double object_y = frame < 245 ? 2.0 : 2.48;
    const double precise_lateral = planning::cruiseObjectLateralDistance(
        slowdown_path, {15.0, object_y, 0.0}, slowdown_shape,
        3.79, 1.10, 1.896, planning::CruisePredictedPose{0, 0, 0});
    const auto collision = planning::findCruiseSlowdownCollisionInterval(
        slowdown_path, {15.0, object_y, 0.0}, slowdown_shape,
        3.79, 1.10, 1.896, 1.3,
        planning::CruisePredictedPose{0, 0, 0});
    exit_state.beginFrame();
    const bool active = exit_state.shouldSlowDown(
        "exit_slowdown", precise_lateral) && collision.has_value();
    if (active) exit_state.accepted("exit_slowdown");
    exit_state.endFrame();
    const std::vector<planning::ObstacleCruiseSlowdownCandidate> frame_candidates =
        active ? std::vector<planning::ObstacleCruiseSlowdownCandidate>{
                     {"exit_slowdown", collision->front_arc_m,
                      collision->back_arc_m, precise_lateral, 0.0, 0.0}}
               : std::vector<planning::ObstacleCruiseSlowdownCandidate>{};
    const auto frame_result = exit_core.plan(
        path, {0, 0}, 5.0, 0.0, frame_candidates);
    check(active == (frame >= 244 && frame <= 248) &&
              frame_result.intervals.slowdowns.size() ==
                  static_cast<std::size_t>(active),
          "원본 합성 감속은 5프레임 진입 뒤 5프레임 이탈로 해제됩니다");
    if (frame >= 244 && frame <= 246) {
      check(std::abs(frame_result.intervals.slowdowns.front().from.x -
                     expected_exit_starts[frame - 244]) < 1.0e-5,
            "연속 감속의 원본 이전 상태 저역 통과 시작점을 보존합니다");
    }
  }
  auto noncrossing_path = crossing_path;
  noncrossing_path.confidence = 2.0;
  noncrossing_path.poses = {{15.0, 3.0, 0.0}, {15.0, 3.0, 0.0},
                            {15.0, 3.0, 0.0}};
  outside.objects.front().predicted_paths = {noncrossing_path, crossing_path};
  planning::PlanningCruiseStopCandidateCore one_outside_path(enabled_outside);
  check(one_outside_path.plan(path, ego, 5.0, 10.0, outside).empty(),
        "차로 밖 정지는 기본적으로 최고 신뢰도 예측 경로 한 개만 검사합니다");
  enabled_outside.outside_stop_predicted_path_count = 2;
  planning::PlanningCruiseStopCandidateCore two_outside_paths(enabled_outside);
  check(two_outside_paths.plan(path, ego, 5.0, 10.0, outside).size() == 1,
        "예측 경로 검사 수를 늘리면 다음 충돌 경로를 검사합니다");
}
