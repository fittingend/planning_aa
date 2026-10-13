#include "planning/behavior_route_action.hpp"

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
}  // 익명 네임스페이스

int main() {
  planning::BehaviorGoalPath phased_path;
  for (int index = 0; index < 9; ++index) {
    planning::TrajectoryPoint point;
    point.x = index;
    point.longitudinal_velocity_mps = 2.0F;
    phased_path.points.push_back(point);
    phased_path.lane_ids.push_back({17});
  }
  planning::BehaviorPullOverPhaseState phase_state;
  const std::vector<std::size_t> phase_ends{2, 5, 8};
  const auto phase_at = [&](double time_s, double x, double speed_mps,
                            planning::BehaviorGoalPose goal = {8.0, 0.0, 0.0}) {
    planning::TrajectoryPoint pose;
    pose.x = x;
    return planning::selectBehaviorPullOverPhase(
        phased_path, phase_ends, pose, speed_mps, time_s,
        goal, phase_state);
  };
  check(phase_at(0.0, 0.0, 0.0).points.size() == 3 &&
            phase_state.index == 0 &&
            phase_at(5.0, 2.0, 1.0).points.size() == 3 &&
            phase_at(5.0, 0.0, 0.0).points.size() == 3,
        "주차 구간 전환에는 정차와 구간 끝 도착이 모두 필요합니다");
  const auto second_phase = phase_at(5.0, 2.0, 0.0);
  check(phase_state.index == 1 && second_phase.points.front().x == 2.0 &&
            second_phase.points.back().x == 5.0 &&
            second_phase.points.back().longitudinal_velocity_mps == 0.0F &&
            second_phase.lane_ids.size() == second_phase.points.size(),
        "다음 주차 구간은 앞 구간 종점에서 시작하고 종점에 정지합니다");
  const auto third_phase = phase_at(9.0, 5.0, 0.0);
  check(phase_state.index == 2 && third_phase.points.front().x == 5.0 &&
            third_phase.points.back().x == 8.0,
        "두 번째 정차 후 마지막 주차 구간을 선택합니다");
  check(phase_at(10.0, 8.0, 0.0, {9.0, 0.0, 0.0}).points.back().x == 2.0 &&
            phase_state.index == 0,
        "목표가 바뀌면 주차 구간을 처음부터 다시 선택합니다");
  planning::BehaviorRoutePhaseState start_phase_state;
  planning::TrajectoryPoint stopped_at_turn;
  stopped_at_turn.x = 2.0;
  const auto first_start_phase = planning::selectBehaviorStartPhase(
      phased_path, {2, 8}, stopped_at_turn, 0.0, 0.0,
      {8.0, 0.0, 0.0}, start_phase_state);
  const auto second_start_phase = planning::selectBehaviorStartPhase(
      phased_path, {2, 8}, stopped_at_turn, 0.0, 5.0,
      {8.0, 0.0, 0.0}, start_phase_state);
  check(first_start_phase.points.back().x == 2.0 &&
            second_start_phase.points.front().x == 2.0 &&
            second_start_phase.points.back().x == 8.0 &&
            second_start_phase.points.back().longitudinal_velocity_mps ==
                2.0F &&
            start_phase_state.index == 1,
        "자유공간 출발의 후진 구간 종점 정차 뒤 전진 구간을 선택합니다");
  planning::BehaviorGoalPath reference;
  for (int index = 0; index <= 80; ++index) {
    planning::TrajectoryPoint point;
    point.x = index;
    point.longitudinal_velocity_mps = 5.0F;
    reference.points.push_back(point);
    reference.lane_ids.push_back({17});
  }
  const std::vector<std::vector<planning::BehaviorPathPoint2d>> lanes{
      {{-10.0, -5.0}, {120.0, -5.0},
       {120.0, 5.0}, {-10.0, 5.0}}};
  auto ego = reference.points[5];
  planning::BehaviorRouteActionInput sampling_request;
  sampling_request.sampling = planning::BehaviorRouteSamplingRequest{};
  sampling_request.sampling->require_unsafe_reference = false;
  sampling_request.sampling->sampling.target_lengths_m = {20.0};
  sampling_request.sampling->sampling.target_lateral_positions_m =
      {0.0, 2.0};
  sampling_request.sampling->sampling.target_lateral_derivatives = {0.0};
  sampling_request.sampling->maximum_curvature_per_m = 0.3;
  const auto sampled = planning::composeBehaviorRouteAction(
      reference, ego, lanes, sampling_request);
  check(sampled.kind == planning::BehaviorRouteActionKind::Sampling &&
            sampled.sampling_candidate_count == 2 &&
            std::abs(sampled.path.points.back().y) < 1.0e-6,
        "표본 계획기는 차로 안에서 비용이 낮은 중심 경로를 선택합니다");
  sampling_request.sampling->require_unsafe_reference = true;
  const auto safe_reference = planning::composeBehaviorRouteAction(
      reference, ego, lanes, sampling_request);
  check(safe_reference.kind ==
            planning::BehaviorRouteActionKind::LaneFollow,
        "기준 경로에 장애물이 없으면 표본 계획기를 요청하지 않습니다");
  sampling_request.sampling->obstacle_polygons.push_back(
      {{19.0, -0.2}, {21.0, -0.2}, {21.0, 0.2}, {19.0, 0.2}});
  const auto sampled_around_object = planning::composeBehaviorRouteAction(
      reference, ego, lanes, sampling_request);
  check(sampled_around_object.kind ==
            planning::BehaviorRouteActionKind::Sampling &&
            sampled_around_object.path.points.back().y > 1.9,
        "표본 계획기는 객체 외곽과 겹치는 중심 후보를 제외합니다");
  planning::BehaviorRouteLaneChangeRequest::AvoidanceActivation
      avoidance_lane_change;
  avoidance_lane_change.objects.push_back(
      {80.0, 25.0, 1, true});
  avoidance_lane_change.minimum_lane_change_buffer_m = 30.0;
  check(std::abs(planning::behaviorMinimumLaneChangeDistanceBuffer(
            6.0, 2.78, 0.5, 0.65, 2.0) -
            22.888759150052067) < 1.0e-8,
        "한 차로 변경의 최소 길이를 횡저크·횡가속도와 종료 여유로 계산합니다");
  check(planning::behaviorAvoidanceByLaneChangeRequested(
            avoidance_lane_change),
        "가장 가까운 회피 객체까지 필요한 두 거리보다 길면 차로 변경을 요청합니다");
  avoidance_lane_change.objects.front().longitudinal_distance_m = 20.0;
  check(!planning::behaviorAvoidanceByLaneChangeRequested(
            avoidance_lane_change),
        "회피 객체가 너무 가까우면 차로 변경을 요청하지 않습니다");
  avoidance_lane_change.objects.front().longitudinal_distance_m = 80.0;
  avoidance_lane_change.target_types[1] = false;
  check(!planning::behaviorAvoidanceByLaneChangeRequested(
            avoidance_lane_change),
        "대상 종류가 아닌 객체는 회피 차로 변경을 켜지 않습니다");
  planning::BehaviorRouteActionInput side_shift_request;
  side_shift_request.side_shift =
      planning::BehaviorRouteSideShiftRequest{};
  side_shift_request.side_shift->requested_lateral_offset_m = 1.0;
  const auto waiting_side_shift = planning::composeBehaviorRouteAction(
      reference, ego, lanes, side_shift_request);
  check(waiting_side_shift.kind ==
            planning::BehaviorRouteActionKind::LaneFollow &&
            waiting_side_shift.side_shift_candidate_count == 1,
        "사이드 시프트 승인 전에는 후보만 계산합니다");
  side_shift_request.side_shift->approved = true;
  const auto approved_side_shift = planning::composeBehaviorRouteAction(
      reference, ego, lanes, side_shift_request);
  check(approved_side_shift.kind ==
            planning::BehaviorRouteActionKind::SideShift &&
            std::abs(approved_side_shift.path.points.back().y - 1.0) <
                1.0e-6 &&
            approved_side_shift.path.points.size() ==
                approved_side_shift.path.lane_ids.size(),
        "승인한 사이드 시프트 경로에 차로 ID를 보존합니다");
  side_shift_request.side_shift->forward_path_length_m = 50.0;
  side_shift_request.side_shift->backward_path_length_m = 2.0;
  side_shift_request.side_shift->input_path_interval_m = 2.0;
  const auto cropped_side_shift = planning::composeBehaviorRouteAction(
      reference, ego, lanes, side_shift_request);
  check(cropped_side_shift.kind ==
            planning::BehaviorRouteActionKind::SideShift &&
            cropped_side_shift.path.points.front().x == 1.0 &&
            cropped_side_shift.path.points.back().x == 55.0 &&
            cropped_side_shift.side_shift_end_index &&
            *cropped_side_shift.side_shift_end_index <
                cropped_side_shift.path.points.size(),
        "사이드 시프트 경로를 자차 앞뒤 길이로 자르고 이동 끝 인덱스를 보정합니다");
  ego.y = -1.0;
  planning::BehaviorRouteActionInput start;
  start.start = planning::BehaviorRouteStartRequest{};
  start.start->start = {ego.x, ego.y};
  start.start->lateral_offset_m = -1.0;
  const auto departure = planning::composeBehaviorRouteAction(
      reference, ego, lanes, start);
  check(departure.kind == planning::BehaviorRouteActionKind::StartPullOut &&
            departure.start_candidate_count > 0 &&
            departure.path.points.size() == departure.path.lane_ids.size() &&
            std::abs(departure.path.points[5].y) > 0.05,
        "출발 후보를 차로 포함 검사 뒤 차로 ID와 함께 출력합니다");
  auto backward_shift_start = start;
  backward_shift_start.start->start_pose_candidates = {
      {0.0, 3.0, -1.0, 0.0, 0.0}};
  const auto backward_shift_departure =
      planning::composeBehaviorRouteAction(
          reference, ego, lanes, backward_shift_start);
  check(backward_shift_departure.kind ==
            planning::BehaviorRouteActionKind::StartPullOut &&
            backward_shift_departure.start_phase_end_indices.size() == 2 &&
            backward_shift_departure.path.points[
                backward_shift_departure.start_phase_end_indices.front()].x ==
                3.0,
        "일반 횡이동 출발에도 후방 자세의 정지 단계를 붙입니다");
  auto blocked_shift_start = start;
  blocked_shift_start.start->path_stop_objects.push_back(
      {{19.0, -4.0}, {21.0, -4.0},
       {21.0, 4.0}, {19.0, 4.0}});
  const auto blocked_shift_departure =
      planning::composeBehaviorRouteAction(
          reference, ego, lanes, blocked_shift_start);
  check(blocked_shift_departure.kind ==
            planning::BehaviorRouteActionKind::LaneFollow,
        "일반 횡이동 출발 경로도 같은 콜백의 정지 객체를 검사합니다");
  ego = reference.points[5];
  const auto forward_parking =
      planning::generateGeometricPullOverCandidate(
          reference.points, ego, {60.0, -2.0, 0.0});
  check(forward_parking &&
            std::hypot(forward_parking->path.back().x - 60.0,
                       forward_parking->path.back().y + 2.0) < 1.0e-6 &&
            forward_parking->shift_end_index >
                forward_parking->shift_start_index + 2 &&
            forward_parking->phase_end_indices.size() == 3 &&
            forward_parking->path[
                forward_parking->phase_end_indices[1]]
                    .longitudinal_velocity_mps == 0.0F,
        "전진 주차의 서로 반대 방향 원호와 목표 직선을 생성합니다");
  planning::GeometricPullOverParameters backward_parameters;
  backward_parameters.forward = false;
  const auto backward_parking =
      planning::generateGeometricPullOverCandidate(
          reference.points, ego, {60.0, -2.0, 0.0},
          backward_parameters);
  check(backward_parking &&
            backward_parking->path.back().longitudinal_velocity_mps == 0.0F &&
            backward_parking->path[backward_parking->shift_start_index + 1]
                    .longitudinal_velocity_mps < 0.0F &&
            backward_parking->phase_end_indices.size() == 3 &&
            backward_parking->path[
                backward_parking->phase_end_indices[1]]
                    .longitudinal_velocity_mps == 0.0F,
        "후진 주차 원호에는 음수 속도를 부여하고 목표에서 정지합니다");
  auto parked_start_pose = reference.points[60];
  parked_start_pose.y = -2.0;
  planning::BehaviorRouteActionInput geometric_start;
  geometric_start.start = planning::BehaviorRouteStartRequest{};
  geometric_start.start->use_geometric = true;
  geometric_start.start->vehicle = {0.1, 0.1, 0.1, 0.5};
  const auto geometric_departure = planning::composeBehaviorRouteAction(
      reference, parked_start_pose, lanes, geometric_start);
  check(geometric_departure.kind ==
            planning::BehaviorRouteActionKind::StartPullOut &&
            geometric_departure.start_phase_end_indices.size() == 2 &&
            geometric_departure.path.points.front().y == -2.0 &&
            geometric_departure.path.points.back().x == 80.0,
        "역방향 기하 주차 원호를 출발 경로로 바꾸고 도로 경로를 이어 붙입니다");
  planning::GeometricPullOverParameters asymmetric_parameters;
  asymmetric_parameters.far_turn_radius_m = 10.0;
  const auto asymmetric_parking =
      planning::generateGeometricPullOverCandidate(
          reference.points, ego, {60.0, -2.0, 0.0},
          asymmetric_parameters);
  check(asymmetric_parking &&
            std::hypot(asymmetric_parking->path.back().x - 60.0,
                       asymmetric_parking->path.back().y + 2.0) < 1.0e-6 &&
            asymmetric_parking->shift_end_index >
                asymmetric_parking->shift_start_index + 2,
        "가까운 원호와 먼 원호의 반경이 달라도 목표 자세에 연결합니다");
  asymmetric_parameters.forward = false;
  const auto asymmetric_backward =
      planning::generateGeometricPullOverCandidate(
          reference.points, ego, {60.0, -2.0, 0.0},
          asymmetric_parameters);
  check(asymmetric_backward &&
            asymmetric_backward->path[
                asymmetric_backward->shift_start_index + 1]
                    .longitudinal_velocity_mps < 0.0F,
        "비대칭 후진 원호에도 후진 속도와 연속 경로를 부여합니다");
  planning::Trajectory curved_reference;
  for (int index = 0; index <= 100; ++index) {
    auto point = reference.points[index <= 80 ? index : 80];
    point.x = index;
    point.y = 0.01 * index * index;
    const double yaw = std::atan(0.02 * index);
    point.orientation_z = std::sin(yaw / 2.0);
    point.orientation_w = std::cos(yaw / 2.0);
    curved_reference.push_back(point);
  }
  auto curved_ego = curved_reference[5];
  const double curved_goal_yaw = std::atan(0.02 * 60.0);
  const planning::BehaviorGoalPose curved_goal{
      60.0 + 2.0 * std::sin(curved_goal_yaw),
      curved_reference[60].y - 2.0 * std::cos(curved_goal_yaw),
      curved_goal_yaw};
  const auto curved_geometric_parking =
      planning::generateCurvedGeometricPullOverCandidate(
          curved_reference, curved_ego, curved_goal);
  check(curved_geometric_parking &&
            curved_geometric_parking->phase_end_indices.size() == 3 &&
            std::hypot(curved_geometric_parking->path.back().x - curved_goal.x,
                       curved_geometric_parking->path.back().y - curved_goal.y) < 1.0e-6,
        "곡선 차로의 접선에서 두 원호를 접속해 주차 목표에 도달합니다");
  for (std::size_t index = 1;
       index < curved_geometric_parking->path.size(); ++index)
    check(std::hypot(
              curved_geometric_parking->path[index].x -
                  curved_geometric_parking->path[index - 1].x,
              curved_geometric_parking->path[index].y -
                  curved_geometric_parking->path[index - 1].y) < 1.5,
          "곡선 차로의 기준선과 원호 사이에 위치 비약이 없습니다");
  planning::GeometricPullOverParameters curved_backward_parameters;
  curved_backward_parameters.forward = false;
  const auto curved_backward_parking =
      planning::generateCurvedGeometricPullOverCandidate(
          curved_reference, curved_ego, curved_goal,
          curved_backward_parameters);
  check(curved_backward_parking &&
            curved_backward_parking->phase_end_indices.size() == 3 &&
            curved_backward_parking->path[
                curved_backward_parking->shift_start_index + 1]
                    .longitudinal_velocity_mps < 0.0F,
        "곡선 차로의 후진 원호와 단계별 정지점을 생성합니다");
  planning::BehaviorGoalPath curved_behavior_reference;
  curved_behavior_reference.points = curved_reference;
  curved_behavior_reference.lane_ids.assign(
      curved_reference.size(), {17});
  planning::BehaviorRouteActionInput curved_action;
  curved_action.pull_over = planning::BehaviorRoutePullOverRequest{};
  curved_action.pull_over->goal = curved_goal;
  curved_action.pull_over->approved = true;
  curved_action.pull_over->use_geometric = true;
  curved_action.pull_over->vehicle.front_m = 0.1;
  curved_action.pull_over->vehicle.back_m = 0.1;
  curved_action.pull_over->vehicle.half_width_m = 0.1;
  const std::vector<std::vector<planning::BehaviorPathPoint2d>>
      curved_lanes{{{-20.0, -20.0}, {120.0, -20.0},
                    {120.0, 120.0}, {-20.0, 120.0}}};
  auto curved_start_pose = curved_reference[60];
  curved_start_pose.x = curved_goal.x;
  curved_start_pose.y = curved_goal.y;
  const auto curved_departure = planning::composeBehaviorRouteAction(
      curved_behavior_reference, curved_start_pose,
      curved_lanes, geometric_start);
  check(curved_departure.kind ==
            planning::BehaviorRouteActionKind::StartPullOut &&
            curved_departure.start_phase_end_indices.size() == 2 &&
            std::hypot(curved_departure.path.points.front().x -
                           curved_goal.x,
                       curved_departure.path.points.front().y -
                           curved_goal.y) < 1.0e-6,
        "곡선 차로의 후진 주차 원호도 기하 출발로 역순 연결합니다");
  const auto curved_action_result = planning::composeBehaviorRouteAction(
      curved_behavior_reference, curved_ego, curved_lanes,
      curved_action);
  check(curved_action_result.kind ==
            planning::BehaviorRouteActionKind::PullOver &&
            curved_action_result.pull_over_phase_end_indices.size() == 3 &&
            std::hypot(
                curved_action_result.path.points[
                    curved_action_result.pull_over_phase_end_indices[0]].x -
                    curved_geometric_parking->path[
                        curved_geometric_parking->phase_end_indices[0]].x,
                curved_action_result.path.points[
                    curved_action_result.pull_over_phase_end_indices[0]].y -
                    curved_geometric_parking->path[
                        curved_geometric_parking->phase_end_indices[0]].y) < 0.01,
        "곡선 차로에서는 접선 원호 후보를 행동 경로에 먼저 연결합니다");
  planning::BehaviorRouteActionInput geometric_action;
  geometric_action.pull_over = planning::BehaviorRoutePullOverRequest{};
  geometric_action.pull_over->goal = {60.0, -2.0, 0.0};
  geometric_action.pull_over->approved = true;
  geometric_action.pull_over->use_geometric = true;
  geometric_action.pull_over->vehicle.front_m = 0.1;
  geometric_action.pull_over->vehicle.back_m = 0.1;
  geometric_action.pull_over->vehicle.half_width_m = 0.1;
  const auto geometric_result = planning::composeBehaviorRouteAction(
      reference, ego, lanes, geometric_action);
  check(geometric_result.kind ==
            planning::BehaviorRouteActionKind::PullOver &&
            geometric_result.pull_over_candidate_count > 0 &&
            geometric_result.pull_over_phase_end_indices.size() == 3,
        "기하 주차 후보를 행동 경로 선택과 차로 포함 검사에 연결합니다");
  geometric_action.pull_over->path_stop_objects.push_back(
      {{35.0, -0.5}, {38.0, -0.5},
       {38.0, 0.5}, {35.0, 0.5}});
  const auto blocked_geometric = planning::composeBehaviorRouteAction(
      reference, ego, lanes, geometric_action);
  check(blocked_geometric.kind ==
            planning::BehaviorRouteActionKind::LaneFollow &&
            blocked_geometric.pull_over_candidate_count > 0,
        "목표 앞 경로의 정지 객체 외곽과 겹치는 주차 후보를 거부합니다");
  geometric_action.pull_over->path_stop_objects.clear();
  geometric_action.pull_over->path_moving_objects.push_back(
      {{35.0, -0.5}, {38.0, -0.5},
       {38.0, 0.5}, {35.0, 0.5}});
  const auto moving_geometric = planning::composeBehaviorRouteAction(
      reference, ego, lanes, geometric_action);
  check(moving_geometric.kind ==
            planning::BehaviorRouteActionKind::LaneFollow &&
            moving_geometric.pull_over_candidate_count > 0,
        "이동 객체가 현재 주차 경로를 차지하면 실행을 보류합니다");
  planning::Trajectory stopping_path(2);
  stopping_path[0].longitudinal_velocity_mps = 5.0F;
  stopping_path[1].x = 1.0;
  stopping_path[1].longitudinal_velocity_mps = 5.0F;
  const std::vector<planning::BehaviorSafetyPolygon> near_front_object{
      {{5.0, -0.5}, {6.0, -0.5},
       {6.0, 0.5}, {5.0, 0.5}}};
  planning::BehaviorPathVehicleFootprint stopping_vehicle;
  stopping_vehicle.front_m = 3.79;
  stopping_vehicle.back_m = 0.1;
  stopping_vehicle.half_width_m = 0.5;
  check(!planning::behaviorPullOverPathCollides(
            stopping_path, stopping_path.front(),
            stopping_vehicle, near_front_object, 0.0, 1.0, 0.0) &&
            planning::behaviorPullOverPathCollides(
                stopping_path, stopping_path.front(),
                stopping_vehicle, near_front_object, 0.0, 1.0, 1.0),
        "이동 객체에는 속도에 따른 앞쪽 최대 정지 여유를 추가합니다");
  planning::CrosswalkOccupancyGrid parking_grid;
  parking_grid.width = 60;
  parking_grid.height = 60;
  parking_grid.resolution_m = 1.0;
  parking_grid.origin_x = -10.0;
  parking_grid.origin_y = -30.0;
  parking_grid.values.assign(
      parking_grid.width * parking_grid.height, 0);
  planning::FreespacePullOverParameters free_parameters;
  free_parameters.vehicle_shape_margin_m = 0.0;
  free_parameters.maximum_expansions = 30000;
  planning::BehaviorRouteActionInput free_start;
  free_start.start = planning::BehaviorRouteStartRequest{};
  free_start.start->use_freespace = true;
  free_start.start->freespace_grid = parking_grid;
  free_start.start->freespace = free_parameters;
  auto offset_ego = ego;
  offset_ego.y = -2.0;
  const auto freespace_departure = planning::composeBehaviorRouteAction(
      reference, offset_ego, lanes, free_start);
  check(freespace_departure.kind ==
            planning::BehaviorRouteActionKind::StartPullOut &&
            freespace_departure.start_candidate_count == 1 &&
            !freespace_departure.start_phase_end_indices.empty() &&
            freespace_departure.start_phase_end_indices.back() ==
                freespace_departure.path.points.size() - 1 &&
            std::abs(freespace_departure.path.points.front().y + 2.0) <
                1.0e-6 &&
            freespace_departure.path.points.back().x == 80.0,
        "점유 격자에서 출발 자세와 도로 중심선 합류 자세를 잇습니다");
  auto backed_start = free_start;
  backed_start.start->start_pose_candidates = {
      {0.0, 3.0, -2.0, 0.0, 0.0}};
  const auto backed_departure = planning::composeBehaviorRouteAction(
      reference, offset_ego, lanes, backed_start);
  check(backed_departure.kind ==
            planning::BehaviorRouteActionKind::StartPullOut &&
            backed_departure.selected_start_pose &&
            backed_departure.start_phase_end_indices.size() == 2 &&
            backed_departure.path.points.front().x == 5.0 &&
            backed_departure.path.points[
                backed_departure.start_phase_end_indices.front()].x ==
                3.0 &&
            backed_departure.path.points[1]
                    .longitudinal_velocity_mps < 0.0F,
        "안전한 후방 출발 자세를 선택해 정지점까지 후진 경로를 앞에 붙입니다");
  free_start.start->path_stop_objects.push_back(
      {{7.0, -10.0}, {19.0, -10.0},
       {19.0, 10.0}, {7.0, 10.0}});
  const auto blocked_departure = planning::composeBehaviorRouteAction(
      reference, offset_ego, lanes, free_start);
  check(blocked_departure.kind ==
            planning::BehaviorRouteActionKind::LaneFollow,
        "자유공간 출발 경로에 새 정지 객체가 있으면 거부합니다");
  const auto free_parking =
      planning::generateFreespacePullOverCandidate(
          ego, {12.0, -2.0, 0.0}, parking_grid,
          free_parameters);
  check(free_parking &&
            free_parking->path.size() > 3 &&
            std::hypot(free_parking->path.back().x - 12.0,
                       free_parking->path.back().y + 2.0) < 1.0e-6 &&
            !free_parking->phase_end_indices.empty() &&
            free_parking->phase_end_indices.back() ==
                free_parking->path.size() - 1 &&
            free_parking->path.size() >
                free_parking->shift_end_index + 2 &&
            std::abs(free_parking->path[
                free_parking->path.size() - 2].y + 2.0) < 1.0e-6,
        "빈 점유 격자에서 차량 조향 원호로 자유공간 주차 경로를 찾습니다");
  free_parameters.use_reeds_shepp_heuristic = true;
  const auto reeds_shepp_parking =
      planning::generateFreespacePullOverCandidate(
          ego, {12.0, -2.0, 0.0}, parking_grid,
          free_parameters);
  check(reeds_shepp_parking &&
            !reeds_shepp_parking->phase_end_indices.empty(),
        "원본 Reeds–Shepp 거리 휴리스틱으로 자유공간 경로를 검색합니다");
  free_parameters.use_reeds_shepp_heuristic = false;
  free_parameters.use_backward_goal_approach = true;
  const auto backward_goal_parking =
      planning::generateFreespacePullOverCandidate(
          ego, {12.0, -2.0, 0.0}, parking_grid,
          free_parameters);
  check(backward_goal_parking &&
            backward_goal_parking->path.size() ==
                backward_goal_parking->shift_end_index + 2,
        "후진 목표 접근은 목표 앞 직선 구간을 추가하지 않습니다");
  free_parameters.use_backward_goal_approach = false;
  free_parameters.use_rrt_star = true;
  free_parameters.rrt_iterations = 100;
  const auto rrt_parking =
      planning::generateFreespacePullOverCandidate(
          ego, {12.0, -2.0, 0.0}, parking_grid,
          free_parameters);
  check(rrt_parking &&
            !rrt_parking->phase_end_indices.empty() &&
            std::hypot(rrt_parking->path.back().x - 12.0,
                       rrt_parking->path.back().y + 2.0) < 0.1 &&
            rrt_parking->path.size() >
                rrt_parking->shift_end_index + 2,
        "원본 RRT* 탐색을 점유 격자와 차량 외곽 검사에 연결합니다");
  const auto repeated_rrt =
      planning::generateFreespacePullOverCandidate(
          ego, {12.0, -2.0, 0.0}, parking_grid,
          free_parameters);
  check(repeated_rrt &&
            repeated_rrt->path.size() == rrt_parking->path.size() &&
            repeated_rrt->phase_end_indices ==
                rrt_parking->phase_end_indices,
        "같은 난수 시드의 RRT* 탐색은 단계 경로를 재현합니다");
  parking_grid.values[28 * parking_grid.width + 22] = 100;
  check(!planning::generateFreespacePullOverCandidate(
            ego, {12.0, -2.0, 0.0}, parking_grid,
            free_parameters),
        "RRT*에서도 목표 외곽과 겹치는 점유 셀을 거부합니다");
  parking_grid.values[28 * parking_grid.width + 22] = 0;
  free_parameters.use_rrt_star = false;
  parking_grid.values[28 * parking_grid.width + 22] = 100;
  check(!planning::generateFreespacePullOverCandidate(
            ego, {12.0, -2.0, 0.0}, parking_grid,
            free_parameters),
        "목표 차량 외곽과 겹친 점유 셀은 주차 목표를 차단합니다");
  parking_grid.values[28 * parking_grid.width + 22] = 0;
  parking_grid.values[30 * parking_grid.width + 20] = 100;
  const auto detour_parking =
      planning::generateFreespacePullOverCandidate(
          ego, {12.0, -2.0, 0.0}, parking_grid,
          free_parameters);
  check(detour_parking && detour_parking->path.size() > 3,
        "점유 장애물을 피해 주차 목표까지 차량 운동 경로를 검색합니다");
  parking_grid.values[30 * parking_grid.width + 20] = 0;
  planning::BehaviorRouteActionInput free_action;
  free_action.pull_over = planning::BehaviorRoutePullOverRequest{};
  free_action.pull_over->goal = {12.0, -2.0, 0.0};
  free_action.pull_over->approved = true;
  free_action.pull_over->use_freespace = true;
  free_action.pull_over->freespace_grid = parking_grid;
  free_action.pull_over->freespace = free_parameters;
  const auto free_result = planning::composeBehaviorRouteAction(
      reference, ego, lanes, free_action);
  check(free_result.kind == planning::BehaviorRouteActionKind::PullOver &&
            free_result.pull_over_candidate_count == 1 &&
            !free_result.pull_over_phase_end_indices.empty() &&
            free_result.pull_over_phase_end_indices.back() ==
                free_result.path.points.size() - 1,
        "자유공간 주차 후보를 승인된 행동 경로에 연결합니다");
  free_action.pull_over->freespace.use_rrt_star = true;
  free_action.pull_over->freespace.rrt_iterations = 100;
  const auto rrt_result = planning::composeBehaviorRouteAction(
      reference, ego, lanes, free_action);
  check(rrt_result.kind == planning::BehaviorRouteActionKind::PullOver &&
            !rrt_result.pull_over_phase_end_indices.empty(),
        "RRT* 주차 후보도 승인된 행동 경로와 단계 끝점에 연결합니다");

  planning::StaticAvoidanceSingleObjectInput object;
  object.ego_x = 5.0;
  object.object_polygon = {
      {30.0, -2.0}, {34.0, -2.0},
      {34.0, -1.0}, {30.0, -1.0}};
  object.closest_reference_pose = reference.points[30];
  object.object_on_right = true;
  object.margin.vehicle_width_m = 2.0;
  object.margin.distance_factor = 1.0;
  object.margin.lateral_hard_margin_m = 0.2;
  object.margin.lateral_soft_margin_m = 0.7;
  object.margin.soft_drivable_bound_margin_m = 0.5;
  object.margin.hard_drivable_bound_margin_m = 0.1;
  object.boundary = planning::StaticAvoidanceBoundaryInput{
      {{0.0, 3.0}, {80.0, 3.0}}, 0.0};
  object.feasibility.front_constant_distance_m = 2.0;
  object.feasibility.prepare_distance_m = 2.0;
  object.feasibility.nominal_avoidance_distance_m = 6.0;
  object.feasibility.avoidance_speed_mps = 5.0;
  object.feasibility.maximum_lateral_jerk_mps3 = 0.5;
  object.feasibility.minimum_distance_jerk_mps3 = 1.0;
  object.feasibility.minimum_distance_speed_mps = 1.0;
  object.outline.front_constant_distance_m = 2.0;
  object.outline.rear_constant_distance_m = 2.0;
  object.outline.maximum_return_distance_m = 10.0;
  object.outline.to_start_point_m = 10.0;
  object.outline.to_return_point_m = 40.0;
  planning::BehaviorRouteActionInput avoidance;
  avoidance.avoidance = planning::BehaviorRouteAvoidanceRequest{};
  avoidance.avoidance->objects.push_back(object);
  ego.y = 0.0;
  const auto waiting = planning::composeBehaviorRouteAction(
      reference, ego, lanes, avoidance);
  check(waiting.kind == planning::BehaviorRouteActionKind::LaneFollow &&
            waiting.avoidance_candidate_count == 1 &&
            waiting.registered_shift_arcs.empty(),
        "승인 전 회피 후보는 출력과 등록 이동선을 바꾸지 않습니다");
  avoidance.avoidance->approved = true;
  const auto approved = planning::composeBehaviorRouteAction(
      reference, ego, lanes, avoidance);
  check(approved.kind == planning::BehaviorRouteActionKind::StaticAvoidance &&
            approved.registered_shift_arcs.size() == 2 &&
            approved.path.points.size() == approved.path.lane_ids.size() &&
            approved.path.points[30].y > 0.5,
        "승인된 회피·복귀 이동선을 출력하고 다음 콜백에 등록합니다");
  ego.x = 6.0;
  const auto continued = planning::composeBehaviorRouteAction(
      reference, ego, lanes, {}, approved.registered_shift_arcs);
  check(continued.kind == planning::BehaviorRouteActionKind::StaticAvoidance &&
            continued.path.points[30].y > 0.5,
        "다음 콜백에서 등록 이동선을 재투영해 유지합니다");

  planning::BehaviorRouteAvoidancePerceptionParameters perception;
  perception.nominal_avoidance_speed_mps = 4.0;
  perception.nominal_lateral_jerk_mps3 = 1.0;
  perception.maximum_prepare_time_s = 1.0;
  const planning::BehaviorRouteObservedObject observed{
      60.0, -1.5, 0.0, 4.0, 1.0, 0.0, 2.1, {}};
  ego = reference.points[5];
  const auto early_objects = planning::makeBehaviorRouteAvoidanceObjects(
      reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
      {{0.0, -3.0}, {80.0, -3.0}},
      {{60.0, -1.5, 0.0, 4.0, 1.0, 0.0, 1.9, {}}},
      perception);
  check(early_objects.empty(),
        "정차 시간이 부족한 인식 객체는 회피 대상으로 선택하지 않습니다");
  const auto selected_objects = planning::makeBehaviorRouteAvoidanceObjects(
      reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
      {{0.0, -3.0}, {80.0, -3.0}}, {observed}, perception);
  check(selected_objects.size() == 1 &&
            selected_objects.front().object_on_right &&
            selected_objects.front().object_polygon.size() == 4 &&
            std::abs(selected_objects.front().margin.lateral_soft_margin_m -
                     0.5) < 1.0e-9 &&
            selected_objects.front().boundary.has_value(),
        "정차 객체와 차로 경계에서 회피 입력을 구성합니다");
  auto overhanging_object = observed;
  overhanging_object.y = -6.0;
  overhanging_object.width_m = 8.0;
  overhanging_object.stationary_duration_s = 4.0;
  overhanging_object.on_current_road_lane = true;
  const auto overhang_inputs = planning::makeBehaviorRouteAvoidanceObjects(
      reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
      {{0.0, -3.0}, {80.0, -3.0}},
      {overhanging_object}, perception);
  check(overhang_inputs.size() == 1 &&
            overhang_inputs.front().object_polygon.size() == 4,
        "중심은 측방 한계 밖이어도 외곽이 차로에 닿는 객체를 유지합니다");
  auto parked_vehicle = observed;
  parked_vehicle.y = -2.5;
  const auto parked_inputs = planning::makeBehaviorRouteAvoidanceObjects(
      reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
      {{0.0, -3.0}, {80.0, -3.0}}, {parked_vehicle}, perception);
  check(parked_inputs.size() == 1 && parked_inputs.front().margin.parked &&
            std::abs(
                parked_inputs.front().margin.parked_lateral_hard_margin_m -
                0.7) < 1.0e-9,
        "차로 가장자리 정차 차량에는 주차 차량 하드 여유를 적용합니다");
  perception.parked_shiftable_ratio = 0.9;
  check(!planning::makeBehaviorRouteAvoidanceObjects(
            reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
            {{0.0, -3.0}, {80.0, -3.0}},
            {parked_vehicle}, perception).front().margin.parked,
        "주차 차량 측방 비율 임계값을 실행 설정에서 조정합니다");
  perception.parked_shiftable_ratio = 0.8;
  parked_vehicle.within_intersection = true;
  check(!planning::makeBehaviorRouteAvoidanceObjects(
            reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
            {{0.0, -3.0}, {80.0, -3.0}},
            {parked_vehicle}, perception).front().margin.parked,
        "교차로 안 정차 차량은 주차 차량으로 분류하지 않습니다");
  auto ambiguous_vehicle = observed;
  ambiguous_vehicle.on_current_road_lane = true;
  check(planning::makeBehaviorRouteAvoidanceObjects(
            reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
            {{0.0, -3.0}, {80.0, -3.0}},
            {ambiguous_vehicle}, perception).empty(),
        "같은 차로의 주차 여부가 불명확한 차량은 3초까지 기다립니다");
  auto ambiguous_motorcycle = ambiguous_vehicle;
  ambiguous_motorcycle.classification = 5;
  check(planning::makeBehaviorRouteAvoidanceObjects(
            reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
            {{0.0, -3.0}, {80.0, -3.0}},
            {ambiguous_motorcycle}, perception).empty(),
        "오토바이에도 모호한 차량 정차 대기 조건을 적용합니다");
  ambiguous_vehicle.stationary_duration_s = 3.1;
  check(planning::makeBehaviorRouteAvoidanceObjects(
            reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
            {{0.0, -3.0}, {80.0, -3.0}},
            {ambiguous_vehicle}, perception).size() == 1,
        "같은 차로의 모호한 차량이 3초 넘게 정지하면 회피 후보로 전달합니다");
  ambiguous_vehicle.initial_displacement_m = 1.2;
  check(planning::makeBehaviorRouteAvoidanceObjects(
            reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
            {{0.0, -3.0}, {80.0, -3.0}},
            {ambiguous_vehicle}, perception).empty(),
        "같은 차로의 모호한 차량은 정차 시작 뒤 1m 넘게 이동하면 제외합니다");
  ambiguous_vehicle.initial_displacement_m = 0.0;
  ambiguous_vehicle.within_intersection = true;
  check(planning::makeBehaviorRouteAvoidanceObjects(
            reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
            {{0.0, -3.0}, {80.0, -3.0}},
            {ambiguous_vehicle}, perception).empty(),
        "교차로 안의 같은 차로 정지 차량은 모호한 회피 후보에서 제외합니다");
  ambiguous_vehicle.yaw_rad = -0.7;
  check(planning::makeBehaviorRouteAvoidanceObjects(
            reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
            {{0.0, -3.0}, {80.0, -3.0}},
            {ambiguous_vehicle}, perception).size() == 1,
        "교차로에서 자차 차로를 이탈하는 방향의 정차 차량은 회피 후보로 남깁니다");
  ambiguous_vehicle.yaw_rad = 0.7;
  check(planning::makeBehaviorRouteAvoidanceObjects(
            reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
            {{0.0, -3.0}, {80.0, -3.0}},
            {ambiguous_vehicle}, perception).empty(),
        "교차로에서 자차 차로로 합류하는 방향의 정차 차량은 제외합니다");
  auto unknown = observed;
  unknown.classification = 0;
  unknown.stationary_duration_s = 2.9;
  const auto unknown_early = planning::makeBehaviorRouteAvoidanceObjects(
      reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
      {{0.0, -3.0}, {80.0, -3.0}}, {unknown}, perception);
  check(unknown_early.empty(),
        "미분류 정차 객체는 3초가 지나기 전 회피 대상에서 제외합니다");
  unknown.stationary_duration_s = 3.1;
  unknown.speed_mps = 0.3;
  check(planning::makeBehaviorRouteAvoidanceObjects(
            reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
            {{0.0, -3.0}, {80.0, -3.0}}, {unknown}, perception).empty(),
        "미분류 객체의 정차 속도 임계값 0.28m/s를 적용합니다");
  unknown.speed_mps = 0.0;
  const auto unknown_input = planning::makeBehaviorRouteAvoidanceObjects(
      reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
      {{0.0, -3.0}, {80.0, -3.0}}, {unknown}, perception);
  check(unknown_input.size() == 1 &&
            std::abs(unknown_input.front().margin.lateral_hard_margin_m +
                     0.2) < 1.0e-9 &&
            std::abs(unknown_input.front().margin.lateral_soft_margin_m -
                     0.7) < 1.0e-9,
        "미분류 객체의 음수 하드 여유와 0.7m 소프트 여유를 적용합니다");
  unknown.on_current_road_lane = true;
  unknown.has_side_road_lane = true;
  check(planning::makeBehaviorRouteAvoidanceObjects(
            reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
            {{0.0, -3.0}, {80.0, -3.0}},
            {unknown}, perception).size() == 1,
        "미분류 객체에는 차량의 이웃 차로 가장자리 필터를 적용하지 않습니다");
  unknown.y = 0.0;
  check(planning::makeBehaviorRouteAvoidanceObjects(
            reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
            {{0.0, -3.0}, {80.0, -3.0}},
            {unknown}, perception).size() == 1,
        "미분류 객체는 중심선 위에서도 별도 분류로 유지합니다");
  unknown = observed;
  unknown.classification = 0;
  unknown.stationary_duration_s = 3.1;
  perception.goal_distance_from_ego_m = 70.0;
  check(planning::makeBehaviorRouteAvoidanceObjects(
            reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
            {{0.0, -3.0}, {80.0, -3.0}}, {unknown}, perception).empty(),
        "목표 바로 앞 객체는 목표 경로 수정이 없을 때 제외합니다");
  perception.allow_goal_modification = true;
  check(planning::makeBehaviorRouteAvoidanceObjects(
            reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
            {{0.0, -3.0}, {80.0, -3.0}}, {unknown}, perception).size() == 1,
        "목표 경로 수정을 허용하면 목표 앞 정차 객체를 유지합니다");
  perception.goal_distance_from_ego_m.reset();
  perception.allow_goal_modification = false;
  auto pedestrian = observed;
  pedestrian.classification = 7;
  pedestrian.stationary_duration_s = 1.2;
  pedestrian.within_crosswalk = true;
  check(planning::makeBehaviorRouteAvoidanceObjects(
            reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
            {{0.0, -3.0}, {80.0, -3.0}}, {pedestrian}, perception).empty(),
        "횡단보도 안 보행자는 정적 회피 대상으로 선택하지 않습니다");
  pedestrian.within_crosswalk = false;
  check(planning::makeBehaviorRouteAvoidanceObjects(
            reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
            {{0.0, -3.0}, {80.0, -3.0}}, {pedestrian}, perception).size() == 1,
        "횡단보도 밖의 정차 보행자는 종류별 1초 임계값을 적용합니다");
  const auto pedestrian_input = planning::makeBehaviorRouteAvoidanceObjects(
      reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
      {{0.0, -3.0}, {80.0, -3.0}}, {pedestrian}, perception);
  check(pedestrian_input.size() == 1 &&
            std::abs(pedestrian_input.front().margin.lateral_hard_margin_m -
                     0.5) < 1.0e-9 &&
            std::abs(pedestrian_input.front().envelope_buffer_m -
                     0.5) < 1.0e-9,
        "보행자 외피와 경계 여유를 차량과 다른 원본 설정으로 계산합니다");
  auto adjacent_vehicle = observed;
  adjacent_vehicle.y = 1.5;
  adjacent_vehicle.on_current_road_lane = true;
  adjacent_vehicle.has_side_road_lane = true;
  check(planning::makeBehaviorRouteAvoidanceObjects(
            reference.points, ego, {{0.0, 3.0}, {80.0, 3.0}},
            {{0.0, -3.0}, {80.0, -3.0}},
            {adjacent_vehicle}, perception).empty(),
        "객체 쪽에 인접 주행 차로가 있으면 정적 회피 대상으로 삼지 않습니다");

  planning::BehaviorRouteActionInput lane_change;
  lane_change.lane_change = planning::BehaviorRouteLaneChangeRequest{};
  auto &change = *lane_change.lane_change;
  change.target_lanelet_ids = {18};
  change.prepare_length_m = 20.0;
  change.shift_length_m = 30.0;
  change.execution.activated = true;
  for (int index = 20; index <= 80; ++index) {
    auto point = reference.points[index];
    point.y = 3.0;
    change.target_reference.push_back(point);
    change.target_reference_lane_ids.push_back({18});
  }
  ego = reference.points.front();
  const std::vector<std::vector<planning::BehaviorPathPoint2d>>
      change_lanes{{{-10.0, -5.0}, {90.0, -5.0},
                    {90.0, 8.0}, {-10.0, 8.0}}};
  const auto changed = planning::composeBehaviorRouteAction(
      reference, ego, change_lanes, lane_change);
  check(changed.kind == planning::BehaviorRouteActionKind::LaneChange &&
            changed.lane_change_candidate_count == 1 &&
            changed.path.points.size() == changed.path.lane_ids.size() &&
            changed.path.lane_ids[25] ==
                std::vector<std::int64_t>({17, 18}) &&
            changed.path.lane_ids.back() ==
                std::vector<std::int64_t>{18} &&
            changed.path.points.back().y > 2.5,
        "준비 경로와 목표 차로 기준선에서 승인된 변경 경로를 만듭니다");
  change.execution.activated = false;
  const auto waiting_change = planning::composeBehaviorRouteAction(
      reference, ego, change_lanes, lane_change);
  check(waiting_change.kind == planning::BehaviorRouteActionKind::LaneFollow &&
            waiting_change.lane_change_candidate_count == 1 &&
            waiting_change.path.lane_ids.back() ==
                std::vector<std::int64_t>{17},
        "차로 변경 승인 전에는 현재 차로 경로를 유지합니다");
  change.execution.activated = true;
  change.sampling.enabled = true;
  change.shift_length_m = 50.0;
  change.target_reference.clear();
  change.target_reference_lane_ids.clear();
  for (int index = 0; index <= 80; ++index) {
    auto point = reference.points[index];
    point.y = 3.0;
    change.target_reference.push_back(point);
    change.target_reference_lane_ids.push_back({18});
  }
  const auto sampled_change = planning::composeBehaviorRouteAction(
      reference, ego, change_lanes, lane_change);
  check(sampled_change.kind ==
            planning::BehaviorRouteActionKind::LaneChange &&
            sampled_change.lane_change_candidate_count > 0 &&
            sampled_change.path.lane_ids.back() ==
                std::vector<std::int64_t>{18},
        "시간·가속도 표본에서 첫 안전 차로 변경 후보를 출력합니다");
  planning::BehaviorRouteActionInput competing = lane_change;
  competing.avoidance = avoidance.avoidance;
  const auto prioritized_avoidance = planning::composeBehaviorRouteAction(
      reference, ego, change_lanes, competing);
  check(prioritized_avoidance.kind ==
            planning::BehaviorRouteActionKind::StaticAvoidance &&
            prioritized_avoidance.avoidance_candidate_count == 1,
        "같은 슬롯에서 승인된 정적 회피를 차로 변경보다 먼저 선택합니다");
  competing.avoidance->approved = false;
  const auto prioritized_change = planning::composeBehaviorRouteAction(
      reference, ego, change_lanes, competing);
  check(prioritized_change.kind ==
            planning::BehaviorRouteActionKind::LaneChange &&
            prioritized_change.avoidance_candidate_count == 1,
        "회피가 승인 대기 중이면 실행 가능한 차로 변경을 선택합니다");
  competing.lane_change->regulation.distances.crosswalk_m = 10.0;
  competing.avoidance->approved = true;
  const auto regulated_fallback = planning::composeBehaviorRouteAction(
      reference, ego, change_lanes, competing);
  check(regulated_fallback.kind ==
            planning::BehaviorRouteActionKind::StaticAvoidance,
        "규제 요소가 차로 변경을 막아도 다른 행동 후보를 평가합니다");
  lane_change.lane_change->regulation.distances.traffic_light_m = 10.0;
  lane_change.lane_change->regulation.stopped_time_s = 4.0;
  const auto light_limited = planning::composeBehaviorRouteAction(
      reference, ego, change_lanes, lane_change);
  check(light_limited.lane_change_candidate_count == 0 &&
            light_limited.kind ==
                planning::BehaviorRouteActionKind::LaneFollow,
        "장시간 정차해도 신호등 정지선 제한은 유지합니다");

  planning::BehaviorRouteActionInput pull_over;
  pull_over.pull_over = planning::BehaviorRoutePullOverRequest{};
  pull_over.pull_over->goal = {100.0, -2.0, 0.0};
  pull_over.pull_over->goal_lanelet_ids = {19};
  const auto bezier_parking = planning::generateBezierPullOverCandidates(
      reference.points, ego, pull_over.pull_over->goal,
      pull_over.pull_over->parameters);
  check(bezier_parking.size() == 48 &&
            std::abs(bezier_parking.front().path.back().x - 100.0) < 1.0e-9 &&
            std::abs(bezier_parking.front().path.back().y + 2.0) < 1.0e-9 &&
            bezier_parking.front().path.back().longitudinal_velocity_mps == 0.0F,
        "원본 계수 격자의 48개 베지어 주차 후보와 목표 정지를 만듭니다");
  const auto parking_waiting = planning::composeBehaviorRouteAction(
      reference, ego, lanes, pull_over);
  check(parking_waiting.pull_over_candidate_count == 1 &&
            parking_waiting.kind ==
                planning::BehaviorRouteActionKind::LaneFollow,
        "주차 횡이동 후보를 계산하고 승인 전 차로 경로를 유지합니다");
  pull_over.pull_over->approved = true;
  const auto parking = planning::composeBehaviorRouteAction(
      reference, ego, lanes, pull_over);
  check(parking.kind == planning::BehaviorRouteActionKind::PullOver &&
            parking.path.points.size() == parking.path.lane_ids.size() &&
            parking.path.lane_ids[5] ==
                std::vector<std::int64_t>{17} &&
            std::abs(parking.path.points.back().x - 100.0) < 1.0e-6 &&
            std::abs(parking.path.points.back().y + 2.0) < 1.0e-6 &&
            parking.path.points.back().longitudinal_velocity_mps == 0.0F &&
            parking.path.lane_ids.back() ==
                std::vector<std::int64_t>{19},
        "승인된 주차 경로를 목표 영속도와 차로 ID까지 출력합니다");
  auto bezier_request = pull_over;
  bezier_request.pull_over->use_bezier = true;
  const auto selected_bezier = planning::composeBehaviorRouteAction(
      reference, ego, lanes, bezier_request);
  check(selected_bezier.kind ==
            planning::BehaviorRouteActionKind::PullOver &&
            selected_bezier.pull_over_candidate_count > 0 &&
            selected_bezier.path.lane_ids.back() ==
                std::vector<std::int64_t>{19},
        "베지어 주차 방식을 선택해 차로 포함 후보를 행동 경로로 출력합니다");
  const auto projected_parking = planning::generateShiftPullOverCandidate(
      reference.points, ego, {70.3, -2.0, 0.0});
  check(projected_parking &&
            std::abs(projected_parking->path[
                projected_parking->shift_end_index].x - 65.3) < 1.0e-6,
        "목표 앞 횡이동 종료점을 경로 선분에 정확히 투영합니다");
  const auto extended_parking = planning::generateShiftPullOverCandidate(
      reference.points, ego, {100.0, -2.0, 0.0});
  check(extended_parking &&
            std::abs(extended_parking->path[
                extended_parking->shift_end_index].x - 95.0) < 1.0e-6 &&
            parking.path.lane_ids[
                extended_parking->shift_start_index] ==
                std::vector<std::int64_t>{17} &&
            parking.path.lane_ids[
                extended_parking->shift_end_index] ==
                std::vector<std::int64_t>{19},
        "기준 경로 끝 밖의 목표는 직전 방향으로 경로를 연장합니다");
  auto stopped_reference = reference.points;
  stopped_reference.back().longitudinal_velocity_mps = 0.0F;
  const auto extended_from_stop = planning::generateShiftPullOverCandidate(
      stopped_reference, ego, {100.0, -2.0, 0.0});
  check(extended_from_stop &&
            extended_from_stop->path[80]
                    .longitudinal_velocity_mps > 0.0F &&
            extended_from_stop->path.back()
                    .longitudinal_velocity_mps == 0.0F,
        "경로 연장 시 중간 영속도를 제거하고 최종 목표에서 정지합니다");
  const auto curved_parking = planning::generateShiftPullOverCandidate(
      reference.points, ego, {70.3, -2.0, 0.2});
  check(curved_parking && curved_parking->path.size() >
          curved_parking->shift_end_index + 2 &&
            std::abs(curved_parking->path[
                curved_parking->shift_end_index + 1].orientation_z -
                     std::sin(0.1)) > 1.0e-4,
        "목표 앞 보간 곡선의 진행 방향에서 중간 자세를 계산합니다");
  planning::BehaviorRouteActionInput sampled_parking;
  sampled_parking.pull_over = planning::BehaviorRoutePullOverRequest{};
  sampled_parking.pull_over->goal = {70.3, -2.0, 0.0};
  sampled_parking.pull_over->approved = true;
  sampled_parking.pull_over->parameters.minimum_lateral_jerk_mps3 = 0.2;
  sampled_parking.pull_over->parameters.maximum_lateral_jerk_mps3 = 2.0;
  sampled_parking.pull_over->parameters.jerk_samples = 1;
  sampled_parking.pull_over->vehicle = {0.01, 0.01, 0.001, 0.5};
  const std::vector<std::vector<planning::BehaviorPathPoint2d>>
      delayed_shoulder{{{-10.0, -0.01}, {58.0, -0.01},
                        {58.0, -5.0}, {120.0, -5.0},
                        {120.0, 5.0}, {-10.0, 5.0}}};
  auto first_jerk_parameters = sampled_parking.pull_over->parameters;
  first_jerk_parameters.maximum_lateral_jerk_mps3 =
      first_jerk_parameters.minimum_lateral_jerk_mps3;
  const auto first_jerk_path = planning::generateShiftPullOverCandidate(
      reference.points, ego, sampled_parking.pull_over->goal,
      first_jerk_parameters);
  check(first_jerk_path && planning::firstBehaviorPathLaneDeparture(
            first_jerk_path->path, delayed_shoulder,
            sampled_parking.pull_over->vehicle),
        "첫 주차 저크 표본은 좁은 차로 구간을 벗어납니다");
  const auto later_jerk = planning::composeBehaviorRouteAction(
      reference, ego, delayed_shoulder, sampled_parking);
  check(later_jerk.kind == planning::BehaviorRouteActionKind::PullOver &&
            later_jerk.pull_over_candidate_count == 2,
        "첫 저크 경로가 차로 밖이면 다음 저크 후보를 검사합니다");
  return EXIT_SUCCESS;
}
