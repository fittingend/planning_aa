#include "planning/behavior_path_lane_safety.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace {

void check(bool value, const char *message) {
  if (!value) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

}  // 익명 네임스페이스

int main() {
  const std::vector<std::vector<planning::BehaviorPathPoint2d>> lanes{
      {{0.0, -2.0}, {20.0, -2.0}, {20.0, 0.0}, {0.0, 0.0}},
      {{0.0, 0.0}, {20.0, 0.0}, {20.0, 2.0}, {0.0, 2.0}}};
  planning::BehaviorPathVehicleFootprint vehicle;
  vehicle.front_m = 2.5;
  vehicle.back_m = 1.0;
  vehicle.half_width_m = 0.8;
  planning::Trajectory path(2);
  path[0].x = 2.0;
  path[1].x = 17.0;
  check(!planning::firstBehaviorPathLaneDeparture(path, lanes, vehicle),
        "이어진 두 차로의 공통 경계 위 직선 주행을 허용합니다");
  auto outside = path;
  outside[1].y = 1.3;
  const auto edge = planning::firstBehaviorPathLaneDeparture(
      outside, lanes, vehicle);
  check(edge && edge->segment_index == 0 &&
            edge->segment_fraction > 0.0 && edge->segment_fraction < 1.0,
        "경로 끝에 닿기 전에 차량 측면이 차로 밖으로 나가는 지점을 찾습니다");
  check(planning::behaviorPathPointsInsideLanes(outside, lanes),
        "차로 변경의 경로점 유효성은 차량 외곽 검사와 구분합니다");
  outside[1].y = 3.0;
  check(!planning::behaviorPathPointsInsideLanes(outside, lanes),
        "차로 변경 경로점이 차로를 벗어나면 거절합니다");
  auto sparse_turn = path;
  sparse_turn[1].orientation_z = 1.0;
  sparse_turn[1].orientation_w = 0.0;
  const auto middle = planning::firstBehaviorPathLaneDeparture(
      sparse_turn, lanes, vehicle);
  check(middle && middle->segment_index == 0 &&
            middle->segment_fraction > 0.0 &&
            middle->segment_fraction < 1.0,
        "양 끝 자세가 안전해도 희소 경로 중간의 회전 이탈을 찾습니다");
  planning::StartPullOutCandidate safe;
  safe.path = path;
  planning::StartPullOutCandidate unsafe;
  unsafe.path = outside;
  const auto selected = planning::selectLaneSafeStartPullOutCandidates(
      {safe, unsafe}, lanes, vehicle);
  check(selected.size() == 1 && selected.front().path.back().y == 0.0,
        "출발 후보 목록에서 차로 이탈 경로를 제거합니다");
  const std::vector<std::vector<planning::BehaviorPathPoint2d>> notch_lane{
      {{0.0, -2.0}, {20.0, -2.0}, {20.0, 2.0},
       {10.4, 2.0}, {10.4, 0.2}, {10.2, 0.2},
       {10.2, 2.0}, {0.0, 2.0}}};
  planning::Trajectory notch_path(1);
  notch_path.front().x = 10.0;
  check(planning::firstBehaviorPathLaneDeparture(
            notch_path, notch_lane, vehicle).has_value(),
        "차체 표본점 사이의 좁은 차로 침범도 외곽 전체로 찾습니다");
  planning::StartPullOutSearchInput start_search;
  start_search.start_lane_arc_m = 5.0;
  start_search.lane_length_m = 10.0;
  start_search.base_link_to_rear_m = 1.0;
  start_search.maximum_backward_distance_m = 6.0;
  start_search.front_collision_margin_m = 0.2;
  start_search.backward_collision_margin_m = 0.2;
  start_search.vehicle.front_m = 0.2;
  start_search.vehicle.rear_m = 0.2;
  start_search.vehicle.half_width_m = 0.2;
  start_search.samples = {
      {{0.0, 0.0, 0.0, 0.0, 0.0}, 0.0, 5.0},
      {{0.0, -2.0, 0.0, 0.0, 0.0}, 2.0, 3.0},
      {{0.0, -4.0, 0.0, 0.0, 0.0}, 4.0, 1.0},
      {{0.0, -6.0, 0.0, 0.0, 0.0}, 6.0, 0.0}};
  planning::BehaviorSafetyObject small_object;
  small_object.length_m = 0.2;
  small_object.width_m = 0.2;
  const auto front_object = planning::behaviorSafetyWorldPolygon(
      {0.0, 0.0, 0.0, 0.0, 0.0}, small_object);
  const auto rear_object = planning::behaviorSafetyWorldPolygon(
      {0.0, -4.0, 0.0, 0.0, 0.0}, small_object);
  start_search.front_stop_objects = {front_object};
  start_search.all_stop_objects = {front_object, rear_object};
  const auto searched = planning::searchStartPullOutPoseCandidates(
      start_search);
  check(searched.size() == 1 && searched[0].x == -2.0,
        "앞쪽 객체는 표본을 건너뛰고 뒤쪽 객체에서는 검색을 중단합니다");
  start_search.front_stop_objects.clear();
  start_search.all_stop_objects.clear();
  start_search.ignore_distance_from_lane_end_m = 6.0;
  const auto lane_end_filtered =
      planning::searchStartPullOutPoseCandidates(start_search);
  check(lane_end_filtered.size() == 2 &&
            lane_end_filtered[0].x == -2.0 &&
            lane_end_filtered[1].x == -4.0,
        "차로 끝 여유와 차량 뒤 길이로 허용 후진 거리를 제한합니다");
  const auto backward_samples = planning::sampleStartPullOutBackwardPoses(
      {{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}},
      {0.0, 15.0, 1.0, 0.0, 0.0},
      {0.0, 2.0, 5.0, 20.0});
  check(backward_samples.size() == 4 &&
            backward_samples[0] && backward_samples[1] &&
            backward_samples[2] && !backward_samples[3] &&
            std::abs(backward_samples[0]->x - 15.0) < 1.0e-9 &&
            std::abs(backward_samples[1]->x - 13.0) < 1.0e-9 &&
            std::abs(backward_samples[2]->x - 10.0) < 1.0e-9 &&
            std::abs(backward_samples[2]->y - 1.0) < 1.0e-9,
        "측방 이동 중심선에서 지정한 후진 거리 자세를 보간합니다");
  planning::StartPullOutStopObject front_stop;
  front_stop.pose = {0.0, 12.0, 0.0, 0.0, 0.0};
  front_stop.shape.length_m = 1.0;
  front_stop.shape.width_m = 1.0;
  auto rear_stop = front_stop;
  rear_stop.pose.x = 8.0;
  auto outside_stop = front_stop;
  outside_stop.pose.y = 5.0;
  auto moving = front_stop;
  moving.pose.x = 14.0;
  moving.velocity_x_mps = 1.0;
  auto disallowed = front_stop;
  disallowed.pose.x = 13.0;
  disallowed.allowed_class = false;
  auto reverse = front_stop;
  reverse.pose.x = 15.0;
  reverse.pose.yaw_rad = std::acos(-1.0);
  auto forward_boundary = front_stop;
  forward_boundary.pose.x = 15.0;
  auto backward_boundary = front_stop;
  backward_boundary.pose.x = 5.0;
  const std::vector<planning::StartPullOutStopObject> stop_objects{
      front_stop, rear_stop, outside_stop, moving, disallowed, reverse,
      forward_boundary, backward_boundary};
  const std::vector<std::vector<planning::BehaviorPathPoint2d>> stop_lanes{
      {{0.0, -2.0}, {20.0, -2.0}, {20.0, 2.0}, {0.0, 2.0}}};
  const std::vector<planning::BehaviorPathPoint2d> stop_path{
      {0.0, 0.0}, {20.0, 0.0}};
  const auto all_stop = planning::filterStartPullOutStopObjects(
      stop_objects, stop_lanes, stop_path, 10.0, 0.0, 1.0, 5.0, 5.0);
  const auto front_only = planning::filterStartPullOutStopObjects(
      stop_objects, stop_lanes, stop_path, 10.0, 0.0, 1.0, 5.0, 0.0);
  const auto backward_only = planning::filterStartPullOutStopObjects(
      stop_objects, stop_lanes, stop_path, 10.0, 0.0, 1.0, 0.0, 5.0);
  check(all_stop.size() == 2 && front_only.size() == 1 &&
            backward_only.size() == 1,
        "정지 객체의 속도·차로 외형·전후방 호길이·종류를 순서대로 고릅니다");
  auto edge_overlap = front_stop;
  edge_overlap.pose.y = 2.3;
  check(planning::filterStartPullOutStopObjects(
            {edge_overlap}, stop_lanes, stop_path, 10.0, 0.0,
            1.0, 5.0, 5.0).size() == 1,
        "객체 중심은 차로 밖이어도 외형이 차로에 닿으면 선택합니다");
  planning::StartPullOutCandidatePlanInput plan_input;
  plan_input.centerline = {{0.0, 0.0}, {30.0, 0.0}};
  plan_input.lane_polygons = {
      {{0.0, -2.0}, {30.0, -2.0}, {30.0, 2.0}, {0.0, 2.0}}};
  plan_input.original_start_pose = {0.0, 10.0, 0.0, 0.0, 0.0};
  plan_input.start_lane_arc_m = 10.0;
  plan_input.lane_length_m = 30.0;
  plan_input.base_link_to_rear_m = 1.0;
  plan_input.maximum_backward_distance_m = 6.0;
  plan_input.backward_search_resolution_m = 2.0;
  plan_input.first_collision_margin_m = 0.0;
  plan_input.second_collision_margin_m = 0.1;
  auto blocking_rear = front_stop;
  blocking_rear.pose.x = 3.0;
  plan_input.objects = {blocking_rear};
  const auto planned = planning::planStartPullOutPoseCandidates(plan_input);
  check(planned.size() == 3 && planned[0].x == 10.0 &&
            planned[1].x == 8.0 && planned[2].x == 6.0,
        "지도 표본과 정지 객체를 이어 뒤쪽 충돌에서 출발 후보 검색을 끝냅니다");
  return EXIT_SUCCESS;
}
