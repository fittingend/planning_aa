#include "planning/planning_pipeline_map_adapter.hpp"

#include <lanelet2_io/io_handlers/Serialize.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <boost/archive/binary_oarchive.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <vector>

namespace {
void check(bool value, const char *message) {
  if (!value) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

std::vector<std::uint8_t> mapBytes(bool with_crosswalk = false,
                                   bool with_light = false,
                                   bool with_intersection = false,
                                   bool with_continuation = false,
                                   bool with_third_lane = false,
                                   bool with_opposite_lane = false,
                                   bool with_elevation = false,
                                   double terminal_extension_m = 0.0,
                                   double continuation_shift_m = 0.0) {
  lanelet::LaneletMap map;
  const auto line = [with_continuation, continuation_shift_m](lanelet::Id id, double y) {
    return lanelet::LineString3d(
        id, {lanelet::Point3d(id + 1, -10.0, y, 0.0),
             lanelet::Point3d(id + 2,
                              with_continuation ? 80.0 + continuation_shift_m : 160.0,
                              y, 0.0)});
  };
  auto right = line(100, -3.0);
  auto shared = line(200, 3.0);
  auto left = line(300, 9.0);
  lanelet::Lanelet current(10, shared, right);
  lanelet::Lanelet target(11, left, shared);
  current.setAttribute("subtype", "road");
  target.setAttribute("subtype", "road");
  const auto centerline = [with_continuation, with_elevation,
                           terminal_extension_m, continuation_shift_m](
      lanelet::Id id, double y) {
    std::vector<lanelet::Point3d> points;
    for (int index = 0;
         index <= (with_continuation ? 80 : 150); ++index)
      points.emplace_back(id + index + 1, index, y,
                          with_elevation ? 0.1 * index : 0.0);
    if (with_continuation && continuation_shift_m > 0.0)
      points.emplace_back(id + 100000, 80.0 + continuation_shift_m, y,
                          with_elevation ? 0.1 *
                              (80.0 + continuation_shift_m) : 0.0);
    if (terminal_extension_m > 0.0) {
      const double terminal = with_continuation ? 80.0 : 150.0;
      points.emplace_back(id + 1000, terminal + terminal_extension_m, y,
                          with_elevation ? 0.1 *
                              (terminal + terminal_extension_m) : 0.0);
    }
    return lanelet::LineString3d(id, points);
  };
  current.setCenterline(centerline(1000, 0.0));
  target.setCenterline(centerline(2000, 6.0));
  map.add(current);
  map.add(target);
  if (with_third_lane) {
    auto outer = line(400, 15.0);
    lanelet::Lanelet third(12, outer, left);
    third.setAttribute("subtype", "road");
    third.setCenterline(centerline(9000, 12.0));
    map.add(third);
  }
  if (with_opposite_lane) {
    auto outer = line(9500, -9.0);
    lanelet::Lanelet opposite(13, outer.invert(), right.invert());
    opposite.setAttribute("subtype", "road");
    map.add(opposite);
  }
  if (with_continuation) {
    const auto following_bound = [](lanelet::Id id,
                                    const lanelet::LineString3d &previous,
                                    double y) {
      return lanelet::LineString3d(
          id, {lanelet::Point3d(previous.back().id(),
                               previous.back().x(),
                               previous.back().y(),
                               previous.back().z()),
               lanelet::Point3d(id + 1, 160.0, y, 0.0)});
    };
    const auto next_right = following_bound(6000, right, -3.0);
    const auto next_shared = following_bound(6010, shared, 3.0);
    const auto next_left = following_bound(6020, left, 9.0);
    lanelet::Lanelet current_next(20, next_shared, next_right);
    lanelet::Lanelet target_next(21, next_left, next_shared);
    current_next.setAttribute("subtype", "road");
    target_next.setAttribute("subtype", "road");
    const auto next_centerline = [continuation_shift_m](lanelet::Id id, double y) {
      std::vector<lanelet::Point3d> points;
      points.emplace_back(id + 100000, 80.0 + continuation_shift_m, y, 0.0);
      for (int x = 81; x <= 150; ++x)
        points.emplace_back(id + x + 1, x, y, 0.0);
      return lanelet::LineString3d(id, points);
    };
    current_next.setCenterline(next_centerline(7000, 0.0));
    target_next.setCenterline(next_centerline(8000, 6.0));
    map.add(current_next);
    map.add(target_next);
  }
  if (with_crosswalk) {
    lanelet::LineString3d crossing_left(
        3000, {lanelet::Point3d(3001, 40.0, 10.0, 0.0),
               lanelet::Point3d(3002, 44.0, 10.0, 0.0)});
    lanelet::LineString3d crossing_right(
        3010, {lanelet::Point3d(3011, 40.0, -5.0, 0.0),
               lanelet::Point3d(3012, 44.0, -5.0, 0.0)});
    lanelet::Lanelet crossing(12, crossing_left, crossing_right);
    crossing.setAttribute("subtype", "crosswalk");
    map.add(crossing);
  }
  if (with_light) {
    lanelet::LineString3d stop_line(
        4000, {lanelet::Point3d(4001, 40.0, -3.0, 0.0),
               lanelet::Point3d(4002, 40.0, 3.0, 0.0)});
    lanelet::LineString3d signal(
        4010, {lanelet::Point3d(4011, 42.0, 3.0, 0.0),
               lanelet::Point3d(4012, 42.0, 5.0, 0.0)});
    auto light = lanelet::TrafficLight::make(
        4020, {}, {signal}, stop_line);
    map.add(light);
    map.laneletLayer.get(10).addRegulatoryElement(light);
  }
  if (with_intersection) {
    lanelet::LineString3d left_edge(
        5000, {lanelet::Point3d(5001, 40.0, 3.0, 0.0),
               lanelet::Point3d(5002, 44.0, 3.0, 0.0)});
    lanelet::LineString3d right_edge(
        5010, {lanelet::Point3d(5011, 40.0, -3.0, 0.0),
               lanelet::Point3d(5012, 44.0, -3.0, 0.0)});
    lanelet::Lanelet intersection(13, left_edge, right_edge);
    intersection.setAttribute("subtype", "road");
    intersection.setAttribute("turn_direction", "straight");
    map.add(intersection);
  }
  std::ostringstream stream(std::ios::out | std::ios::binary);
  lanelet::Id next_id = continuation_shift_m > 0.0 ? 110000 : 10000;
  {
    boost::archive::binary_oarchive archive(stream);
    archive << map;
    archive << next_id;
  }
  const auto bytes = stream.str();
  return {bytes.begin(), bytes.end()};
}
}  // 익명 네임스페이스

int main() {
  planning::PlanningPipelineMapAdapter adapter(mapBytes(), {});
  planning::MissionRoute route;
  route.sections.push_back({10, {10}});
  planning::PlanningPipelineFrame frame;
  frame.timestamp_s = 1.0;
  frame.ego.x = 5.0;
  frame.ego_speed_mps = 5.0;
  {
    planning::PlanningPipelineMapAdapter elevated_map(
        mapBytes(false, false, false, false, false, false, true), {});
    const auto elevated = elevated_map.onRouteFrame(frame, route, {});
    const auto &reference = elevated.route_reference_path.points;
    const auto near_50 = std::min_element(
        reference.begin(), reference.end(), [](const auto &a, const auto &b) {
          return std::abs(a.x - 50.0) < std::abs(b.x - 50.0);
        });
    check(near_50 != reference.end() &&
              std::abs(near_50->z - 0.1 * near_50->x) < 1.0e-3,
          "경사 지도에서 기준 경로는 원본 Lanelet2 중심선 높이를 유지합니다");
  }
  {
    planning::PlanningPipelineMapAdapter endpoint_map(
        mapBytes(false, false, false, false, false, false, false, 0.05), {});
    const auto endpoint = endpoint_map.onRouteFrame(frame, route, {});
    const auto &reference = endpoint.route_reference_path.points;
    check(!reference.empty() &&
              std::abs(reference.back().x - 150.05) < 1.0e-4 &&
              std::none_of(reference.begin(), reference.end(),
                           [](const auto &point) {
                             return std::abs(point.x - 150.0) < 1.0e-4;
                           }),
          "원본 재표본화는 0.1m 안의 끝점을 추가하지 않고 격자점을 교체합니다");
  }
  {
    planning::PlanningPipelineMapAdapter boundary_map(
        mapBytes(false, false, false, true, false, false, false, 0.0, 0.05), {});
    planning::MissionRoute continuation;
    continuation.sections.push_back({10, {10}});
    continuation.sections.push_back({20, {20}});
    const auto boundary = boundary_map.onRouteFrame(frame, continuation, {});
    const auto &reference = boundary.route_reference_path.points;
    check(std::any_of(reference.begin(), reference.end(),
                      [](const auto &point) {
                        return std::abs(point.x - 80.05) < 1.0e-4;
                      }) &&
              std::none_of(reference.begin(), reference.end(),
                           [](const auto &point) {
                             return std::abs(point.x - 80.0) < 1.0e-4;
                           }),
          "원본 재표본화는 0.1m 안의 차로 경계점을 격자점 대신 유지합니다");
  }
  {
    planning::PlanningPipelineMapAdapter nominal_map(mapBytes(), {});
    planning::LaneletBinAdapter lane_map(mapBytes());
    const auto nominal = nominal_map.onRouteFrame(frame, route, {});
    const auto &path = nominal.route_behavior_path.points;
    check(nominal.pipeline.status ==
              planning::PlanningPipelineStatus::Complete &&
              path.size() > 60 &&
              std::hypot(path[1].x - path[0].x,
                         path[1].y - path[0].y) > 1.9 &&
              std::hypot(path[1].x - path[0].x,
                         path[1].y - path[0].y) < 2.1 &&
              std::abs(path.front().longitudinal_velocity_mps -
                       lane_map.laneSpeedLimit(10)) < 1.0e-5,
          "일반 route 경로는 2m로 재표본화하고 지도 제한속도를 사용합니다");
  }
  {
    planning::MissionRoute extended_route;
    extended_route.sections = {{10, {10}}, {20, {20}}};
    planning::PlanningPipelineMapAdapter sequence_map(
        mapBytes(false, false, false, true), {}, {}, {},
        planning::InvalidTrajectoryPolicy::UsePreviousResult,
        {}, {}, {}, true);
    frame.ego.x = 85.0;
    const auto near_boundary = sequence_map.onRouteFrame(
        frame, extended_route, {});
    const auto &first_path = near_boundary.route_behavior_path.points;
    check(near_boundary.pipeline.status ==
              planning::PlanningPipelineStatus::Complete &&
              !first_path.empty() && first_path.front().x < 80.0,
          "차로 초입에서는 이전 route 차로의 중심선을 포함합니다");
    frame.timestamp_s += 0.1;
    frame.ego.x = 100.0;
    const auto in_next_lane = sequence_map.onRouteFrame(
        frame, extended_route, {});
    const auto &next_path = in_next_lane.route_behavior_path.points;
    check(in_next_lane.pipeline.status ==
              planning::PlanningPipelineStatus::Complete &&
              !next_path.empty() && next_path.front().x > 90.0,
          "차로 안쪽에서는 자차 기준으로 원시 중심선을 잘라 재표본화합니다");
    planning::PlanningPipelineMapAdapter source_crop_map(
        mapBytes(false, false, false, true), {}, {}, {},
        planning::InvalidTrajectoryPolicy::UsePreviousResult,
        {}, {}, {}, true, 7.0);
    frame.ego.x = 25.0;
    const auto source_crop = source_crop_map.onRouteFrame(
        frame, extended_route, {});
    check(source_crop.pipeline.status ==
              planning::PlanningPipelineStatus::Complete &&
              !source_crop.route_behavior_path.points.empty() &&
              std::abs(source_crop.route_behavior_path.points.front().x -
                       16.0) < 1.0e-6,
          "원본 후방 절단은 경계를 넘는 선분의 시작 2m 표본에서 시작합니다");
    planning::MissionRoute branched_route;
    branched_route.sections = {{10, {10}}, {21, {20, 21}}};
    frame.timestamp_s += 0.1;
    frame.ego.x = 75.0;
    const auto connected = sequence_map.onRouteFrame(
        frame, branched_route, {});
    check(connected.pipeline.status ==
              planning::PlanningPipelineStatus::Complete &&
              std::any_of(connected.route_behavior_path.lane_ids.begin(),
                          connected.route_behavior_path.lane_ids.end(),
                          [](const auto &ids) {
                            return std::find(ids.begin(), ids.end(), 20) !=
                                   ids.end();
                          }),
          "route의 선호 차로가 직접 연결되지 않으면 연결된 차로를 따릅니다");
    planning::PlanningPipelineMapAdapter default_sequence_map(
        mapBytes(false, false, false, true), {});
    const auto default_connected = default_sequence_map.onRouteFrame(
        frame, branched_route, {});
    check(default_connected.pipeline.status ==
              planning::PlanningPipelineStatus::Complete &&
              std::any_of(default_connected.route_behavior_path.lane_ids.begin(),
                          default_connected.route_behavior_path.lane_ids.end(),
                          [](const auto &ids) {
                            return std::find(ids.begin(), ids.end(), 20) !=
                                   ids.end();
                          }),
          "기본 route도 선호 차로 대신 실제로 이어지는 primitive를 따릅니다");
    planning::MissionRoute disconnected_route;
    disconnected_route.sections = {{10, {10}}, {21, {21}}};
    const auto local_only = default_sequence_map.onRouteFrame(
        frame, disconnected_route, {});
    check(local_only.pipeline.status ==
              planning::PlanningPipelineStatus::Complete &&
              !local_only.route_behavior_path.points.empty() &&
              std::all_of(local_only.route_behavior_path.lane_ids.begin(),
                          local_only.route_behavior_path.lane_ids.end(),
                          [](const auto &ids) {
                            return std::find(ids.begin(), ids.end(), 21) ==
                                   ids.end();
                          }),
          "후속 route 차로가 없으면 현재까지의 지역 경로를 유지합니다");
    const planning::BehaviorGoalPose distant_goal{140.0, 6.0, 0.0};
    const auto local_with_goal = default_sequence_map.onRouteFrame(
        frame, disconnected_route, {}, {}, {}, distant_goal);
    check(local_with_goal.pipeline.status ==
              planning::PlanningPipelineStatus::Complete &&
              !local_with_goal.route_behavior_path.points.empty() &&
              local_with_goal.route_behavior_path.points.back()
                      .longitudinal_velocity_mps == 0.0F &&
              std::all_of(local_with_goal.route_behavior_path.lane_ids.begin(),
                          local_with_goal.route_behavior_path.lane_ids.end(),
                          [](const auto &ids) {
                            return std::find(ids.begin(), ids.end(), 21) ==
                                   ids.end();
                          }),
          "지역 경로가 목표 차로에 못 닿아도 끝점 정지 규칙을 적용합니다");
    frame.ego.x = 5.0;
  }
  {
    planning::PlanningPipelineMapAdapter shifted_map(mapBytes(), {});
    planning::BehaviorRouteActionInput side_shift;
    side_shift.side_shift = planning::BehaviorRouteSideShiftRequest{};
    side_shift.side_shift->requested_lateral_offset_m = 1.0;
    side_shift.side_shift->approved = true;
    const auto shifted = shifted_map.onRouteFrame(
        frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, side_shift);
    check(shifted.behavior_action &&
              shifted.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::SideShift &&
              std::abs(shifted.behavior_action->path.points.back().y - 1.0) <
                  1.0e-6 &&
              shifted.pipeline.status ==
                  planning::PlanningPipelineStatus::Complete,
          "지도 경로에서 승인된 사이드 시프트를 종단 계산합니다");
    auto rapid_frame = frame;
    rapid_frame.timestamp_s += 0.5;
    rapid_frame.ego = shifted.behavior_action->path.points[60];
    rapid_frame.ego_speed_mps = 5.0;
    auto rapid_request = side_shift;
    rapid_request.side_shift->requested_lateral_offset_m = 0.0;
    const auto throttled = shifted_map.onRouteFrame(
        rapid_frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, rapid_request);
    check(throttled.behavior_action &&
              throttled.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::SideShift &&
              std::abs(throttled.behavior_action->path.points.back().y - 1.0) <
                  1.0e-6,
          "사이드 시프트 변경 요청은 원본의 1초 간격을 지킵니다");
    auto early_frame = frame;
    early_frame.timestamp_s += 1.0;
    early_frame.ego = shifted.behavior_action->path.points[20];
    early_frame.ego_speed_mps = 5.0;
    auto early_return_request = side_shift;
    early_return_request.side_shift->requested_lateral_offset_m = 0.0;
    const auto deferred = shifted_map.onRouteFrame(
        early_frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, early_return_request);
    check(deferred.behavior_action &&
              deferred.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::SideShift &&
              std::abs(deferred.behavior_action->path.points.back().y - 1.0) <
                  1.0e-6,
          "횡이동 중 새 사이드 시프트 요청은 현재 이동 구간 뒤로 미룹니다");
    auto shifted_frame = frame;
    shifted_frame.timestamp_s += 5.0;
    shifted_frame.ego = shifted.behavior_action->path.points[60];
    shifted_frame.ego_speed_mps = 5.0;
    const auto continued = shifted_map.onRouteFrame(
        shifted_frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, side_shift);
    check(continued.behavior_action &&
              continued.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::SideShift &&
              std::abs(continued.behavior_action->path.points.back().y -
                       1.0) < 1.0e-6,
          "같은 사이드 시프트 요청은 다음 콜백에서도 승인 경로를 유지합니다");
    side_shift.side_shift->requested_lateral_offset_m = 0.0;
    shifted_frame.timestamp_s += 1.0;
    const auto returned = shifted_map.onRouteFrame(
        shifted_frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, side_shift);
    check(returned.behavior_action &&
              returned.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::SideShift &&
              std::abs(returned.behavior_action->path.points.front().y -
                       1.0) < 1.0e-6 &&
              std::abs(returned.behavior_action->path.points.back().y) <
                  1.0e-6,
          "새 사이드 시프트 요청은 현재 이동량에서 중심선으로 복귀합니다");
    shifted_frame.ego = returned.behavior_action->path.points.back();
    shifted_frame.timestamp_s += 5.0;
    const auto finished = shifted_map.onRouteFrame(
        shifted_frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, side_shift);
    check(finished.behavior_action &&
              finished.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneFollow,
          "중심선 복귀 뒤에는 사이드 시프트 상태를 완료합니다");
    planning::PlanningPipelineMapAdapter wide_shift_map(mapBytes(), {});
    auto wide_request = side_shift;
    wide_request.side_shift->requested_lateral_offset_m = 4.0;
    const auto wide_shift = wide_shift_map.onRouteFrame(
        frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, wide_request);
    check(wide_shift.behavior_action &&
              wide_shift.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::SideShift &&
              std::abs(wide_shift.behavior_action->path.points.back().y -
                       4.0) < 1.0e-6 &&
              wide_shift.pipeline.status ==
                  planning::PlanningPipelineStatus::Complete,
          "차로 경계 밖 사이드 시프트에 주행 가능 경계를 확장합니다");
  }
  {
    planning::LaneletBinAdapter adjacent_map(mapBytes());
    check(adjacent_map.pointInRoadLanelet(10, 30.0, 1.5) &&
              adjacent_map.hasAdjacentRoadLanelet(10, true) &&
              adjacent_map.adjacentRoadLaneletId(10, true) == 11 &&
              !adjacent_map.hasAdjacentRoadLanelet(10, false),
          "현재 차로 왼쪽의 같은 방향 도로 차로만 찾아냅니다");
  }
  {
    planning::LaneletBinAdapter opposite_map(
        mapBytes(false, false, false, false, false, true));
    check(opposite_map.hasOppositeRoadLanelet(10, false) &&
              !opposite_map.hasOppositeRoadLanelet(10, true),
          "반대 방향 도로가 공유하는 오른쪽 경계를 판정합니다");
  }
  {
    planning::LaneletBinAdapter regulated_map(
        mapBytes(false, true, true));
    planning::Trajectory reference;
    for (int x = 0; x <= 80; ++x) {
      planning::TrajectoryPoint point;
      point.x = x;
      reference.push_back(point);
    }
    const auto distances = regulated_map.laneChangeRegulatoryDistances(
        reference, {10, 13}, 5.0, 0.0);
    check(distances.traffic_light_m &&
              std::abs(*distances.traffic_light_m - 35.0) < 1.0e-6 &&
              distances.intersection_m &&
              std::abs(*distances.intersection_m - 35.0) < 1.0e-6,
          "이진 지도 정지선과 교차로 진입까지의 경로 거리를 계산합니다");
  }
  {
    planning::PlanningPipelineMapAdapter automatic(mapBytes(), {});
    planning::MissionRoute preferred;
    preferred.sections.push_back({11, {10, 11}});
    const auto awaiting = automatic.onRouteFrame(
        frame, preferred, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0);
    check(awaiting.behavior_action &&
              awaiting.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneFollow,
          "외부 승인 전에는 자동 차로 변경 후보를 실행하지 않습니다");
    const auto result = automatic.onRouteFrame(
        frame, preferred, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, std::nullopt, true);
    check(result.behavior_action &&
              result.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneChange &&
              result.behavior_action->path.lane_ids.back() ==
                  std::vector<std::int64_t>{11},
          "현재 차로와 선호 차로가 인접하면 지도 경로가 변경 요청을 만듭니다");
  }
  {
    planning::PlanningPipelineMapAdapter automatic(
        mapBytes(false, false, false, false, true), {});
    planning::LaneletBinAdapter triple_map(
        mapBytes(false, false, false, false, true));
    check(triple_map.nextRoadLaneletTowardPreferred(
              10, 12, {10, 11, 12}) == 11 &&
              triple_map.nextRoadLaneletTowardPreferred(
                  11, 12, {10, 11, 12}) == 12,
          "두 칸 떨어진 선호 차로까지 한 칸씩 이웃을 고릅니다");
    planning::MissionRoute preferred;
    preferred.sections.push_back({12, {10, 11, 12}});
    const auto first = automatic.onRouteFrame(
        frame, preferred, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, std::nullopt, true);
    check(first.behavior_action &&
              first.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneChange &&
              first.behavior_action->path.lane_ids.back() ==
                  std::vector<std::int64_t>{11},
          "선호 차로가 두 칸 떨어지면 첫 이웃 차로로 변경합니다");
    auto second_frame = frame;
    second_frame.timestamp_s = 2.0;
    second_frame.ego.y = 6.0;
    const auto second = automatic.onRouteFrame(
        second_frame, preferred, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, std::nullopt, true);
    check(second.behavior_action &&
              second.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneChange &&
              second.behavior_action->path.lane_ids.back() ==
                  std::vector<std::int64_t>{12},
          "다음 콜백에서는 선호 차로까지 남은 한 칸을 변경합니다");
  }
  {
    planning::PlanningPipelineMapAdapter automatic(
        mapBytes(false, false, false, true), {});
    planning::MissionRoute ahead;
    ahead.sections = {{10, {10}}, {21, {20, 21}}};
    auto upcoming = frame;
    upcoming.ego.x = 5.0;
    const auto distant = automatic.onRouteFrame(
        upcoming, ahead, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0);
    check(!distant.behavior_action &&
              !distant.route_behavior_path.points.empty(),
          "선호 차로가 먼 다음 구간에 있으면 현재 차로를 유지합니다");
    upcoming.timestamp_s += 0.1;
    upcoming.ego.x = 70.0;
    const auto near = automatic.onRouteFrame(
        upcoming, ahead, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0);
    check(near.behavior_action &&
              !near.route_behavior_path.points.empty(),
          "다음 구간의 선호 차로를 미리 찾아 차로 변경 후보를 만듭니다");
  }
  {
    planning::PlanningPipelineMapAdapter automatic(
        mapBytes(false, false, false, true), {});
    planning::LaneletBinAdapter split_map(
        mapBytes(false, false, false, true));
    check(split_map.followingRoadLaneletId(10, {20, 21}) == 20 &&
              split_map.followingRoadLaneletId(11, {20, 21}) == 21,
          "두 차로의 후속 연결을 이진 지도에서 각각 찾습니다");
    planning::MissionRoute preferred;
    preferred.sections.push_back({11, {10, 11}});
    preferred.sections.push_back({21, {20, 21}});
    const auto result = automatic.onRouteFrame(
        frame, preferred, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, std::nullopt, true);
    check(result.behavior_action &&
              result.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneChange &&
              result.behavior_action->path.lane_ids.back() ==
                  std::vector<std::int64_t>{21},
          "두 경로 구간의 현재·선호 차로 후속 관계를 따라 변경 경로를 만듭니다");
  }
  planning::BehaviorRouteActionInput request;
  request.lane_change = planning::BehaviorRouteLaneChangeRequest{};
  request.lane_change->target_lanelet_ids = {11};
  request.lane_change->prepare_length_m = 20.0;
  request.lane_change->shift_length_m = 30.0;
  const auto waiting = adapter.onRouteFrame(
      frame, route, {}, {}, {}, std::nullopt,
      std::nullopt, 5.0, request);
  check(waiting.behavior_action &&
            waiting.behavior_action->kind ==
                planning::BehaviorRouteActionKind::LaneFollow &&
            waiting.behavior_action->lane_change_candidate_count == 1,
        "지도 목표 차로에서 변경 후보를 만들고 승인 전 경로를 유지합니다");
  adapter.reset();
  request.lane_change->execution.activated = true;
  const auto approved = adapter.onRouteFrame(
      frame, route, {}, {}, {}, std::nullopt,
      std::nullopt, 5.0, request);
  check(approved.behavior_action &&
            approved.behavior_action->kind ==
                planning::BehaviorRouteActionKind::LaneChange &&
            approved.behavior_action->lane_change_shift_end_index &&
            approved.behavior_action->path.lane_ids.back() ==
                std::vector<std::int64_t>{11} &&
            approved.pipeline.status ==
                planning::PlanningPipelineStatus::Complete,
        "지도에서 생성한 차로 변경을 목표 차로 경계와 함께 종단 계산합니다");
  {
    auto running_frame = frame;
    running_frame.timestamp_s += 0.1;
    running_frame.ego.x += 4.0;
    const auto running = adapter.onRouteFrame(
        running_frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, request);
    check(running.behavior_action &&
              running.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneChange &&
              running.behavior_action->path.points.size() ==
                  approved.behavior_action->path.points.size() &&
              std::hypot(
                  running.behavior_action->path.points.front().x -
                      approved.behavior_action->path.points.front().x,
                  running.behavior_action->path.points.front().y -
                      approved.behavior_action->path.points.front().y) < 1.0e-6,
          "승인된 차로 변경은 다음 콜백에서 시프트 시작점을 다시 만들지 않습니다");
    running_frame.timestamp_s += 0.1;
    const auto without_new_request = adapter.onRouteFrame(
        running_frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, std::nullopt, true);
    check(without_new_request.behavior_action &&
              without_new_request.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneChange &&
              without_new_request.behavior_action->path.points.size() ==
                  approved.behavior_action->path.points.size() &&
              std::hypot(
                  without_new_request.behavior_action->path.points.front().x -
                      approved.behavior_action->path.points.front().x,
                  without_new_request.behavior_action->path.points.front().y -
                      approved.behavior_action->path.points.front().y) < 1.0e-6,
          "실행 중 후보 요청이 사라져도 승인 경로를 유지합니다");
    auto deactivated = request;
    deactivated.lane_change->execution.activated = false;
    deactivated.lane_change->execution.failure.force_deactivated = true;
    running_frame.timestamp_s += 0.1;
    const auto released = adapter.onRouteFrame(
        running_frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, deactivated);
    check(released.behavior_action &&
              released.behavior_action->kind !=
                  planning::BehaviorRouteActionKind::LaneChange,
          "외부 해제는 저장한 차로 변경 경로를 더 이상 출력하지 않습니다");
    adapter.reset();
  }
  {
    planning::PlanningPipelineMapAdapter externally_requested(
        mapBytes(), {});
    auto directional = request;
    directional.lane_change->target_lanelet_ids.clear();
    directional.lane_change->external_direction =
        planning::BehaviorRouteLaneChangeRequest::ExternalDirection::Left;
    const auto left = externally_requested.onRouteFrame(
        frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, directional);
    check(left.behavior_action &&
              left.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneChange &&
              left.behavior_action->path.lane_ids.back() ==
                  std::vector<std::int64_t>{11},
          "외부 왼쪽 차로 변경 요청은 인접 도로 차로를 자동 선택합니다");
    directional.lane_change->avoidance_activation.emplace();
    directional.lane_change->avoidance_activation->objects.push_back(
        {80.0, 20.0, 1, true});
    directional.lane_change->avoidance_activation->
        minimum_lane_change_buffer_m = 30.0;
    const auto avoidance_requested = externally_requested.onRouteFrame(
        frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, directional);
    check(avoidance_requested.behavior_action &&
              avoidance_requested.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneChange,
          "회피 객체가 충분히 멀면 외부 요청 차로 변경 경로를 사용합니다");
    directional.lane_change->avoidance_activation->objects.front().
        longitudinal_distance_m = 10.0;
    const auto avoidance_too_close = externally_requested.onRouteFrame(
        frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, directional);
    check(avoidance_too_close.behavior_action &&
              avoidance_too_close.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneFollow,
          "회피 객체가 너무 가까우면 차로 변경 후보를 실행하지 않습니다");
    directional.lane_change->avoidance_activation.reset();
    directional.lane_change->external_direction =
        planning::BehaviorRouteLaneChangeRequest::ExternalDirection::Right;
    const auto right = externally_requested.onRouteFrame(
        frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, directional);
    check(right.behavior_action &&
              right.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneFollow,
          "인접 도로 차로가 없는 외부 오른쪽 요청은 변경하지 않습니다");
  }
  {
    planning::PlanningPipelineMapAdapter perceived_avoidance(
        mapBytes(), {});
    auto avoidance_request = request;
    avoidance_request.lane_change->avoidance_activation.emplace();
    planning::PlanningCruisePerceptionFrame perception;
    planning::PlanningPerceivedObject roadside;
    roadside.id = "회피 차로 변경 객체";
    roadside.classification = 1;
    roadside.x = 80.0;
    roadside.y = -1.4;
    roadside.length_m = 4.0;
    roadside.width_m = 1.8;
    perception.objects.push_back(roadside);
    perception.stamp_s = frame.timestamp_s;
    const auto waiting_for_stop = perceived_avoidance.onRouteFrame(
        frame, route, {}, perception, {}, std::nullopt,
        std::nullopt, 5.0, avoidance_request);
    check(waiting_for_stop.behavior_action &&
              waiting_for_stop.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneFollow,
          "회피 객체의 정차 시간이 부족하면 차로 변경을 실행하지 않습니다");
    auto later_frame = frame;
    planning::PlanningPipelineMapResult observed_change;
    for (int second = 2; second <= 6; ++second) {
      later_frame.timestamp_s = second;
      perception.stamp_s = second;
      observed_change = perceived_avoidance.onRouteFrame(
          later_frame, route, {}, perception, {}, std::nullopt,
          std::nullopt, 5.0, avoidance_request);
    }
    check(observed_change.avoidance_perception_input_count > 0 &&
              observed_change.behavior_action &&
              observed_change.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneChange,
          "같은 인식 콜백의 정차 객체에서 회피 차로 변경 실행 조건을 계산합니다");
    check(!observed_change.behavior_action->path.points.empty() &&
              observed_change.behavior_action->path.points.back().y > 1.0,
          "오른쪽 회피 객체가 왼쪽 목표 차로를 자동 선택합니다");
    planning::PlanningPipelineMapAdapter nearer_avoidance(
        mapBytes(), {});
    roadside.x = 45.0;
    perception.objects.front() = roadside;
    planning::PlanningPipelineMapResult nearer_change;
    for (int second = 1; second <= 6; ++second) {
      later_frame.timestamp_s = second;
      perception.stamp_s = second;
      nearer_change = nearer_avoidance.onRouteFrame(
          later_frame, route, {}, perception, {}, std::nullopt,
          std::nullopt, 5.0, avoidance_request);
    }
    check(nearer_change.behavior_action &&
              nearer_change.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneChange,
          "원본 최소 차로 변경 거리 이내의 유효 객체에서 변경 후보를 실행합니다");
    auto left_object_request = request;
    left_object_request.lane_change->avoidance_activation.emplace();
    left_object_request.lane_change->avoidance_activation->objects.push_back(
        {80.0, 5.0, 1, true, false});
    const auto no_right_lane = perceived_avoidance.onRouteFrame(
        later_frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, left_object_request);
    check(no_right_lane.behavior_action &&
              no_right_lane.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneFollow,
          "왼쪽 회피 객체의 오른쪽 목표 차로가 없으면 변경하지 않습니다");
  }
  {
    planning::PlanningPipelineMapAdapter sampled_map(
        mapBytes(), {});
    planning::BehaviorRouteActionInput sampled_request;
    sampled_request.sampling =
        planning::BehaviorRouteSamplingRequest{};
    sampled_request.sampling->sampling.target_lengths_m = {20.0};
    sampled_request.sampling->sampling.target_lateral_positions_m =
        {-1.8, 0.0, 1.8};
    sampled_request.sampling->sampling.target_lateral_derivatives =
        {0.0};
    planning::PlanningCruisePerceptionFrame perception;
    planning::PlanningPerceivedObject obstacle;
    obstacle.id = "표본 계획 장애물";
    obstacle.x = 24.5;
    obstacle.y = 0.0;
    obstacle.length_m = 0.5;
    obstacle.width_m = 0.4;
    perception.objects.push_back(obstacle);
    perception.stamp_s = frame.timestamp_s;
    const auto sampled_result = sampled_map.onRouteFrame(
        frame, route, {}, perception, {}, std::nullopt,
        std::nullopt, 5.0, sampled_request);
    check(sampled_result.behavior_action &&
              sampled_result.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::Sampling &&
              sampled_result.behavior_action->sampling_candidate_count == 3 &&
              std::abs(sampled_result.behavior_action->path.points.back().y) > 1.5,
          "지도 콜백의 인식 객체를 표본 경로 충돌 검사에 전달합니다");
    auto next_sampling_frame = frame;
    next_sampling_frame.timestamp_s += 1.0;
    perception.stamp_s = next_sampling_frame.timestamp_s;
    const auto extended_sample = sampled_map.onRouteFrame(
        next_sampling_frame, route, {}, perception, {},
        std::nullopt, std::nullopt, 5.0, sampled_request);
    check(extended_sample.behavior_action &&
              extended_sample.behavior_action->sampling_candidate_count >
                  sampled_result.behavior_action->sampling_candidate_count,
          "다음 지도 콜백에서 이전 표본 경로를 보존하고 연장 후보를 만듭니다");
    sampled_request.sampling->previous_path_reuse_points = 0;
    sampled_request.sampling->require_unsafe_reference = false;
    sampled_request.sampling->sampling.target_lateral_positions_m =
        {4.5};
    next_sampling_frame.timestamp_s += 1.0;
    const auto adjacent_sample = sampled_map.onRouteFrame(
        next_sampling_frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, sampled_request);
    check(adjacent_sample.behavior_action &&
              adjacent_sample.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::Sampling &&
              adjacent_sample.behavior_action->path.lane_ids.back() ==
                  std::vector<std::int64_t>{11},
          "인접 차로로 들어간 표본 경로의 출력 차로 ID를 지도에서 고릅니다");
  }
  {
    planning::PlanningPipelineMapAdapter dynamic_map(
        mapBytes(), {});
    planning::BehaviorRouteActionInput dynamic_request;
    dynamic_request.dynamic_avoidance =
        planning::BehaviorRouteDynamicAvoidanceRequest{};
    planning::PlanningCruisePerceptionFrame perception;
    planning::PlanningPerceivedObject moving;
    moving.id = "연속 관측 차량";
    moving.classification = 1;
    moving.x = 60.0;
    moving.y = 2.5;
    moving.length_m = 4.0;
    moving.width_m = 1.8;
    moving.velocity_x_mps = 3.0;
    perception.objects.push_back(moving);
    auto dynamic_frame = frame;
    for (int second = 1; second <= 5; ++second) {
      dynamic_frame.timestamp_s = second;
      perception.stamp_s = second;
      const auto observed = dynamic_map.onRouteFrame(
          dynamic_frame, route, {}, perception, {},
          std::nullopt, std::nullopt, 5.0, dynamic_request);
      check(observed.dynamic_avoidance_targets.size() ==
                static_cast<std::size_t>(second == 5) &&
                observed.dynamic_avoidance_candidate_polygons.size() ==
                    static_cast<std::size_t>(second == 5),
          "동일 지도 콜백의 차량을 다섯 번 연속 관측한 뒤 회피 대상으로 등록합니다");
      if (second == 5)
        check(observed.dynamic_avoidance_selected_targets.size() == 1 &&
                  observed.dynamic_avoidance_selected_targets.front().id ==
                      moving.id &&
                  observed.dynamic_drivable_area.applied_obstacles == 1,
              "선별된 동적 회피 다각형을 종단 최적화 경계에 전달합니다");
      if (second == 5) {
        const auto &bound = observed.dynamic_drivable_area.left_bound;
        check(std::any_of(bound.begin(), bound.end(),
                          [](const auto &point) { return point.y < 2.9; }),
              "왼쪽 동적 장애물이 실제 주행 경계를 안쪽으로 줄입니다");
      }
    }
    planning::BehaviorRouteActionInput wide_request = dynamic_request;
    wide_request.dynamic_avoidance->parameters.successive_entry_count = 1;
    planning::PlanningPipelineMapAdapter wide_vehicle_map(
        mapBytes(), {}, {}, {},
        planning::InvalidTrajectoryPolicy::UsePreviousResult,
        {}, {}, planning::CruiseVehicleGeometry{3.79, 1.10, 6.0});
    perception.stamp_s = 5.0;
    dynamic_frame.timestamp_s = 5.0;
    const auto wide_vehicle = wide_vehicle_map.onRouteFrame(
        dynamic_frame, route, {}, perception, {},
        std::nullopt, std::nullopt, 5.0, wide_request);
    check(wide_vehicle.dynamic_avoidance_targets.size() == 1 &&
              wide_vehicle.dynamic_avoidance_selected_targets.empty() &&
              wide_vehicle.dynamic_avoidance_candidate_polygons.empty(),
          "동적 회피 최종 대상은 설정한 차량 폭으로 앞 차량을 분류합니다");
    dynamic_frame.timestamp_s = 6.0;
    perception.objects.clear();
    perception.stamp_s = 6.0;
    const auto missed = dynamic_map.onRouteFrame(
        dynamic_frame, route, {}, perception, {},
        std::nullopt, std::nullopt, 5.0, dynamic_request);
    check(missed.dynamic_avoidance_targets.size() == 1 &&
              missed.dynamic_avoidance_selected_targets.size() == 1 &&
              missed.dynamic_avoidance_candidate_polygons.size() == 1 &&
              missed.dynamic_drivable_area.applied_obstacles == 1,
          "일시 누락한 동적 회피 객체의 외형과 회피 경계를 다음 콜백에 보존합니다");
    planning::BehaviorRouteActionInput selection_request;
    selection_request.dynamic_avoidance =
        planning::BehaviorRouteDynamicAvoidanceRequest{};
    selection_request.dynamic_avoidance->parameters
        .successive_entry_count = 1;
    planning::PlanningPerceivedObject follower = moving;
    follower.id = "앞 차량";
    follower.y = 0.0;
    planning::PlanningCruisePerceptionFrame selection_perception;
    selection_perception.stamp_s = 1.0;
    selection_perception.objects = {follower};
    planning::PlanningPipelineMapAdapter follower_map(mapBytes(), {});
    auto selection_frame = frame;
    selection_frame.timestamp_s = 1.0;
    const auto followed = follower_map.onRouteFrame(
        selection_frame, route, {}, selection_perception, {},
        std::nullopt, std::nullopt, 5.0, selection_request);
    check(followed.dynamic_avoidance_targets.size() == 1 &&
              followed.dynamic_avoidance_selected_targets.empty() &&
              followed.dynamic_avoidance_candidate_polygons.empty(),
          "앞에서 같은 방향으로 달리는 차량은 최종 회피 대상에서 제외합니다");
    follower.y = 2.5;
    follower.velocity_y_mps = 0.5;
    selection_perception.objects = {follower};
    selection_perception.stamp_s = 2.0;
    selection_frame.timestamp_s = 2.0;
    const auto cut_out = follower_map.onRouteFrame(
        selection_frame, route, {}, selection_perception, {},
        std::nullopt, std::nullopt, 5.0, selection_request);
    check(cut_out.dynamic_avoidance_targets.size() == 1 &&
              cut_out.dynamic_avoidance_candidate_polygons.empty(),
          "최근 자차 통로를 벗어난 차량은 cut-out 상태로 회피에서 제외합니다");

    auto cutting = moving;
    cutting.id = "진입 예측 차량";
    planning::CruisePredictedPath predicted;
    predicted.confidence = 1.0;
    predicted.time_step_s = 1.0;
    predicted.poses = {{60.0, 2.5, 0.0}, {65.0, 0.0, 0.0}};
    cutting.predicted_paths = {predicted};
    selection_perception.objects = {cutting};
    selection_perception.stamp_s = 1.0;
    selection_frame.timestamp_s = 1.0;
    planning::PlanningPipelineMapAdapter cut_in_map(mapBytes(), {});
    const auto cut_in = cut_in_map.onRouteFrame(
        selection_frame, route, {}, selection_perception, {},
        std::nullopt, std::nullopt, 5.0, selection_request);
    check(cut_in.dynamic_avoidance_targets.size() == 1 &&
              cut_in.dynamic_avoidance_candidate_polygons.empty(),
          "자차 통로로 진입할 차량은 원본의 최종 회피 제외 조건을 적용합니다");
    auto stale_prediction = moving;
    stale_prediction.id = "초기 자세 대상";
    planning::CruisePredictedPath departing;
    departing.confidence = 1.0;
    departing.time_step_s = 1.0;
    departing.poses = {{60.0, 2.5, 0.0}, {60.0, 20.0, 0.0}};
    stale_prediction.predicted_paths = {departing};
    selection_perception.objects = {stale_prediction};
    selection_perception.stamp_s = 1.0;
    selection_frame.timestamp_s = 2.0;
    planning::PlanningPipelineMapAdapter initial_pose_map(mapBytes(), {});
    const auto initial_pose_target = initial_pose_map.onRouteFrame(
        selection_frame, route, {}, selection_perception, {},
        std::nullopt, std::nullopt, 5.0, selection_request);
    check(initial_pose_target.dynamic_avoidance_targets.size() == 1 &&
              initial_pose_target.dynamic_avoidance_candidate_polygons.size() == 1,
          "동적 회피 객체 선별은 예측 시각 보간이 아닌 perception 초기 자세를 사용합니다");
    auto transverse = moving;
    transverse.id = "뒤쪽 회전 차량";
    transverse.x = 2.0;
    transverse.y = 6.0;
    transverse.yaw_rad = std::acos(-1.0) * 0.5;
    transverse.length_m = 8.0;
    transverse.velocity_x_mps = 0.5;
    selection_perception.objects = {transverse};
    selection_perception.stamp_s = 1.0;
    selection_frame.timestamp_s = 1.0;
    planning::PlanningPipelineMapAdapter transverse_map(mapBytes(), {});
    const auto passed_transverse = transverse_map.onRouteFrame(
        selection_frame, route, {}, selection_perception, {},
        std::nullopt, std::nullopt, 5.0, selection_request);
    check(passed_transverse.dynamic_avoidance_targets.size() == 1 &&
              passed_transverse.dynamic_avoidance_candidate_polygons.empty(),
          "자차 뒤에 완전히 놓인 회전 차량은 실제 외곽 끝점으로 제외합니다");
    auto pedestrian = moving;
    pedestrian.id = "측방 이동 보행자";
    pedestrian.classification = 7;
    pedestrian.length_m = 0.6;
    pedestrian.width_m = 0.6;
    pedestrian.velocity_x_mps = 0.4;
    pedestrian.velocity_y_mps = 0.0;
    planning::CruisePredictedPath walking_path;
    walking_path.poses = {{60.0, 2.5, 0.0},
                          {61.0, 2.5, 0.0}};
    walking_path.time_step_s = 1.0;
    walking_path.confidence = 1.0;
    pedestrian.predicted_paths = {walking_path};
    selection_perception.objects = {pedestrian};
    planning::PlanningPipelineMapAdapter pedestrian_map(mapBytes(), {});
    const auto passing_pedestrian = pedestrian_map.onRouteFrame(
        selection_frame, route, {}, selection_perception, {},
        std::nullopt, std::nullopt, 5.0, selection_request);
    check(passing_pedestrian.dynamic_avoidance_targets.size() == 1 &&
              passing_pedestrian.dynamic_avoidance_final_targets.size() == 1 &&
              passing_pedestrian.dynamic_avoidance_selected_targets.size() == 1,
          "자차 통로 바깥을 걷는 보행자는 최종 회피 대상으로 남깁니다");
    auto no_prediction = pedestrian;
    no_prediction.predicted_paths.clear();
    selection_perception.objects = {no_prediction};
    planning::PlanningPipelineMapAdapter no_prediction_map(mapBytes(), {});
    const auto final_without_polygon = no_prediction_map.onRouteFrame(
        selection_frame, route, {}, selection_perception, {},
        std::nullopt, std::nullopt, 5.0, selection_request);
    check(final_without_polygon.dynamic_avoidance_final_targets.size() == 1 &&
              final_without_polygon.dynamic_avoidance_selected_targets.empty() &&
              final_without_polygon.dynamic_avoidance_candidate_polygons.empty(),
          "최종 회피 판정과 예측 경로 다각형 생성 실패를 구분합니다");
    pedestrian.y = 0.8;
    pedestrian.velocity_y_mps = -0.5;
    pedestrian.predicted_paths.front().poses =
        {{60.0, 0.8, 0.0}, {61.0, 0.3, 0.0}};
    selection_perception.objects = {pedestrian};
    planning::PlanningPipelineMapAdapter crossing_pedestrian_map(
        mapBytes(), {});
    const auto crossing_pedestrian =
        crossing_pedestrian_map.onRouteFrame(
            selection_frame, route, {}, selection_perception, {},
            std::nullopt, std::nullopt, 5.0,
            selection_request);
    check(crossing_pedestrian.dynamic_avoidance_targets.size() == 1 &&
              crossing_pedestrian.dynamic_avoidance_final_targets.empty() &&
              crossing_pedestrian.dynamic_avoidance_selected_targets.empty(),
          "3초 측방 예측이 자차 경로를 가로지르면 회피 대상에서 제외합니다");
  }
  {
    planning::PlanningPipelineMapAdapter regulated(mapBytes(true), {});
    planning::LaneletBinAdapter crosswalk_map(mapBytes(true));
    check(crosswalk_map.pointInCrosswalk(42.0, 0.0) &&
              !crosswalk_map.pointInCrosswalk(20.0, 0.0),
          "횡단보도 안팎의 객체 위치를 이진 지도에서 구분합니다");
    const auto crossing = regulated.onRouteFrame(
        frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, request);
    check(crossing.behavior_action &&
              crossing.behavior_action->lane_change_candidate_count == 0 &&
              crossing.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneFollow,
          "전방 횡단보도보다 긴 차로 변경 후보를 만들지 않습니다");
    request.lane_change->regulation.stopped_time_s = 4.0;
    const auto stopped = regulated.onRouteFrame(
        frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, request);
    check(stopped.behavior_action &&
              stopped.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneChange,
          "정차 시간이 임계값을 넘으면 횡단보도 제한을 해제합니다");
    // This next case starts a separate stop sequence. A running lane-change
    // module intentionally keeps its approved path across callbacks.
    regulated.reset();
    request.lane_change->regulation.stopped_time_s.reset();
    auto stopped_frame = frame;
    stopped_frame.ego_speed_mps = 0.0;
    const auto first_stop = regulated.onRouteFrame(
        stopped_frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, request);
    check(first_stop.behavior_action &&
              first_stop.behavior_action->lane_change_candidate_count == 0,
          "정차 직후에는 횡단보도 제한을 유지합니다");
    stopped_frame.timestamp_s += 4.0;
    const auto later_stop = regulated.onRouteFrame(
        stopped_frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, request);
    check(later_stop.behavior_action &&
              later_stop.behavior_action->lane_change_candidate_count == 1,
          "연속 콜백의 자차 정차 시간을 차로 변경 제한에 전달합니다");
    request.lane_change->sampling.enabled = true;
    request.lane_change->shift_length_m = 50.0;
    regulated.reset();
    const auto sampled_regulation = regulated.onRouteFrame(
        frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, request);
    check(sampled_regulation.behavior_action &&
              sampled_regulation.behavior_action->lane_change_candidate_count == 0,
          "최대 준비 길이가 횡단보도까지 거리보다 크면 표본 검색을 시작하지 않습니다");
    request.lane_change->sampling.enabled = false;
    request.lane_change->shift_length_m = 30.0;
  }
  adapter.reset();
  request.lane_change->sampling.enabled = true;
  request.lane_change->shift_length_m = 50.0;
  const auto sampled = adapter.onRouteFrame(
      frame, route, {}, {}, {}, std::nullopt,
      std::nullopt, 5.0, request);
  check(sampled.behavior_action &&
            sampled.behavior_action->kind ==
                planning::BehaviorRouteActionKind::LaneChange &&
            sampled.behavior_action->lane_change_candidate_count > 0,
        "지도 기준선에서 준비·횡이동 시간 표본을 검색합니다");

  adapter.reset();
  request.lane_change->sampling.enabled = false;
  request.lane_change->shift_length_m = 30.0;
  planning::PlanningCruisePerceptionFrame perception;
  planning::PlanningPerceivedObject blocking;
  blocking.id = "목표 차로 장애물";
  blocking.classification = 1;
  blocking.x = 51.0;
  blocking.y = 6.0;
  blocking.length_m = 4.0;
  blocking.width_m = 2.0;
  perception.objects.push_back(blocking);
  request.lane_change->use_perception_objects = true;
  const auto blocked = adapter.onRouteFrame(
      frame, route, {}, perception, {}, std::nullopt,
      std::nullopt, 5.0, request);
  check(blocked.behavior_action &&
            blocked.behavior_action->lane_change_candidate_count == 1 &&
            blocked.behavior_action->kind ==
                planning::BehaviorRouteActionKind::LaneFollow,
        "같은 콜백의 목표 차로 정차 객체가 변경 후보를 거부합니다");
  adapter.reset();
  perception.objects.front().shape_type = 2;
  perception.objects.front().length_m = 0.0;
  perception.objects.front().width_m = 0.0;
  perception.objects.front().footprint = {
      {2.0, 1.0, 0.0}, {-2.0, 1.0, 0.0},
      {-2.0, -1.0, 0.0}, {2.0, -1.0, 0.0}};
  const auto polygon_blocked = adapter.onRouteFrame(
      frame, route, {}, perception, {}, std::nullopt,
      std::nullopt, 5.0, request);
  check(polygon_blocked.behavior_action &&
            polygon_blocked.behavior_action->lane_change_candidate_count == 1 &&
            polygon_blocked.behavior_action->kind ==
                planning::BehaviorRouteActionKind::LaneFollow,
        "외곽 점이 있는 polygon 객체는 크기 필드가 0이어도 원본처럼 검사합니다");
  adapter.reset();
  perception.objects.front().classification = 0;
  const auto unknown_target = adapter.onRouteFrame(
      frame, route, {}, perception, {}, std::nullopt,
      std::nullopt, 5.0, request);
  check(unknown_target.behavior_action &&
            unknown_target.behavior_action->kind ==
                planning::BehaviorRouteActionKind::LaneChange,
        "설정에서 제외한 미분류 객체는 목표 차로 검사에 넣지 않습니다");
  adapter.reset();
  blocking.classification = 5;
  blocking.y = 2.0;
  blocking.width_m = 1.0;
  blocking.velocity_x_mps = 2.0;
  planning::CruisePredictedPath crossing;
  crossing.time_step_s = 1.0;
  crossing.confidence = 1.0;
  crossing.poses.push_back({51.0, 2.0, 0.0});
  for (int index = 1; index <= 10; ++index)
    crossing.poses.push_back({51.0, 6.0, 0.0});
  blocking.predicted_paths = {crossing};
  perception.objects = {blocking};
  const auto motorcycle = adapter.onRouteFrame(
      frame, route, {}, perception, {}, std::nullopt,
      std::nullopt, 5.0, request);
  check(motorcycle.behavior_action &&
            motorcycle.behavior_action->lane_change_candidate_count == 1 &&
            motorcycle.behavior_action->kind ==
                planning::BehaviorRouteActionKind::LaneFollow,
        "예측 경로가 목표 차로로 들어오는 오토바이를 차량으로 분류합니다");
  adapter.reset();
  perception.objects.front().classification = 0;
  const auto unknown_class = adapter.onRouteFrame(
      frame, route, {}, perception, {}, std::nullopt,
      std::nullopt, 5.0, request);
  check(unknown_class.behavior_action &&
            unknown_class.behavior_action->kind ==
                planning::BehaviorRouteActionKind::LaneChange,
        "미분류 객체는 차량의 미래 목표 차로 진입 규칙에 넣지 않습니다");

  adapter.reset();
  request = {};
  request.pull_over = planning::BehaviorRoutePullOverRequest{};
  request.pull_over->goal_lanelet_ids = {11};
  request.pull_over->approved = true;
  const planning::BehaviorGoalPose parking_goal{100.0, 6.0, 0.0};
  const auto parking = adapter.onRouteFrame(
      frame, route, {}, {}, {}, parking_goal,
      std::nullopt, 5.0, request);
  check(parking.behavior_action &&
            parking.behavior_action->kind ==
                planning::BehaviorRouteActionKind::PullOver &&
            parking.behavior_action->path.lane_ids.front() ==
                std::vector<std::int64_t>{10} &&
            parking.behavior_action->path.lane_ids.back() ==
                std::vector<std::int64_t>{11} &&
            parking.pipeline.status ==
                planning::PlanningPipelineStatus::Complete,
        "지도 목표 차로의 주차 횡이동을 종단 경로로 전달합니다");
  {
    planning::PlanningPipelineMapAdapter phased(mapBytes(), {});
    auto geometric = request;
    geometric.pull_over->use_geometric = true;
    geometric.pull_over->use_geometric_fallback = false;
    geometric.pull_over->use_bezier_fallback = false;
    geometric.pull_over->use_freespace_fallback = false;
    geometric.pull_over->vehicle = {0.1, 0.1, 0.1, 0.5};
    const auto first = phased.onRouteFrame(
        frame, route, {}, {}, {}, parking_goal,
        std::nullopt, 5.0, geometric);
    check(first.behavior_action &&
              first.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::PullOver &&
              first.behavior_action->pull_over_phase_end_indices.size() > 1 &&
              first.behavior_action->pull_over_current_phase == 0 &&
              first.behavior_action->path.points.size() <
                  first.behavior_action->pull_over_phase_end_indices.back() + 1,
          "지도 파이프라인이 기하 주차의 첫 구간만 출력합니다");
    auto next_frame = frame;
    next_frame.timestamp_s = frame.timestamp_s + 5.0;
    next_frame.ego = first.behavior_action->path.points.back();
    next_frame.ego_speed_mps = 0.0;
    const auto second = phased.onRouteFrame(
        next_frame, route, {}, {}, {}, parking_goal,
        std::nullopt, 5.0, geometric);
    check(second.behavior_action &&
              second.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::PullOver &&
              second.behavior_action->pull_over_current_phase == 1 &&
              second.behavior_action->path.points.front().x ==
                  first.behavior_action->path.points.back().x,
          "기하 주차는 첫 구간 종점 정차 뒤 두 번째 구간으로 넘어갑니다");
    auto final_frame = next_frame;
    final_frame.timestamp_s += 5.0;
    final_frame.ego = second.behavior_action->path.points.back();
    const auto third = phased.onRouteFrame(
        final_frame, route, {}, {}, {}, parking_goal,
        std::nullopt, 5.0, geometric);
    check(third.behavior_action &&
              third.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::PullOver &&
              third.behavior_action->pull_over_current_phase == 2 &&
              std::hypot(
                  third.behavior_action->path.points.back().x -
                      parking_goal.x,
                  third.behavior_action->path.points.back().y -
                      parking_goal.y) < 1.0e-6,
          "두 번째 구간 종점 정차 뒤 최종 목표 구간을 출력합니다");
    geometric.pull_over->approved = false;
    final_frame.timestamp_s += 1.0;
    const auto cancelled = phased.onRouteFrame(
        final_frame, route, {}, {}, {}, parking_goal,
        std::nullopt, 5.0, geometric);
    check(cancelled.behavior_action &&
              cancelled.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneFollow,
          "주차 승인이 취소되면 보존된 경로를 출력하지 않습니다");
    geometric.pull_over->approved = true;
    auto restart_frame = frame;
    restart_frame.timestamp_s = final_frame.timestamp_s + 1.0;
    const auto restarted = phased.onRouteFrame(
        restart_frame, route, {}, {}, {}, parking_goal,
        std::nullopt, 5.0, geometric);
    check(restarted.behavior_action &&
              restarted.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::PullOver &&
              restarted.behavior_action->pull_over_current_phase == 0,
          "승인을 다시 받으면 첫 주차 구간에서 시작합니다");
    auto blocked_request = geometric;
    blocked_request.pull_over->path_stop_objects.push_back(
        {{20.0, -5.0}, {120.0, -5.0},
         {120.0, 10.0}, {20.0, 10.0}});
    restart_frame.timestamp_s += 1.0;
    const auto newly_blocked = phased.onRouteFrame(
        restart_frame, route, {}, {}, {}, parking_goal,
        std::nullopt, 5.0, blocked_request);
    check(newly_blocked.behavior_action &&
              newly_blocked.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneFollow,
          "새 정지 객체가 나타나면 보존한 주차 경로도 다시 검사합니다");
  }
  planning::PlanningCruisePerceptionFrame path_perception;
  path_perception.objects.push_back(blocking);
  path_perception.objects.front().x = 90.0;
  path_perception.objects.front().y = 4.5;
  path_perception.objects.front().velocity_x_mps = 0.0;
  path_perception.objects.front().velocity_y_mps = 0.0;
  path_perception.objects.front().predicted_paths.clear();
  auto path_request = request;
  path_request.pull_over->use_bezier_fallback = false;
  path_request.pull_over->use_geometric_fallback = false;
  const auto blocked_path_parking = adapter.onRouteFrame(
      frame, route, {}, path_perception, {}, parking_goal,
      std::nullopt, 5.0, path_request);
  check(blocked_path_parking.behavior_action &&
            blocked_path_parking.behavior_action->kind ==
                planning::BehaviorRouteActionKind::LaneFollow &&
            blocked_path_parking.behavior_action->pull_over_candidate_count > 0,
        "동일 콜백의 정지 객체가 목표와 떨어진 주차 경로를 막습니다");
  path_perception.objects.front().x = 0.0;
  path_perception.objects.front().y = 0.0;
  const auto passed_object_parking = adapter.onRouteFrame(
      frame, route, {}, path_perception, {}, parking_goal,
      std::nullopt, 5.0, path_request);
  check(passed_object_parking.behavior_action &&
            passed_object_parking.behavior_action->kind ==
                planning::BehaviorRouteActionKind::PullOver,
        "자차 뒤의 이미 지난 정지 객체는 주차 후보를 차단하지 않습니다");
  auto moving_perception = path_perception;
  moving_perception.objects.front().x = 80.0;
  moving_perception.objects.front().y = 0.0;
  moving_perception.objects.front().width_m = 8.0;
  moving_perception.objects.front().velocity_x_mps = 2.0;
  const auto moving_path_parking = adapter.onRouteFrame(
      frame, route, {}, moving_perception, {}, parking_goal,
      std::nullopt, 5.0, path_request);
  check(moving_path_parking.behavior_action &&
            moving_path_parking.behavior_action->kind ==
                planning::BehaviorRouteActionKind::LaneFollow,
        "동일 인식 프레임의 이동 객체가 주차 경로를 차지하면 실행을 보류합니다");
  planning::BehaviorCandidateCallbackInput parking_candidates;
  parking_candidates.callback_time_ns = 1;
  parking_candidates.perception.receipt_time_ns = 1;
  planning::StartPullOutStopObject at_goal;
  at_goal.pose.x = parking_goal.x;
  at_goal.pose.y = parking_goal.y;
  parking_candidates.perception.objects.push_back(at_goal);
  auto path_candidates = parking_candidates;
  path_candidates.perception.objects.front().pose.x = 90.0;
  path_candidates.perception.objects.front().pose.y = 4.5;
  path_candidates.perception.objects.front().shape.shape_type = 0;
  path_candidates.perception.objects.front().shape.length_m = 5.0;
  path_candidates.perception.objects.front().shape.width_m = 2.0;
  const auto explicit_path_blocked = adapter.onRouteFrame(
      frame, route, {}, {}, {}, parking_goal,
      path_candidates, 5.0, path_request);
  check(explicit_path_blocked.behavior_candidates &&
            explicit_path_blocked.behavior_candidates->goal_safety ==
                planning::BehaviorGoalSafetyReason::Safe &&
            explicit_path_blocked.behavior_action &&
            explicit_path_blocked.behavior_action->kind ==
                planning::BehaviorRouteActionKind::LaneFollow,
        "별도 후보 인식 묶음의 객체도 주차 경로 전체를 막습니다");
  const auto unsafe_parking = adapter.onRouteFrame(
      frame, route, {}, {}, {}, parking_goal,
      parking_candidates, 5.0, request);
  check(unsafe_parking.behavior_candidates &&
            unsafe_parking.behavior_candidates->goal_safety ==
                planning::BehaviorGoalSafetyReason::FootprintObjectCollision &&
            unsafe_parking.behavior_action &&
            unsafe_parking.behavior_action->pull_over_candidate_count > 0 &&
            unsafe_parking.behavior_action->kind ==
                planning::BehaviorRouteActionKind::LaneFollow,
        "같은 콜백의 목표 충돌 판정을 주차 경로 승인에 적용합니다");
  planning::PlanningCruisePerceptionFrame parking_perception;
  parking_perception.objects.push_back(blocking);
  parking_perception.objects.front().x = parking_goal.x;
  parking_perception.objects.front().y = parking_goal.y;
  parking_perception.objects.front().predicted_paths.clear();
  const auto perceived_unsafe = adapter.onRouteFrame(
      frame, route, {}, parking_perception, {}, parking_goal,
      std::nullopt, 5.0, request);
  check(perceived_unsafe.behavior_action &&
            perceived_unsafe.behavior_action->pull_over_candidate_count == 1 &&
            perceived_unsafe.behavior_action->kind ==
                planning::BehaviorRouteActionKind::LaneFollow,
        "공통 인식 프레임의 목표 충돌도 주차 경로 출력에서 거부합니다");
  adapter.reset();
  parking_perception.objects.front().x = 80.0;
  parking_perception.objects.front().y = parking_goal.y;
  parking_perception.objects.front().predicted_paths = {
      {{{80.0, parking_goal.y, 0.0},
        {parking_goal.x, parking_goal.y, 0.0}}, 1.0, 1.0}};
  parking_perception.stamp_s = 1.0;
  frame.timestamp_s = 2.0;
  const auto predicted_unsafe = adapter.onRouteFrame(
      frame, route, {}, parking_perception, {}, parking_goal,
      std::nullopt, 5.0, request);
  check(predicted_unsafe.behavior_action &&
            predicted_unsafe.behavior_action->kind ==
                planning::BehaviorRouteActionKind::LaneFollow,
        "인식 시각과 콜백 시각 사이에 목표로 이동한 객체를 반영합니다");

  {
    planning::PlanningPipelineMapAdapter shift_start(mapBytes(), {});
    planning::BehaviorRouteActionInput start_request;
    start_request.start = planning::BehaviorRouteStartRequest{};
    auto start_frame = frame;
    start_frame.ego.y = -1.0;
    start_frame.ego_speed_mps = 0.0;
    const auto clear = shift_start.onRouteFrame(
        start_frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, start_request);
    check(clear.behavior_action &&
              clear.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::StartPullOut,
          "일반 횡이동 출발을 지도 콜백에서 생성합니다");
    planning::BehaviorCandidateCallbackInput same_callback;
    same_callback.callback_time_ns = 1000000000LL;
    same_callback.perception.receipt_time_ns = 1000000000LL;
    planning::StartPullOutStopObject stop;
    stop.pose.x = 30.0;
    stop.shape.shape_type = 0;
    stop.shape.length_m = 2.0;
    stop.shape.width_m = 2.0;
    same_callback.perception.objects.push_back(stop);
    const auto blocked = shift_start.onRouteFrame(
        start_frame, route, {}, {}, {}, std::nullopt,
        same_callback, 5.0, start_request);
    check(blocked.behavior_action &&
              blocked.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneFollow,
          "일반 횡이동 출발도 동일 콜백의 정지 객체를 경로와 검사합니다");
  }
  {
    planning::PlanningPipelineMapAdapter freespace_start(mapBytes(), {});
    planning::BehaviorRouteActionInput start_request;
    start_request.start = planning::BehaviorRouteStartRequest{};
    start_request.start->use_freespace = true;
    planning::CrosswalkOccupancyGrid grid;
    grid.width = 60;
    grid.height = 40;
    grid.resolution_m = 1.0;
    grid.origin_x = -10.0;
    grid.origin_y = -20.0;
    grid.values.assign(grid.width * grid.height, 0);
    start_request.start->freespace_grid = grid;
    start_request.start->freespace.vehicle_shape_margin_m = 0.0;
    auto start_frame = frame;
    start_frame.ego.y = -2.0;
    start_frame.ego_speed_mps = 0.0;
    const auto departure = freespace_start.onRouteFrame(
        start_frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, start_request);
    check(departure.behavior_action &&
              departure.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::StartPullOut &&
              departure.behavior_candidates &&
              !departure.behavior_candidates->start_candidates.empty() &&
              departure.behavior_action->selected_start_pose &&
              std::abs(departure.behavior_action->selected_start_pose->x -
                       departure.behavior_candidates->start_candidates.front().x) <
                  1.0e-6 &&
              departure.behavior_action->path.points.back().x > 100.0 &&
              departure.pipeline.status ==
                  planning::PlanningPipelineStatus::Complete,
          "지도 출발 경로에 자유공간 탐색과 중심선 연장을 연결합니다");
    planning::BehaviorCandidateCallbackInput same_callback;
    same_callback.callback_time_ns = 1000000000LL;
    same_callback.perception.receipt_time_ns = 1000000000LL;
    planning::StartPullOutStopObject downstream_stop;
    downstream_stop.pose.x = 30.0;
    downstream_stop.shape.shape_type = 0;
    downstream_stop.shape.length_m = 2.0;
    downstream_stop.shape.width_m = 2.0;
    same_callback.perception.objects.push_back(downstream_stop);
    const auto same_callback_blocked = freespace_start.onRouteFrame(
        start_frame, route, {}, {}, {}, std::nullopt,
        same_callback, 5.0, start_request);
    check(same_callback_blocked.behavior_candidates &&
              same_callback_blocked.behavior_candidates->perception_object_count == 1 &&
              same_callback_blocked.behavior_action &&
              same_callback_blocked.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneFollow,
          "출발 후보와 실제 경로 충돌 검사가 같은 인식 묶음을 사용합니다");
    freespace_start.reset();
    planning::PlanningCruisePerceptionFrame obstacle_frame;
    planning::PlanningPerceivedObject obstacle;
    obstacle.id = "출발 장애물";
    obstacle.classification = 1;
    obstacle.x = 13.5;
    obstacle.y = 0.0;
    obstacle.length_m = 13.0;
    obstacle.width_m = 8.0;
    obstacle_frame.objects.push_back(obstacle);
    const auto blocked_start = freespace_start.onRouteFrame(
        start_frame, route, {}, obstacle_frame, {}, std::nullopt,
        std::nullopt, 5.0, start_request);
    check(blocked_start.behavior_action &&
              blocked_start.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::LaneFollow,
          "동일 인식 콜백의 정지 객체가 자유공간 출발 경로를 막습니다");
    auto filtered_start = start_request;
    filtered_start.start->object_types_to_check[1] = false;
    const auto ignored_class = freespace_start.onRouteFrame(
        start_frame, route, {}, obstacle_frame, {}, std::nullopt,
        std::nullopt, 5.0, filtered_start);
    check(ignored_class.behavior_action &&
              ignored_class.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::StartPullOut,
          "출발 경로 생성의 객체 종류 설정을 후보와 충돌 검사에 함께 적용합니다");
  }
  {
    planning::PlanningPipelineMapAdapter geometric_start(mapBytes(), {});
    planning::BehaviorRouteActionInput start_request;
    start_request.start = planning::BehaviorRouteStartRequest{};
    start_request.start->use_geometric = true;
    start_request.start->vehicle = {0.1, 0.1, 0.1, 0.5};
    auto start_frame = frame;
    start_frame.ego.x = 60.0;
    start_frame.ego.y = -2.0;
    start_frame.ego_speed_mps = 0.0;
    const auto first = geometric_start.onRouteFrame(
        start_frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, start_request);
    check(first.behavior_action &&
              first.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::StartPullOut &&
              first.behavior_action->start_phase_end_indices.size() == 2 &&
              first.behavior_action->start_current_phase == 0 &&
              first.pipeline.status ==
                  planning::PlanningPipelineStatus::Complete,
          "지도 기하 출발 경로에서 첫 원호 구간을 출력합니다");
    start_frame.timestamp_s += 5.0;
    start_frame.ego = first.behavior_action->path.points.back();
    const auto second = geometric_start.onRouteFrame(
        start_frame, route, {}, {}, {}, std::nullopt,
        std::nullopt, 5.0, start_request);
    check(second.behavior_action &&
              second.behavior_action->kind ==
                  planning::BehaviorRouteActionKind::StartPullOut &&
              second.behavior_action->start_current_phase == 1 &&
              second.behavior_action->path.points.front().x ==
                  first.behavior_action->path.points.back().x &&
              second.behavior_action->path.points.back()
                      .longitudinal_velocity_mps > 0.0F &&
              second.pipeline.status ==
                  planning::PlanningPipelineStatus::Complete,
          "기하 출발은 원호 종점 정차 뒤 후속 도로 경로로 넘어갑니다");
  }

  adapter.reset();
  request = {};
  request.avoidance = planning::BehaviorRouteAvoidanceRequest{};
  request.avoidance->approved = true;
  request.avoidance->use_perception_objects = true;
  auto &parameters = request.avoidance->perception_parameters;
  parameters.nominal_avoidance_speed_mps = 4.0;
  parameters.nominal_lateral_jerk_mps3 = 1.0;
  parameters.maximum_prepare_time_s = 1.0;
  perception.objects.front().id = "정차 차량";
  perception.objects.front().classification = 1;
  perception.objects.front().x = 80.0;
  perception.objects.front().y = -1.5;
  perception.objects.front().width_m = 1.0;
  perception.objects.front().velocity_x_mps = 0.0;
  perception.objects.front().predicted_paths.clear();
  perception.stamp_s = 1.0;
  const auto early = adapter.onRouteFrame(
      frame, route, {}, perception, {}, std::nullopt,
      std::nullopt, 5.0, request);
  check(early.behavior_action &&
            early.behavior_action->avoidance_candidate_count == 0,
        "정차 시간 2초 전에는 회피 후보를 만들지 않습니다");
  const auto stationary_object = perception.objects.front();
  perception.objects.clear();
  perception.stamp_s = 2.0;
  frame.timestamp_s = 2.0;
  adapter.onRouteFrame(
      frame, route, {}, perception, {}, std::nullopt,
      std::nullopt, 5.0, std::nullopt);
  perception.objects.push_back(stationary_object);
  perception.stamp_s = 3.0;
  frame.timestamp_s = 3.0;
  const auto still_ambiguous = adapter.onRouteFrame(
      frame, route, {}, perception, {}, std::nullopt,
      std::nullopt, 5.0, request);
  check(still_ambiguous.behavior_action &&
            still_ambiguous.behavior_action->avoidance_candidate_count == 0,
        "같은 차로의 정차 차량은 모호한 차량 대기 시간 3초까지 기다립니다");
  perception.stamp_s = 4.1;
  frame.timestamp_s = 4.1;
  const auto stationary = adapter.onRouteFrame(
      frame, route, {}, perception, {}, std::nullopt,
      std::nullopt, 5.0, request);
  check(stationary.behavior_action &&
            stationary.behavior_action->avoidance_candidate_count == 1 &&
            stationary.behavior_action->kind ==
                planning::BehaviorRouteActionKind::StaticAvoidance,
        "짧은 인식 누락 뒤에도 정차 이력을 유지해 회피 경로를 만듭니다");
  perception.objects.front().x = 82.0;
  perception.stamp_s = 5.1;
  frame.timestamp_s = 5.1;
  const auto moved = adapter.onRouteFrame(
      frame, route, {}, perception, {}, std::nullopt,
      std::nullopt, 5.0, request);
  check(moved.behavior_action &&
            moved.behavior_action->avoidance_candidate_count == 0,
        "객체가 1m 넘게 이동하면 모호한 정차 차량에서 제외합니다");
  perception.stamp_s = 7.1;
  frame.timestamp_s = 7.1;
  const auto still_moved = adapter.onRouteFrame(
      frame, route, {}, perception, {}, std::nullopt,
      std::nullopt, 5.0, request);
  check(still_moved.behavior_action &&
            still_moved.behavior_action->avoidance_candidate_count == 0,
        "계속 저속인 객체는 정차 시간만 지나도 초기 위치 판정을 잃지 않습니다");
  perception.objects.clear();
  perception.stamp_s = 10.2;
  frame.timestamp_s = 10.2;
  adapter.onRouteFrame(
      frame, route, {}, perception, {}, std::nullopt,
      std::nullopt, 5.0, request);
  perception.objects.push_back(stationary_object);
  perception.stamp_s = 11.2;
  frame.timestamp_s = 11.2;
  const auto long_missing = adapter.onRouteFrame(
      frame, route, {}, perception, {}, std::nullopt,
      std::nullopt, 5.0, request);
  check(long_missing.behavior_action &&
            long_missing.behavior_action->avoidance_candidate_count == 0,
        "2초 넘는 인식 누락 뒤에는 정차 시간을 다시 셉니다");
  adapter.reset();
  perception.objects.front().id = "차로를 침범한 인접 객체";
  perception.objects.front().x = 80.0;
  perception.objects.front().y = -3.5;
  perception.objects.front().width_m = 2.0;
  perception.stamp_s = 10.0;
  frame.timestamp_s = 10.0;
  adapter.onRouteFrame(
      frame, route, {}, perception, {}, std::nullopt,
      std::nullopt, 5.0, request);
  perception.stamp_s = 11.5;
  frame.timestamp_s = 11.5;
  adapter.onRouteFrame(
      frame, route, {}, perception, {}, std::nullopt,
      std::nullopt, 5.0, request);
  perception.stamp_s = 13.0;
  frame.timestamp_s = 13.0;
  adapter.onRouteFrame(
      frame, route, {}, perception, {}, std::nullopt,
      std::nullopt, 5.0, request);
  perception.stamp_s = 14.0;
  frame.timestamp_s = 14.0;
  const auto lane_overhang = adapter.onRouteFrame(
      frame, route, {}, perception, {}, std::nullopt,
      std::nullopt, 5.0, request);
  check(lane_overhang.behavior_action &&
            lane_overhang.avoidance_perception_input_count == 1,
        "중심은 차로 밖이어도 외곽이 현재 차로에 닿는 정차 객체를 전달합니다");
  return EXIT_SUCCESS;
}
