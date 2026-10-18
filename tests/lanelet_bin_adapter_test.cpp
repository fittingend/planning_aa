#include "planning/lanelet_bin_adapter.hpp"
#include "planning/behavior_velocity_map_planner.hpp"
#include "planning/planning_pipeline_map_adapter.hpp"

#include <lanelet2_io/io_handlers/Serialize.h>
#include <lanelet2_core/primitives/RegulatoryElement.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/point_on_surface.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

void check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

std::vector<std::uint8_t> readMap(const char *path) {
  std::ifstream stream(path, std::ios::binary);
  check(static_cast<bool>(stream), "이진 지도 파일을 열 수 있어야 합니다");
  return {std::istreambuf_iterator<char>(stream),
          std::istreambuf_iterator<char>()};
}

bool near(double a, double b) { return std::abs(a - b) < 1.0e-9; }

}  // 익명 네임스페이스

int main(int argc, char **argv) {
  if (argc != 6) return EXIT_FAILURE;
  planning::LaneletBinAdapter original(readMap(argv[1]));
  const auto goal_centerline = original.laneCenterline(24747);
  check(goal_centerline.size() >= 3,
        "실측 목표 차로 중심선이 충분한 점을 포함합니다");
  double goal_centerline_length = 0.0;
  for (std::size_t index = 1; index < goal_centerline.size(); ++index) {
    goal_centerline_length += std::hypot(
        goal_centerline[index].x - goal_centerline[index - 1].x,
        goal_centerline[index].y - goal_centerline[index - 1].y);
  }
  const auto mapped_change = original.laneChangeTargetReference(
      {24747}, {goal_centerline.front().x, goal_centerline.front().y},
      goal_centerline_length / 4.0, 5.0,
      goal_centerline_length / 4.0, 0.0);
  check(mapped_change && mapped_change->path.size() >= 2 &&
            mapped_change->path.size() == mapped_change->lane_ids.size() &&
            std::abs(mapped_change->end_arc_m -
                     goal_centerline_length / 2.0) < 1.0e-3 &&
            mapped_change->lane_ids.front() ==
                std::vector<std::int64_t>{24747},
        "실측 지도 목표 차로에서 ID를 보존한 변경 기준선을 추출합니다");
  const auto goal_polygons = original.lanePolygons({24747});
  planning::TrajectoryPoint center_sample;
  const auto center_index = goal_centerline.size() / 2;
  center_sample.x = goal_centerline[center_index].x;
  center_sample.y = goal_centerline[center_index].y;
  check(original.nearestRoadLaneletId(
            center_sample.x, center_sample.y) == 24747 &&
            !original.nearestRoadLaneletId(
                center_sample.x + 100000.0,
                center_sample.y + 100000.0, 1.0),
        "실측 중심선 근처 도로 차로를 찾고 먼 위치는 제외합니다");
  planning::BehaviorPathVehicleFootprint small_vehicle;
  small_vehicle.front_m = 0.1;
  small_vehicle.back_m = 0.1;
  small_vehicle.half_width_m = 0.1;
  check(goal_polygons.size() == 1 && goal_polygons.front().size() >= 4 &&
            !planning::firstBehaviorPathLaneDeparture(
                {center_sample}, goal_polygons, small_vehicle),
        "실측 차로 경계 다각형 안에 중심선 점이 포함됩니다");
  const auto nearby_polygons = original.lanePolygonsNearPath(
      {center_sample}, small_vehicle);
  check(!nearby_polygons.empty() &&
            !planning::firstBehaviorPathLaneDeparture(
                {center_sample}, nearby_polygons, small_vehicle),
        "실측 지도에서 차량 외곽 주변 차로를 자동 선택합니다");
  planning::BehaviorGoalPath goal_reference;
  for (std::size_t index = 0; index < goal_centerline.size(); ++index) {
    planning::TrajectoryPoint point;
    point.x = goal_centerline[index].x;
    point.y = goal_centerline[index].y;
    const std::size_t next = std::min(
        index + 1, goal_centerline.size() - 1);
    const std::size_t previous = next == index ? index - 1 : index;
    const double heading = std::atan2(
        goal_centerline[next].y - goal_centerline[previous].y,
        goal_centerline[next].x - goal_centerline[previous].x);
    point.orientation_z = std::sin(heading / 2.0);
    point.orientation_w = std::cos(heading / 2.0);
    point.longitudinal_velocity_mps = 5.0F;
    goal_reference.points.push_back(point);
    goal_reference.lane_ids.push_back({24747});
  }
  planning::PlanningPipelineMapAdapter map_pipeline(readMap(argv[1]), {});
  planning::PlanningPipelineFrame map_frame;
  map_frame.timestamp_s = 1.0;
  map_frame.behavior_path = goal_reference;
  map_frame.ego = goal_reference.points.front();
  map_frame.ego_speed_mps = 5.0;
  const auto road_bounds = original.laneBounds(24747);
  for (const auto &point : road_bounds.left) {
    map_frame.left_bound.push_back({point.x, point.y});
  }
  for (const auto &point : road_bounds.right) {
    map_frame.right_bound.push_back({point.x, point.y});
  }
  const auto mapped_pipeline = map_pipeline.onFrame(map_frame, {});
  check(mapped_pipeline.behavior_velocity &&
            !mapped_pipeline.behavior_velocity->trajectory.empty() &&
            mapped_pipeline.pipeline.status ==
                planning::PlanningPipelineStatus::Complete &&
            mapped_pipeline.pipeline.validation.has_value(),
        "실측 지도 입력을 행동 속도 플러그인부터 최종 검증까지 전달합니다");
  planning::MissionRoute direct_route;
  direct_route.sections.push_back({24747, {24747}});
  planning::PlanningPipelineFrame route_frame = map_frame;
  route_frame.behavior_path = {};
  route_frame.left_bound.clear();
  route_frame.right_bound.clear();
  const auto route_pipeline = map_pipeline.onRouteFrame(
      route_frame, direct_route, {});
  check(route_pipeline.pipeline.status ==
            planning::PlanningPipelineStatus::Complete &&
            route_pipeline.pipeline.behavior_velocity.size() >= 2,
        "실측 지도 차로열에서 행동 경로를 생성해 종단 계산합니다");
  planning::BehaviorCandidateCallbackInput candidate_input;
  candidate_input.callback_time_ns = 100;
  candidate_input.perception.receipt_time_ns = 100;
  candidate_input.goal.goal = {
      goal_centerline[center_index].x,
      goal_centerline[center_index].y, 0.0};
  candidate_input.goal.object_hard_margin_m = 0.1;
  planning::StartPullOutStopObject candidate_object;
  candidate_object.pose.x = goal_centerline[center_index].x;
  candidate_object.pose.y = goal_centerline[center_index].y;
  candidate_input.perception.objects.push_back(candidate_object);
  const auto candidate_pipeline = map_pipeline.onRouteFrame(
      route_frame, direct_route, {}, {}, {}, std::nullopt,
      candidate_input);
  check(candidate_pipeline.behavior_candidates &&
            candidate_pipeline.behavior_candidates->perception_object_count == 1 &&
            candidate_pipeline.behavior_candidates->goal_safety ==
                planning::BehaviorGoalSafetyReason::FootprintObjectCollision,
        "출발·목표 후보에 같은 콜백의 인식 객체를 전달합니다");
  route_frame.ego = goal_reference.points[center_index];
  candidate_input.perception.objects.clear();
  const auto start_pipeline = map_pipeline.onRouteFrame(
      route_frame, direct_route, {}, {}, {}, std::nullopt,
      candidate_input);
  check(start_pipeline.behavior_candidates &&
            !start_pipeline.behavior_candidates->start_candidates.empty() &&
            std::hypot(
                start_pipeline.behavior_candidates->start_candidates.front().x -
                    route_frame.ego.x,
                start_pipeline.behavior_candidates->start_candidates.front().y -
                    route_frame.ego.y) < 1.0,
        "출발 자세 검색에 현재 자차의 세계 좌표를 전달합니다");
  planning::BehaviorRouteActionInput start_action;
  start_action.start = planning::BehaviorRouteStartRequest{};
  start_action.start->vehicle = small_vehicle;
  const auto &offset_point = goal_centerline[center_index];
  const auto &offset_next = goal_centerline[center_index + 1];
  const double offset_yaw = std::atan2(
      offset_next.y - offset_point.y,
      offset_next.x - offset_point.x);
  route_frame.ego.x = offset_point.x - 0.2 * std::sin(offset_yaw);
  route_frame.ego.y = offset_point.y + 0.2 * std::cos(offset_yaw);
  const auto action_pipeline = map_pipeline.onRouteFrame(
      route_frame, direct_route, {}, {}, {}, std::nullopt,
      std::nullopt, 5.0, start_action);
  check(action_pipeline.behavior_action &&
            action_pipeline.behavior_action->start_candidate_count > 0 &&
            action_pipeline.pipeline.status ==
                planning::PlanningPipelineStatus::Complete,
        "실측 지도와 자차 측방 위치에서 출발 횡이동 후보를 종단 호출합니다");
  candidate_input.perception.objects.clear();
  const auto safe_start_pipeline = map_pipeline.onRouteFrame(
      route_frame, direct_route, {}, {}, {}, std::nullopt,
      candidate_input, 5.0, start_action);
  check(safe_start_pipeline.behavior_candidates &&
            !safe_start_pipeline.behavior_candidates->start_candidates.empty() &&
            safe_start_pipeline.behavior_action &&
            safe_start_pipeline.behavior_action->start_candidate_count > 0,
        "안전한 현재 출발 자세는 같은 콜백의 후보 계산 후 경로 생성에 전달합니다");
  candidate_object.pose.x = route_frame.ego.x;
  candidate_object.pose.y = route_frame.ego.y;
  candidate_object.shape.length_m = 5.0;
  candidate_object.shape.width_m = 2.0;
  candidate_input.perception.objects.push_back(candidate_object);
  const auto blocked_start_pipeline = map_pipeline.onRouteFrame(
      route_frame, direct_route, {}, {}, {}, std::nullopt,
      candidate_input, 5.0, start_action);
  check(blocked_start_pipeline.behavior_candidates &&
            blocked_start_pipeline.behavior_candidates->start_candidates.empty() &&
            blocked_start_pipeline.behavior_action &&
            blocked_start_pipeline.behavior_action->kind ==
                planning::BehaviorRouteActionKind::LaneFollow &&
            blocked_start_pipeline.behavior_action->start_candidate_count == 0,
        "동일 콜백의 정지 객체가 출발 자세를 막으면 출발 경로를 선택하지 않습니다");
  planning::PlanningCruisePerceptionFrame start_perception;
  start_perception.stamp_s = route_frame.timestamp_s;
  planning::PlanningPerceivedObject start_blocker;
  start_blocker.id = "출발 방해 객체";
  start_blocker.x = route_frame.ego.x;
  start_blocker.y = route_frame.ego.y;
  start_blocker.length_m = 5.0;
  start_blocker.width_m = 2.0;
  start_perception.objects.push_back(start_blocker);
  const auto shared_start_pipeline = map_pipeline.onRouteFrame(
      route_frame, direct_route, {}, start_perception, {}, std::nullopt,
      std::nullopt, 5.0, start_action);
  check(shared_start_pipeline.behavior_candidates &&
            shared_start_pipeline.behavior_candidates->perception_object_count == 1 &&
            shared_start_pipeline.behavior_candidates->start_candidates.empty() &&
            shared_start_pipeline.behavior_action &&
            shared_start_pipeline.behavior_action->start_candidate_count == 0,
        "별도 후보 입력 없이 같은 호출의 인식 객체로 출발을 차단합니다");
  auto stale_start_frame = route_frame;
  stale_start_frame.timestamp_s = 5.0;
  planning::PlanningPipelineMapAdapter stale_start_adapter(
      readMap(argv[1]), {});
  const auto stale_start_pipeline = stale_start_adapter.onRouteFrame(
      stale_start_frame, direct_route, {}, start_perception, {}, std::nullopt,
      std::nullopt, 5.0, start_action);
  check(stale_start_pipeline.behavior_action &&
            stale_start_pipeline.behavior_action->start_candidate_count == 0,
        "2초 넘게 지난 인식 객체로 출발 후보를 만들지 않습니다");
  start_perception.stamp_s = 6.0;
  planning::PlanningPipelineMapAdapter future_start_adapter(
      readMap(argv[1]), {});
  const auto future_start_pipeline = future_start_adapter.onRouteFrame(
      stale_start_frame, direct_route, {}, start_perception, {}, std::nullopt,
      std::nullopt, 5.0, start_action);
  check(future_start_pipeline.behavior_action &&
            future_start_pipeline.behavior_action->start_candidate_count == 0,
        "현재 콜백보다 미래의 인식 시각으로 출발 후보를 만들지 않습니다");
  start_perception.stamp_s = 1.0;
  planning::CruisePredictedPath departing_path;
  departing_path.time_step_s = 1.0;
  departing_path.confidence = 1.0;
  departing_path.poses = {
      {start_blocker.x, start_blocker.y, 0.0},
      {start_blocker.x + 30.0, start_blocker.y, 0.0}};
  start_perception.objects.front().predicted_paths = {departing_path};
  auto predicted_start_frame = route_frame;
  predicted_start_frame.timestamp_s = 2.0;
  planning::PlanningPipelineMapAdapter predicted_start_adapter(
      readMap(argv[1]), {});
  const auto predicted_start_pipeline =
      predicted_start_adapter.onRouteFrame(
          predicted_start_frame, direct_route, {}, start_perception, {},
          std::nullopt, std::nullopt, 5.0, start_action);
  check(predicted_start_pipeline.behavior_candidates &&
            !predicted_start_pipeline.behavior_candidates->start_candidates.empty() &&
            predicted_start_pipeline.behavior_action &&
            predicted_start_pipeline.behavior_action->start_candidate_count > 0,
        "현재 시각에 떠난 객체의 예측 자세로 출발 후보를 계산합니다");
  route_frame.ego = goal_reference.points.front();
  planning::PlanningPipelinePointcloudInput cloud;
  cloud.cruise_stop_enabled = true;
  cloud.cruise_slowdown_enabled = true;
  cloud.cruise_points.push_back({goal_centerline[center_index].x,
                                 goal_centerline[center_index].y, 0.0});
  const auto cloud_pipeline = map_pipeline.onFrame(
      map_frame, {}, {}, cloud);
  check(cloud_pipeline.pipeline.status ==
            planning::PlanningPipelineStatus::Complete &&
            cloud_pipeline.pipeline.cruise_candidate_count > 0 &&
            cloud_pipeline.cruise_slowdown_candidate_count == 0,
        "같은 점군 객체는 정지 후보로 선택한 뒤 감속에서 제외합니다");
  planning::PlanningCruisePerceptionFrame slowdown_perception;
  slowdown_perception.slowdown_enabled = true;
  planning::PlanningPerceivedObject slowdown_object;
  slowdown_object.id = "지도 감속 객체";
  slowdown_object.classification = 1;
  slowdown_object.length_m = 2.0;
  slowdown_object.width_m = 1.0;
  const auto &nearby_before = goal_centerline[center_index - 1];
  const auto &nearby_after = goal_centerline[center_index + 1];
  const double nearby_yaw = std::atan2(
      nearby_after.y - nearby_before.y,
      nearby_after.x - nearby_before.x);
  slowdown_object.x = goal_centerline[center_index].x -
                      2.2 * std::sin(nearby_yaw);
  slowdown_object.y = goal_centerline[center_index].y +
                      2.2 * std::cos(nearby_yaw);
  slowdown_object.yaw_rad = nearby_yaw;
  slowdown_perception.objects.push_back(slowdown_object);
  for (int callback = 0; callback < 5; ++callback) {
    const auto slowdown_pipeline = map_pipeline.onFrame(
        map_frame, {}, slowdown_perception);
    check(slowdown_pipeline.pipeline.status ==
              planning::PlanningPipelineStatus::Complete &&
              (slowdown_pipeline.cruise_slowdown_candidate_count > 0) ==
                  (callback == 4),
          "원본처럼 예측 객체가 5회 연속 진입한 뒤 순항 감속 후보가 됩니다");
  }
  planning::BehaviorVelocityMapPlanner occlusion_planner(
      readMap(argv[1]), {planning::BehaviorVelocityModule::OcclusionSpot});
  planning::BehaviorVelocityMapInput occlusion_input;
  for (int index = 0; index <= 40; ++index) {
    planning::TrajectoryPoint point;
    point.x = index;
    point.longitudinal_velocity_mps = 8.0F;
    occlusion_input.trajectory.push_back(point);
    occlusion_input.lane_ids.push_back({24747});
  }
  occlusion_input.ego_velocity_mps = 8.0;
  occlusion_input.occlusion_spot_parameters.method =
      planning::OcclusionSpotDetectionMethod::StoppedObjects;
  planning::CrosswalkOcclusionMaskObject occlusion_vehicle;
  occlusion_vehicle.pose = {20.0, 3.0, 0.0};
  occlusion_vehicle.length_m = 4.0;
  occlusion_vehicle.width_m = 2.0;
  occlusion_input.occlusion_spot_vehicles.push_back(occlusion_vehicle);
  const auto occlusion_plan = occlusion_planner.apply(occlusion_input);
  check(!occlusion_plan.occlusion_spot_collision_arcs_m.empty(),
        "지도 속도 계획기가 일반 가림 영역 모듈을 실행합니다");
  check(occlusion_planner.unloadModule(
            planning::BehaviorVelocityModule::OcclusionSpot) &&
            !occlusion_planner.unloadModule(
                planning::BehaviorVelocityModule::OcclusionSpot) &&
            occlusion_planner.apply(occlusion_input)
                .occlusion_spot_collision_arcs_m.empty(),
        "가림 영역 plugin 해제 뒤에는 계산과 보유 상태가 제거됩니다");
  check(occlusion_planner.loadModule(
            planning::BehaviorVelocityModule::OcclusionSpot) &&
            !occlusion_planner.loadModule(
                planning::BehaviorVelocityModule::OcclusionSpot) &&
            !occlusion_planner.apply(occlusion_input)
                 .occlusion_spot_collision_arcs_m.empty(),
        "가림 영역 plugin 재적재는 중복 없이 지도 계산을 복구합니다");
  planning::PlanningPipelineMapAdapter plugin_pipeline(
      readMap(argv[1]), {planning::BehaviorVelocityModule::OcclusionSpot});
  check(planning::behaviorVelocityLoadClass(
            "autoware::behavior_velocity_planner::StopLineModulePlugin") ==
            planning::BehaviorVelocityModule::StopSign &&
            planning::behaviorVelocityUnloadName("stop_line") ==
            planning::BehaviorVelocityModule::StopSign &&
            !planning::behaviorVelocityLoadClass("stop_line") &&
            !planning::behaviorVelocityUnloadName(
                "autoware::behavior_velocity_planner::StopLineModulePlugin") &&
            !planning::behaviorVelocityLoadClass(
                "autoware::behavior_velocity_planner::IntersectionModulePlugin"),
        "원본 적재 class 이름과 해제 모듈 이름을 분리하고 복합 plugin은 어댑터에서 처리합니다");
  check(plugin_pipeline.unloadBehaviorVelocityPlugin("occlusion_spot") &&
            !plugin_pipeline.unloadBehaviorVelocityPlugin("occlusion_spot"),
        "종단 어댑터의 plugin 해제 요청을 지도 계획기에 전달합니다");
  plugin_pipeline.reset();
  check(plugin_pipeline.loadBehaviorVelocityPlugin(
            "autoware::behavior_velocity_planner::OcclusionSpotModulePlugin"),
        "reset 뒤에도 해제한 plugin 구성이 유지됩니다");
  plugin_pipeline.reset();
  check(!plugin_pipeline.loadBehaviorVelocityPlugin(
            "autoware::behavior_velocity_planner::OcclusionSpotModulePlugin") &&
            !plugin_pipeline.loadBehaviorVelocityPlugin("occlusion_spot"),
        "reset 뒤에도 다시 적재한 plugin의 중복을 거부합니다");
  planning::PlanningPipelineMapAdapter intersection_plugin_pipeline(
      readMap(argv[1]), {});
  constexpr auto intersection_class =
      "autoware::behavior_velocity_planner::IntersectionModulePlugin";
  check(intersection_plugin_pipeline.loadBehaviorVelocityPlugin(
            intersection_class) &&
            !intersection_plugin_pipeline.loadBehaviorVelocityPlugin(
                intersection_class),
        "교차로 plugin의 두 계산 모듈은 하나의 요청으로 적재하고 중복을 거부합니다");
  intersection_plugin_pipeline.reset();
  check(intersection_plugin_pipeline.unloadBehaviorVelocityPlugin(
            "intersection") &&
            !intersection_plugin_pipeline.unloadBehaviorVelocityPlugin(
                "intersection") &&
            intersection_plugin_pipeline.loadBehaviorVelocityPlugin(
                intersection_class),
        "교차로 plugin의 두 상태는 재시작 뒤에도 함께 해제·재적재합니다");
  using MotionModule = planning::MotionVelocityModule;
  constexpr auto out_of_lane_class =
      "autoware::motion_velocity_planner::OutOfLaneModule";
  constexpr auto limiter_class =
      "autoware::motion_velocity_planner::ObstacleVelocityLimiterModule";
  constexpr auto dynamic_stop_class =
      "autoware::motion_velocity_planner::DynamicObstacleStopModule";
  check(plugin_pipeline.motionVelocityModules() ==
            std::vector<MotionModule>{MotionModule::OutOfLane,
                                      MotionModule::ObstacleVelocityLimiter,
                                      MotionModule::DynamicObstacleStop} &&
            !plugin_pipeline.loadMotionVelocityPlugin(out_of_lane_class) &&
            !plugin_pipeline.unloadMotionVelocityPlugin("out_of_lane"),
        "동작 속도 기본 적재 순서와 전체 class 이름 계약을 유지합니다");
  check(plugin_pipeline.unloadMotionVelocityPlugin(out_of_lane_class) &&
            !plugin_pipeline.unloadMotionVelocityPlugin(out_of_lane_class) &&
            plugin_pipeline.loadMotionVelocityPlugin(out_of_lane_class) &&
            plugin_pipeline.motionVelocityModules() ==
                std::vector<MotionModule>{
                    MotionModule::ObstacleVelocityLimiter,
                    MotionModule::DynamicObstacleStop,
                    MotionModule::OutOfLane},
        "동작 속도 모듈은 재적재한 순서대로 실행 목록에 추가됩니다");
  plugin_pipeline.reset();
  check(plugin_pipeline.motionVelocityModules() ==
            std::vector<MotionModule>{
                MotionModule::ObstacleVelocityLimiter,
                MotionModule::DynamicObstacleStop,
                MotionModule::OutOfLane} &&
            plugin_pipeline.unloadMotionVelocityPlugin(limiter_class) &&
            plugin_pipeline.unloadMotionVelocityPlugin(dynamic_stop_class) &&
            plugin_pipeline.motionVelocityModules() ==
                std::vector<MotionModule>{MotionModule::OutOfLane} &&
            !plugin_pipeline.loadMotionVelocityPlugin("dynamic_obstacle_stop"),
        "reset 뒤에도 동작 속도 적재 구성을 유지하고 알려진 class만 허용합니다");
  const auto &goal_point = goal_centerline[goal_centerline.size() - 2];
  const auto &goal_next = goal_centerline.back();
  const double goal_yaw = std::atan2(
      goal_next.y - goal_point.y, goal_next.x - goal_point.x);
  const auto fixed_goal = original.refineFixedGoalPath(
      goal_reference, {goal_point.x, goal_point.y, goal_yaw}, 24747);
  check(!fixed_goal.points.empty() &&
            std::hypot(fixed_goal.points.back().x - goal_point.x,
                       fixed_goal.points.back().y - goal_point.y) < 1.0e-6 &&
            fixed_goal.points.back().longitudinal_velocity_mps == 0.0F,
        "실측 차로 안에서 고정 목표점을 영속도 종점으로 연결합니다");
  const auto outside_goal = original.refineFixedGoalPath(
      goal_reference,
      {goal_point.x - 5.0 * std::sin(goal_yaw),
       goal_point.y + 5.0 * std::cos(goal_yaw), goal_yaw}, 24747);
  check(!outside_goal.points.empty() &&
            std::hypot(outside_goal.points.back().x - goal_point.x,
                       outside_goal.points.back().y - goal_point.y) < 0.5,
        "차로 밖의 목표를 실측 중심선 가까이 투영합니다");
  check(original.stopSignStopLinesForLanelet(24747).empty(),
        "실측 지도 차로 24747에는 정지 표지가 없습니다");
  const auto crosswalk = original.crosswalkBounds(24783);
  const auto intersection_ids = original.intersectionLaneletIds();
  std::size_t blind_spot_region_count = 0;
  std::size_t blind_spot_auto_stop_count = 0;
  bool blind_spot_object_checked = false;
  for (const auto lane_id : intersection_ids) {
    const auto direction = original.laneTurnDirection(lane_id);
    if (direction != planning::LaneTurnDirection::Left &&
        direction != planning::LaneTurnDirection::Right) continue;
    const auto region = original.blindSpotRegion(lane_id);
    check(region && region->polygon.size() >= 4 &&
              region->centerline.size() >= 2,
          "실측 회전 차로의 사각지대 감지 띠를 만듭니다");
    planning::Trajectory turn_path;
    for (std::size_t index = 0; index < region->centerline.size(); ++index) {
      planning::TrajectoryPoint point;
      point.x = region->centerline[index].x;
      point.y = region->centerline[index].y;
      const auto next = std::min(index + 1, region->centerline.size() - 1);
      const auto previous = index == next ? index - 1 : index;
      const double yaw = std::atan2(
          region->centerline[next].y - region->centerline[previous].y,
          region->centerline[next].x - region->centerline[previous].x);
      point.orientation_z = std::sin(yaw / 2.0);
      point.orientation_w = std::cos(yaw / 2.0);
      turn_path.push_back(point);
    }
    const auto stop_arcs = original.blindSpotStopArcs(lane_id, turn_path);
    if (stop_arcs) {
      check(stop_arcs->default_stop_arc_m <=
                stop_arcs->critical_stop_arc_m,
            "사각지대 기본 정지점은 긴급 정지점보다 앞섭니다");
      ++blind_spot_auto_stop_count;
    }
    {
      using Point = boost::geometry::model::d2::point_xy<double>;
      boost::geometry::model::polygon<Point> polygon;
      for (const auto &point : region->polygon) {
        boost::geometry::append(polygon.outer(), Point(point.x, point.y));
      }
      boost::geometry::correct(polygon);
      check(boost::geometry::is_valid(polygon),
            "실측 회전 차로의 사각지대 감지 띠가 유효합니다");
    }
    if (!blind_spot_object_checked) {
      using Point = boost::geometry::model::d2::point_xy<double>;
      boost::geometry::model::polygon<Point> polygon;
      for (const auto &point : region->polygon) {
        boost::geometry::append(polygon.outer(), Point(point.x, point.y));
      }
      boost::geometry::correct(polygon);
      Point center(0.0, 0.0);
      boost::geometry::point_on_surface(polygon, center);
      const auto mapped = original.mapBlindSpotObjects(
          {lane_id}, direction, region->centerline.front(),
          {{"자전거", 6, {center.x(), center.y()}, 2.0}}, 100000.0);
      check(mapped.stop_arc_m > 0.0 && mapped.objects.size() == 1 &&
                mapped.objects.front().inside_detection_area,
            "실측 감지 띠 내부 객체의 호길이와 포함 여부를 계산합니다");
      const auto earlier_stop = original.mapBlindSpotObjects(
          {lane_id}, direction, region->centerline.front(), {}, 100000.0,
          region->centerline[region->centerline.size() / 2]);
      check(earlier_stop.stop_arc_m > 0.0 &&
                earlier_stop.stop_arc_m < mapped.stop_arc_m,
            "긴급 정지점 자세를 감지 띠 호길이에 투영합니다");
      blind_spot_object_checked = true;
    }
    ++blind_spot_region_count;
  }
  check(blind_spot_region_count > 0,
        "실측 지도에 사각지대 감지 띠를 검사할 회전 차로가 있습니다");
  check(blind_spot_auto_stop_count > 0,
        "실측 회전 차로에서 사각지대 정지점을 자동으로 찾습니다");
  check(std::find(intersection_ids.begin(), intersection_ids.end(), 25001) !=
            intersection_ids.end() &&
            std::find(intersection_ids.begin(), intersection_ids.end(), 24739) !=
                intersection_ids.end(),
        "실측 이진 지도에서 교차로 차로를 찾습니다");
  const auto conflicting = original.intersectionConflictingLaneletIds(25001);
  const auto second_turn_conflicts =
      original.intersectionConflictingLaneletIds(24739);
  check(std::set<std::int64_t>(second_turn_conflicts.begin(),
                               second_turn_conflicts.end()) ==
            std::set<std::int64_t>{24938, 24906},
        "뒤쪽 좌회전 차로의 충돌 차로를 찾습니다");
  check(std::set<std::int64_t>(conflicting.begin(), conflicting.end()) ==
            std::set<std::int64_t>{24868, 24869, 24897},
        "원본 지도 라우팅 그래프와 같은 교차 차로를 찾습니다");
  const auto conflicting_bounds = original.laneBounds(24897);
  check(conflicting_bounds.left.size() >= 2 &&
            conflicting_bounds.right.size() >= 2,
        "교차 차로의 실제 경계를 읽습니다");
  const auto other_lanes = original.outOfLaneRoadLaneletIds(
      original.laneCenterline(25001));
  check(!other_lanes.empty() &&
            std::is_sorted(other_lanes.begin(), other_lanes.end()) &&
            std::find(other_lanes.begin(), other_lanes.end(), 25001) ==
                other_lanes.end(),
        "차로 밖 주행 후보에서 자차 도로 차로를 제외합니다");
  planning::Trajectory out_of_lane_path;
  for (const auto &point : original.laneCenterline(25001)) {
    planning::TrajectoryPoint trajectory_point;
    trajectory_point.x = point.x;
    trajectory_point.y = point.y;
    out_of_lane_path.push_back(trajectory_point);
  }
  const auto other_polygons = original.outOfLaneRoadPolygons(
      out_of_lane_path);
  check(other_polygons.size() == other_lanes.size() &&
            std::all_of(other_polygons.begin(), other_polygons.end(),
                        [](const auto &polygon) { return polygon.size() >= 3; }),
        "차로 밖 주행 후보 차로의 실제 다각형을 읽습니다");
  std::vector<double> out_of_lane_times(out_of_lane_path.size(), 0.0);
  check(!original.findOutOfLaneMapCandidate(
             out_of_lane_path, out_of_lane_times, {}, 0.0),
        "예측 객체가 없으면 지도 후보에서 정지 요청을 만들지 않습니다");
  const auto divisions = original.intersectionOcclusionDivisions(25001, 0.5);
  check(original.isVehicleOnIntersectionAttentionLane(
            25001, 2589.7, 24040.0, -1.5707963267948966,
            0.0, 3.141592653589793),
        "북쪽 주목 차로의 정차 차량을 찾습니다");
  check(!divisions.empty() &&
            std::all_of(divisions.begin(), divisions.end(),
                        [](const auto &division) { return division.size() >= 2; }),
        "교차 차로와 선행 차로를 잇는 가림 탐색선을 생성합니다");
  check(original.isVehicleOnIntersectionConflictLane(
            25001, 2614.8, 24007.4, 3.141592653589793),
        "서쪽 진행 차량이 충돌 대상 차로에 속합니다");
  check(!original.isVehicleOnIntersectionConflictLane(
            25001, 2596.0, 23981.0, 1.5707963267948966),
        "자차 접근 차로의 차량은 충돌 대상에서 제외합니다");
  check(crosswalk.left.size() >= 2 && crosswalk.right.size() >= 2 &&
            std::isfinite(crosswalk.left.front().x) &&
            std::isfinite(crosswalk.right.front().y),
        "실측 이진 지도에서 횡단보도 경계를 읽습니다");
  const planning::CrosswalkPoint start{
      0.5 * (crosswalk.left[0].x + crosswalk.right[0].x),
      0.5 * (crosswalk.left[0].y + crosswalk.right[0].y)};
  const planning::CrosswalkPoint end{
      0.5 * (crosswalk.left[1].x + crosswalk.right[1].x),
      0.5 * (crosswalk.left[1].y + crosswalk.right[1].y)};
  const double yaw = std::atan2(end.y - start.y, end.x - start.x);
  const auto scenario = original.scenarioPosition(
      start.x, start.y, yaw, end.x, end.y, {24783});
  check(scenario.in_lane && scenario.goal_in_lane &&
            scenario.along_lane && !scenario.in_parking_lot,
        "이진 지도에서 경로 차로와 방향을 판정합니다");
  const auto opposite = original.scenarioPosition(
      start.x, start.y, yaw + 3.141592653589793,
      end.x, end.y, {24783});
  check(!opposite.along_lane, "반대 방향 차로 정렬을 제외합니다");
  const auto outside = original.scenarioPosition(
      0.0, 0.0, 0.0, 0.0, 0.0, {24783});
  check(!outside.in_lane && !outside.goal_in_lane &&
            !outside.in_parking_lot && !outside.along_lane,
        "지도 밖 좌표는 차로·주차장 판정을 거부합니다");

  const auto original_bytes = readMap(argv[1]);
  const std::string original_string(original_bytes.begin(), original_bytes.end());
  std::istringstream input_stream(original_string, std::ios::in | std::ios::binary);
  lanelet::LaneletMap parking_map;
  lanelet::Id next_id = 0;
  {
    boost::archive::binary_iarchive archive(input_stream);
    archive >> parking_map;
    archive >> next_id;
  }
  lanelet::Polygon3d parking_lot(900000020, {
      lanelet::Point3d(900000021, 0.0, 0.0, 0.0),
      lanelet::Point3d(900000022, 2.0, 0.0, 0.0),
      lanelet::Point3d(900000023, 2.0, 2.0, 0.0),
      lanelet::Point3d(900000024, 0.0, 2.0, 0.0)});
  parking_lot.setAttribute("type", "parking_lot");
  parking_map.add(parking_lot);
  lanelet::LineString3d guard_rail(900000300, {
      lanelet::Point3d(900000301, 5000.0, 5000.0, 0.0),
      lanelet::Point3d(900000302, 5010.0, 5000.0, 0.0)});
  guard_rail.setAttribute("type", "guard_rail");
  parking_map.add(guard_rail);
  lanelet::LineString3d other_left(900000401, {
      lanelet::Point3d(900000402, 5005.0, 5.0, 0.0),
      lanelet::Point3d(900000403, 5030.0, 5.0, 0.0)});
  lanelet::LineString3d other_right(900000404, {
      lanelet::Point3d(900000405, 5005.0, 2.0, 0.0),
      lanelet::Point3d(900000406, 5030.0, 2.0, 0.0)});
  lanelet::Lanelet other_road(900000407, other_left, other_right);
  other_road.setAttribute("subtype", "road");
  parking_map.add(other_road);
  std::ostringstream output_stream(std::ios::out | std::ios::binary);
  {
    boost::archive::binary_oarchive archive(output_stream);
    archive << parking_map;
    archive << next_id;
  }
  const auto parking_string = output_stream.str();
  const std::vector<std::uint8_t> parking_bytes(
      parking_string.begin(), parking_string.end());
  planning::LaneletBinAdapter parking_adapter(parking_bytes);
  std::string route_failure;
  const auto off_center_goal = parking_adapter.routeBetweenCheckpoints(
      {5010.0, 3.5, 0.0}, {5020.0, 2.1, 0.0}, false, &route_failure);
  check(!off_center_goal &&
            route_failure.find("Goal footprint exceeds") != std::string::npos,
        "차량 외곽이 차로를 벗어나면 구체적인 거절 사유를 반환합니다");
  const auto centered_goal = parking_adapter.routeBetweenCheckpoints(
      {5010.0, 3.5, 0.0}, {5020.0, 3.5, 0.0}, false, &route_failure);
  check(centered_goal && route_failure.empty(),
        "유효한 목적지는 경로를 생성하고 이전 실패 사유를 지웁니다");
  lanelet::LineString3d shoulder_right(900000408, {
      lanelet::Point3d(900000409, 5005.0, 0.0, 0.0),
      lanelet::Point3d(900000410, 5030.0, 0.0, 0.0)});
  lanelet::Lanelet right_shoulder(900000411, other_right, shoulder_right);
  right_shoulder.setAttribute("subtype", "road_shoulder");
  parking_map.add(right_shoulder);
  std::ostringstream shoulder_stream(std::ios::out | std::ios::binary);
  {
    boost::archive::binary_oarchive archive(shoulder_stream);
    archive << parking_map;
    archive << next_id;
  }
  const auto shoulder_string = shoulder_stream.str();
  const std::vector<std::uint8_t> shoulder_bytes(
      shoulder_string.begin(), shoulder_string.end());
  planning::LaneletBinAdapter shoulder_adapter(shoulder_bytes);
  const auto shoulder_goal = shoulder_adapter.routeBetweenCheckpoints(
      {5010.0, 3.5, 0.0}, {5020.0, 2.1, 0.0}, false, &route_failure);
  check(shoulder_goal && route_failure.empty(),
        "경로 차로와 경계를 공유하는 갓길은 목적지 윤곽 검사에 포함합니다");
  planning::Trajectory out_of_lane_synthetic(31);
  std::vector<double> synthetic_times(31);
  for (std::size_t i = 0; i < out_of_lane_synthetic.size(); ++i) {
    out_of_lane_synthetic[i].x = 5000.0 + i;
    out_of_lane_synthetic[i].y = std::min(1.5, 0.15 * i);
    synthetic_times[i] = 0.1 * i;
  }
  planning::OutOfLanePredictedPath crossing_object;
  crossing_object.time_step = 0.5;
  crossing_object.object_length = 4.0;
  crossing_object.object_width = 2.0;
  planning::TrajectoryPoint crossing_pose;
  crossing_pose.x = 5012.0;
  crossing_pose.y = 2.5;
  crossing_object.poses.push_back(crossing_pose);
  crossing_pose.x = 5012.5;
  crossing_object.poses.push_back(crossing_pose);
  planning::OutOfLaneTrackedObject tracked_crossing;
  tracked_crossing.initial_position = {5012.0, 2.5};
  tracked_crossing.width = 2.0;
  tracked_crossing.predicted_paths = {crossing_object};
  const auto filtered_crossing = planning::filterOutOfLanePredictedObjects(
      out_of_lane_synthetic, {5000.0, 0.0},
      {tracked_crossing}, {});
  check(filtered_crossing.size() == 1,
        "합성 예측 차량이 차로 밖 주행 객체 필터를 통과합니다");
  const auto map_candidate = parking_adapter.findOutOfLaneMapCandidate(
      out_of_lane_synthetic, synthetic_times, filtered_crossing, 0.0);
  check(map_candidate && map_candidate->x >= 5000.0 &&
            map_candidate->x < 5010.0,
        "합성 도로 차로와 예측 객체를 연결해 차로 밖 주행 정지 후보를 찾습니다");
  const auto integrated_candidate =
      parking_adapter.findOutOfLaneMapCandidateFromTrackedObjects(
          out_of_lane_synthetic, synthetic_times, {5000.0, 0.0},
          {tracked_crossing}, {}, 100.0, 0.0);
  check(integrated_candidate &&
            near(integrated_candidate->x, map_candidate->x) &&
            near(integrated_candidate->y, map_candidate->y),
        "지도·객체 필터·차로 겹침을 한 번의 후보 계산에 연결합니다");
  planning::OutOfLaneDecisionCore out_of_lane_decision;
  const auto out_of_lane_plan = out_of_lane_decision.plan(
      out_of_lane_synthetic, {5000.0, 0.0}, map_candidate, 0.0);
  check(out_of_lane_plan.stop_points.size() == 1 &&
            out_of_lane_plan.slowdowns.empty(),
        "합성 지도 후보를 실제 정지 계획 요청으로 연결합니다");
  planning::PlanningPipelineMapAdapter out_of_lane_pipeline(parking_bytes, {});
  planning::PlanningOutOfLaneInput out_of_lane_snapshot;
  out_of_lane_snapshot.time_from_start_s = synthetic_times;
  out_of_lane_snapshot.objects = {tracked_crossing};
  out_of_lane_snapshot.minimum_stop_arc_m = 0.0;
  planning::TrajectoryPoint out_of_lane_ego;
  out_of_lane_ego.x = 5000.0;
  const auto integrated_plan = out_of_lane_pipeline.planOutOfLane(
      out_of_lane_synthetic, out_of_lane_ego, 0.0, out_of_lane_snapshot);
  check(integrated_plan.stop_points.size() == 1 &&
            near(integrated_plan.stop_points.front().x,
                 out_of_lane_plan.stop_points.front().x),
        "동작 속도 콜백 입력으로 지도 차로 밖 주행 후보와 정지 계획을 연결합니다");
  out_of_lane_pipeline.reset();
  const auto replayed_plan = out_of_lane_pipeline.planOutOfLane(
      out_of_lane_synthetic, out_of_lane_ego, 2.0, out_of_lane_snapshot);
  check(replayed_plan.stop_points.size() == 1 &&
            near(replayed_plan.stop_points.front().x,
                 integrated_plan.stop_points.front().x),
        "route 파이프라인 재시작 뒤 차로 밖 주행 상태도 초기화합니다");
  out_of_lane_snapshot.time_from_start_s.clear();
  const auto calculated_time_plan = out_of_lane_pipeline.planOutOfLane(
      out_of_lane_synthetic, out_of_lane_ego, 2.1, out_of_lane_snapshot);
  std::vector<double> source_times(out_of_lane_synthetic.size());
  for (std::size_t index = 1; index < source_times.size(); ++index)
    source_times[index] = static_cast<double>(index);
  const auto source_time_candidate =
      parking_adapter.findOutOfLaneMapCandidateFromTrackedObjects(
          out_of_lane_synthetic, source_times, {5000.0, 0.0},
          {tracked_crossing}, {}, 100.0, 0.0);
  check(!source_time_candidate == calculated_time_plan.stop_points.empty(),
        "자동 도착시각은 원본 1m/s 하한으로 계산한 후보 활성과 같습니다");
  out_of_lane_snapshot.minimum_stop_arc_m.reset();
  bool missing_braking_arc_rejected = false;
  try {
    (void)out_of_lane_pipeline.planOutOfLane(
        out_of_lane_synthetic, out_of_lane_ego, 2.2,
        out_of_lane_snapshot);
  } catch (const std::invalid_argument &) {
    missing_braking_arc_rejected = true;
  }
  check(missing_braking_arc_rejected,
        "원본 최소 제동 호길이가 없으면 안전 분기를 임의의 0m로 대체하지 않습니다");
  out_of_lane_pipeline.reset();
  out_of_lane_snapshot.time_from_start_s = synthetic_times;
  out_of_lane_snapshot.braking =
      planning::PlanningOutOfLaneBrakingInput{0.0, 0.0, -2.5, -1.5};
  const auto calculated_braking_plan = out_of_lane_pipeline.planOutOfLane(
      out_of_lane_synthetic, out_of_lane_ego, 3.0, out_of_lane_snapshot);
  check(calculated_braking_plan.stop_points.size() == 1 &&
            near(calculated_braking_plan.stop_points.front().x,
                 integrated_plan.stop_points.front().x),
        "콜백 속도·가속도·평활화 제한에서 최소 정지 호길이를 계산합니다");
  out_of_lane_pipeline.reset();
  out_of_lane_snapshot.braking->ego_velocity_mps = 16.7;
  const auto long_braking_plan = out_of_lane_pipeline.planOutOfLane(
      out_of_lane_synthetic, out_of_lane_ego, 4.0, out_of_lane_snapshot);
  check(long_braking_plan.stop_points.empty() &&
            long_braking_plan.slowdowns.empty(),
        "원본 저크 제약의 긴 제동 거리 앞에는 불가능한 정지점을 넣지 않습니다");
  const auto guard_segments = parking_adapter.staticObstacleSegments(
      4999.0, 4999.0, 5011.0, 5001.0);
  check(guard_segments.size() == 1 &&
            near(guard_segments.front().from.x, 5000.0) &&
            near(guard_segments.front().to.x, 5010.0),
        "정적 가드레일 선분을 이진 지도에서 읽습니다");
  check(parking_adapter.staticObstacleSegments(
            4999.0, 4999.0, 5011.0, 5001.0, {"wall"}).empty(),
        "장애물 지도 태그를 적용합니다");
  planning::Trajectory limiter_path(3);
  for (std::size_t i = 0; i < limiter_path.size(); ++i) {
    limiter_path[i].x = 4990.0 + i * 5.0;
    limiter_path[i].y = 5000.0;
    limiter_path[i].longitudinal_velocity_mps = 10.0F;
  }
  const std::vector<double> limiter_times{0.0, 0.5, 1.0};
  const auto guard_plan = parking_adapter.planStaticObstacleVelocityLimit(
      limiter_path, limiter_times, 0, 0.0);
  check(guard_plan.slowdowns.size() == 1 &&
            near(guard_plan.slowdowns.front().velocity, 6.26),
        "지도 가드레일을 직선 투영 감속 계산에 연결합니다");
  check(parking_adapter.planStaticObstacleVelocityLimit(
            limiter_path, limiter_times, 0, 0.0, {}, {"wall"})
            .slowdowns.empty(),
        "선택하지 않은 지도 태그는 감속하지 않습니다");
  planning::ObstacleVelocityLimiterParameters bicycle_velocity;
  bicycle_velocity.model = planning::VelocityLimiterProjectionModel::Bicycle;
  check(parking_adapter.planStaticObstacleVelocityLimit(
            limiter_path, limiter_times, 0, 0.0,
            bicycle_velocity).slowdowns.size() == 1,
        "지도 가드레일을 자전거 전방 투영 계산에 연결합니다");
  const auto inside_parking = parking_adapter.scenarioPosition(
      1.0, 1.0, 0.0, 0.0, 0.0, {});
  check(inside_parking.in_parking_lot && !inside_parking.in_lane,
        "합성 주차장 다각형 안의 차량을 판정합니다");
  const auto border_parking = parking_adapter.scenarioPosition(
      0.0, 1.0, 0.0, 0.0, 0.0, {});
  check(!border_parking.in_parking_lot,
        "원본과 같이 주차장 경계점은 내부에서 제외합니다");

  lanelet::LineString3d start_line(900000210, {
      lanelet::Point3d(900000211, 2590.0, 23980.0, 0.0),
      lanelet::Point3d(900000212, 2600.0, 23980.0, 0.0)});
  lanelet::LineString3d stop_line(900000213, {
      lanelet::Point3d(900000214, 2590.0, 23982.0, 0.0),
      lanelet::Point3d(900000215, 2600.0, 23982.0, 0.0)});
  lanelet::LineString3d end_line(900000216, {
      lanelet::Point3d(900000217, 2590.0, 23990.0, 0.0),
      lanelet::Point3d(900000218, 2600.0, 23990.0, 0.0)});
  lanelet::LineString3d instrument(900000220, {
      lanelet::Point3d(900000221, 2590.0, 23981.0, 0.0),
      lanelet::Point3d(900000222, 2600.0, 23981.0, 0.0)});
  instrument.setAttribute("type", "virtual_traffic_light");
  instrument.setAttribute("zone", "시험구역");
  auto virtual_light =
      std::make_shared<lanelet::GenericRegulatoryElement>(900000219);
  virtual_light->setAttribute("subtype", "virtual_traffic_light");
  virtual_light->addParameter("start_line", start_line);
  virtual_light->addParameter("ref_line", stop_line);
  virtual_light->addParameter("end_line", end_line);
  virtual_light->addParameter("refers", instrument);
  parking_map.add(virtual_light);
  parking_map.laneletLayer.get(24747).addRegulatoryElement(virtual_light);
  std::ostringstream virtual_stream(std::ios::out | std::ios::binary);
  {
    boost::archive::binary_oarchive archive(virtual_stream);
    archive << parking_map;
    archive << next_id;
  }
  const auto virtual_string = virtual_stream.str();
  planning::LaneletBinAdapter virtual_adapter({
      virtual_string.begin(), virtual_string.end()});
  const auto virtual_features =
      virtual_adapter.virtualTrafficLightsForLanelet(24747);
  check(virtual_features.size() == 1 &&
            virtual_features[0].regulatory_id == 900000219 &&
            virtual_features[0].start_lines.size() == 1 &&
            virtual_features[0].stop_lines.size() == 1 &&
            virtual_features[0].end_lines.size() == 1 &&
            virtual_features[0].command_metadata.instrument_type ==
                "virtual_traffic_light" &&
            virtual_features[0].command_metadata.instrument_id == "900000220" &&
            virtual_features[0].command_metadata.custom_tags.size() == 3 &&
            near(virtual_features[0].start_lines[0].y1, 23980.0) &&
            near(virtual_features[0].stop_lines[0].y1, 23982.0) &&
            near(virtual_features[0].end_lines[0].y1, 23990.0),
        "합성 가상 신호등의 시작·정지·종료선을 이진 지도에서 읽습니다");
  const auto &command_tags = virtual_features[0].command_metadata.custom_tags;
  const auto has_tag = [&](const std::string &key, const std::string &value) {
    return std::find(command_tags.begin(), command_tags.end(),
                     std::pair<std::string, std::string>{key, value}) !=
           command_tags.end();
  };
  check(has_tag("zone", "시험구역") && has_tag("lane_id", "24747") &&
            has_tag("turn_direction", "straight"),
        "가상 신호등 명령에 지도 속성과 차로 태그를 보존합니다");
  planning::BehaviorVelocityMapPlanner virtual_planner(
      {virtual_string.begin(), virtual_string.end()},
      {planning::BehaviorVelocityModule::VirtualTrafficLight});
  planning::BehaviorVelocityMapInput virtual_input;
  virtual_input.now_s = 10.0;
  virtual_input.ego_x = 2595.0;
  virtual_input.ego_y = 23980.5;
  virtual_input.ego_velocity_mps = 2.0;
  virtual_input.virtual_traffic_light_front_offset_m = 0.0;
  for (int index = 0; index <= 30; ++index) {
    planning::TrajectoryPoint point;
    point.x = 2595.0;
    point.y = 23970.0 + index;
    point.longitudinal_velocity_mps = 5.0F;
    virtual_input.trajectory.push_back(point);
    virtual_input.lane_ids.push_back({24747});
  }
  const auto awaiting = virtual_planner.apply(virtual_input);
  check(awaiting.virtual_traffic_light_states.at(900000219).stop ==
            planning::VirtualTrafficLightStop::StopLine &&
            awaiting.virtual_traffic_light_commands.at(900000219).state == 1 &&
            near(awaiting.virtual_traffic_light_commands.at(900000219).stamp_s,
                 10.0) &&
            awaiting.trajectory.back().longitudinal_velocity_mps == 0.0F,
        "승인 전 가상 신호등의 지도 정지선에 정지합니다");
  virtual_input.virtual_traffic_light_messages[900000219] =
      {10.0, true, false};
  const auto approved = virtual_planner.apply(virtual_input);
  check(approved.virtual_traffic_light_states.at(900000219).stop ==
            planning::VirtualTrafficLightStop::None &&
            approved.trajectory.back().longitudinal_velocity_mps == 5.0F,
        "승인된 가상 신호등의 정지선을 통과합니다");
  virtual_input.ego_y = 23983.0;
  const auto passing = virtual_planner.apply(virtual_input);
  check(passing.virtual_traffic_light_states.at(900000219).stop ==
            planning::VirtualTrafficLightStop::EndLine &&
            passing.virtual_traffic_light_commands.at(900000219).state == 2 &&
            passing.trajectory.back().longitudinal_velocity_mps == 0.0F,
        "진입 후 완료 전에는 종료선에 정지합니다");
  virtual_input.virtual_traffic_light_messages[900000219].finalized = true;
  const auto completed = virtual_planner.apply(virtual_input);
  check(completed.virtual_traffic_light_states.at(900000219).stop ==
            planning::VirtualTrafficLightStop::None,
        "완료 승인 뒤에는 종료선 정지를 해제합니다");
  planning::BehaviorVelocityMapPlanner blind_spot_planner(
      {virtual_string.begin(), virtual_string.end()},
      {planning::BehaviorVelocityModule::BlindSpot});
  auto blind_input = virtual_input;
  blind_input.now_s = 0.0;
  blind_input.ego_y = 23980.0;
  blind_input.ego_velocity_mps = 5.0;
  planning::BlindSpotInput blind_frame;
  blind_frame.default_stop_arc_m = 20.0;
  blind_frame.critical_stop_arc_m = 22.0;
  blind_frame.ego_detection_lane_arc_m = 10.0;
  blind_frame.stop_detection_lane_arc_m = 20.0;
  blind_frame.objects = {{"보행자", 7, true, 24.0, 2.0}};
  blind_input.blind_spot_inputs[24747] = blind_frame;
  const auto blind_stop = blind_spot_planner.apply(blind_input);
  check(blind_stop.blind_spot_decisions.at(24747).kind ==
            planning::BlindSpotDecisionKind::Unsafe &&
            blind_stop.trajectory[20].longitudinal_velocity_mps == 0.0F,
        "사각지대 시간 충돌을 지도 통합 경로의 정지점에 적용합니다");
  blind_input.blind_spot_inputs[24747].objects.clear();
  blind_input.now_s = 1.0;
  check(blind_spot_planner.apply(blind_input).blind_spot_decisions.at(24747).kind ==
            planning::BlindSpotDecisionKind::Unsafe,
        "사각지대 객체가 사라진 첫 프레임에는 정지를 유지합니다");
  blind_input.now_s = 1.1;
  check(blind_spot_planner.apply(blind_input).blind_spot_decisions.at(24747).kind ==
            planning::BlindSpotDecisionKind::Safe,
        "사각지대 정지 유지가 끝나면 통과합니다");
  planning::BehaviorVelocityMapPlanner run_out_planner(
      {virtual_string.begin(), virtual_string.end()},
      {planning::BehaviorVelocityModule::RunOut});
  virtual_input.run_out_collision_arc_m = 20.0;
  virtual_input.ego_y = 23980.5;
  virtual_input.ego_velocity_mps = 5.0;
  virtual_input.now_s = 0.0;
  const auto momentary = run_out_planner.apply(virtual_input);
  check(!momentary.run_out_stop_arc_m,
        "순간 검출된 돌발 진입 장애물은 정지시키지 않습니다");
  virtual_input.now_s = 0.16;
  const auto run_out_stopped = run_out_planner.apply(virtual_input);
  check(run_out_stopped.run_out_deceleration_required &&
            run_out_stopped.run_out_stop_arc_m &&
            std::abs(*run_out_stopped.run_out_stop_arc_m - 13.71) < 1.0e-6 &&
            run_out_stopped.trajectory.back().longitudinal_velocity_mps == 0.0F,
        "전달된 돌발 진입 충돌 호길이를 지도 통합 경로에 적용합니다");
  virtual_input.run_out_collision_arc_m.reset();
  virtual_input.now_s = 0.5;
  const auto held_run_out = run_out_planner.apply(virtual_input);
  check(held_run_out.run_out_stop_arc_m &&
            std::abs(*held_run_out.run_out_stop_arc_m - 13.71) < 1.0e-6,
        "장애물이 사라져도 설정 시간 동안 정지점을 유지합니다");
  virtual_input.now_s = 2.16;
  const auto released_run_out = run_out_planner.apply(virtual_input);
  check(!released_run_out.run_out_stop_arc_m,
        "정지점 유지 시간이 지나면 경로 정지를 해제합니다");
  planning::BehaviorVelocityMapPlanner immediate_run_out_planner(
      {virtual_string.begin(), virtual_string.end()},
      {planning::BehaviorVelocityModule::RunOut},
      {false, 0.15, 2.0, false});
  virtual_input.run_out_collision_arc_m = 20.0;
  virtual_input.now_s = 0.0;
  const auto immediate_run_out = immediate_run_out_planner.apply(virtual_input);
  check(immediate_run_out.run_out_stop_arc_m.has_value(),
        "생성자 설정으로 순간 검출 필터를 끌 수 있습니다");
  planning::BehaviorVelocityMapPlanner object_run_out_planner(
      {virtual_string.begin(), virtual_string.end()},
      {planning::BehaviorVelocityModule::RunOut},
      {false, 0.15, 2.0, false});
  auto object_input = virtual_input;
  object_input.run_out_collision_arc_m.reset();
  for (auto &point : object_input.trajectory) {
    point.orientation_z = std::sqrt(0.5);
    point.orientation_w = std::sqrt(0.5);
  }
  planning::RunOutCollisionObject run_out_object;
  run_out_object.id = "합성 보행자";
  run_out_object.pose = {2595.6, 23987.6, 0.0};
  run_out_object.predicted_paths = {
      {{run_out_object.pose, run_out_object.pose}, 1.0, 1.0}};
  run_out_object.shape_type = 1;
  run_out_object.length_m = 0.8;
  object_input.run_out_objects.push_back(run_out_object);
  const auto object_run_out = object_run_out_planner.apply(object_input);
  check(object_run_out.run_out_collision.has_value() &&
            object_run_out.run_out_collision->id == "합성 보행자",
        "예측 객체 입력으로 돌발 진입 충돌점을 직접 계산합니다");

  planning::LaneletBinAdapter stop_sign(readMap(argv[2]));
  const auto signs = stop_sign.stopSignStopLinesForLanelet(24747);
  check(signs.size() == 1 && signs[0].regulatory_id == 900000007 &&
            near(signs[0].segment.x1, 2590.0) &&
            near(signs[0].segment.y1, 23981.0) &&
            near(signs[0].segment.x2, 2602.0) &&
            near(signs[0].segment.y2, 23981.0),
        "합성 정지 표지의 차로 연결과 좌표를 읽습니다");

  planning::LaneletBinAdapter signal(readMap(argv[3]));
  const auto light_lines = signal.trafficLightStopLines(25005);
  check(!light_lines.empty() && std::isfinite(light_lines[0].x1),
        "신호등 규제 요소의 정지선을 읽습니다");
  check(signal.crosswalkBounds(24783).left.size() == crosswalk.left.size(),
        "합성 신호 지도에서도 횡단보도 형상을 유지합니다");
  check(original.crosswalkTrafficLightIds(24783).empty(),
        "실측 횡단보도에는 신호등 규제 요소가 없습니다");
  check(signal.crosswalkTrafficLightIds(24783) ==
            std::vector<std::int64_t>{25005},
        "합성 지도에서 횡단보도에 연결한 신호등을 찾습니다");
  check(original.intersectionObservedTrafficLightIds(25001).empty(),
        "실측 교차로 신호등에는 전구 연결 정보가 없습니다");
  planning::LaneletBinAdapter intersection_signal(readMap(argv[4]));
  check(intersection_signal.intersectionObservedTrafficLightIds(25001) ==
            std::vector<std::int64_t>{25004},
        "합성 전구 연결에서 교차로 관측 신호 ID를 찾습니다");
  const auto out_of_lane_red_lines = intersection_signal.outOfLaneRedStopLines(
      {2614.8, 24007.4}, 200.0, {25004});
  check(!out_of_lane_red_lines.empty() &&
            out_of_lane_red_lines.front().vertices.size() >= 2 &&
            out_of_lane_red_lines.front().lane_polygon.size() >= 3,
        "적색 신호 정지선을 선행 차로 다각형과 연결합니다");
  const auto &red_line = out_of_lane_red_lines.front();
  planning::MotionVelocityPoint2D lane_center{};
  for (const auto &point : red_line.lane_polygon) {
    lane_center.x += point.x;
    lane_center.y += point.y;
  }
  lane_center.x /= red_line.lane_polygon.size();
  lane_center.y /= red_line.lane_polygon.size();
  const planning::MotionVelocityPoint2D stop_center{
      (red_line.vertices.front().x + red_line.vertices.back().x) / 2.0,
      (red_line.vertices.front().y + red_line.vertices.back().y) / 2.0};
  planning::OutOfLanePredictedPath red_crossing;
  for (int i = 0; i <= 10; ++i) {
    planning::TrajectoryPoint pose;
    const double ratio = static_cast<double>(i) / 5.0;
    pose.x = lane_center.x + ratio * (stop_center.x - lane_center.x);
    pose.y = lane_center.y + ratio * (stop_center.y - lane_center.y);
    red_crossing.poses.push_back(pose);
  }
  const auto red_cut = planning::cutOutOfLanePredictedPathAtStopLines(
      red_crossing, out_of_lane_red_lines, 0.0);
  check(red_cut.poses.size() < red_crossing.poses.size(),
        "지도 적색 정지선으로 예측 객체 경로를 자릅니다");
  check(intersection_signal.outOfLaneRedStopLines(
            {2614.8, 24007.4}, 200.0, {}).empty(),
        "적색 신호가 없으면 차로 밖 주행 정지선을 만들지 않습니다");
  planning::LaneletBinAdapter intersection_control(readMap(argv[5]));
  const auto controlled = intersection_control.intersectionTrafficControlDistances(
      25001, 2614.8, 24007.4, 3.141592653589793);
  check(controlled && controlled->conflict_lanelet_id == 24869 &&
            std::abs(controlled->distance_to_stopline_m) < 0.1 &&
            controlled->stopline_to_ego_path_m > 20.0,
        "합성 상대 차로 정지선과 자차 경로의 거리를 읽습니다");
  check(!intersection_signal.intersectionTrafficControlDistances(
            25001, 2614.8, 24007.4, 3.141592653589793),
        "상대 차로 정지선이 없으면 신호 제어 거리를 사용하지 않습니다");

  bool rejected = false;
  try {
    planning::LaneletBinAdapter empty({});
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, "빈 이진 지도 자료를 거부합니다");
}
