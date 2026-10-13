// SPDX-License-Identifier: Apache-2.0
// 기록된 경로·인식 객체·차량 상태를 모든 지도 플러그인과 종단 코어에 전달합니다.

#include "planning/behavior_velocity_perception_bridge.hpp"
#include "planning/dynamic_obstacle_stop_core.hpp"
#include "planning/planning_final_pipeline_core.hpp"
#include "planning/planning_pipeline_map_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {

bool readObjects(std::vector<planning::PlanningPerceivedObject> &objects) {
  std::size_t object_count = 0;
  if (!(std::cin >> object_count)) return false;
  objects.resize(object_count);
  for (auto &object : objects) {
    unsigned classification = 0;
    std::size_t path_count = 0;
    std::size_t footprint_count = 0;
    if (!(std::cin >> object.id >> classification >> object.shape_type >>
          object.x >> object.y >> object.yaw_rad >>
          object.length_m >> object.width_m >>
          object.velocity_x_mps >> object.velocity_y_mps >>
          footprint_count >> path_count)) return false;
    object.classification = static_cast<std::uint8_t>(classification);
    object.footprint.resize(footprint_count);
    for (auto &point : object.footprint)
      if (!(std::cin >> point.x >> point.y)) return false;
    for (std::size_t index = 0; index < path_count; ++index) {
      planning::CruisePredictedPath path;
      std::size_t pose_count = 0;
      if (!(std::cin >> path.time_step_s >> path.confidence >> pose_count))
        return false;
      for (std::size_t pose_index = 0; pose_index < pose_count;
           ++pose_index) {
        planning::CruisePredictedPose pose;
        if (!(std::cin >> pose.x >> pose.y >> pose.yaw_rad)) return false;
        path.poses.push_back(pose);
      }
      object.predicted_paths.push_back(std::move(path));
    }
  }
  return true;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) return 2;
  bool dynamic_stop_enabled = true;
  bool cruise_candidates_enabled = false;
  bool cruise_slowdown_enabled = false;
  bool computed_cruise_limit_enabled = true;
  bool strict_cruise = false;
  bool package_cruise_hysteresis = false;
  std::optional<double> cruise_stop_to_cruise_speed_mps;
  std::optional<double> cruise_cruise_to_stop_speed_mps;
  bool enable_outside_stop_classes = false;
  std::size_t outside_predicted_path_count = 3;
  double cruise_maximum_lateral_time_s = 5.0;
  double cruise_stop_collision_time_margin_s = 1.0;
  std::size_t outside_stop_predicted_path_count = 1;
  double outside_stop_lateral_time_s = 4.5;
  double pedestrian_deceleration_mps2 = 0.5;
  double bicycle_deceleration_mps2 = 0.5;
  bool from_route = false;
  bool experimental_route_sequence_clip = false;
  bool simulation = false;
  bool crosswalk_auto_activation = false;
  bool cruise_callback_snapshot = false;
  bool recorded_behavior_control = false;
  std::vector<std::string> disabled_velocity_modules;
  std::vector<std::size_t> diagnose_velocity_frames;
  double route_backward_path_length_m = 5.0;
  planning::CruisePidParameters cruise_pid_parameters;
  for (int index = 2; index < argc; ++index) {
    const std::string option(argv[index]);
    if (option == "--without-dynamic-stop")
      dynamic_stop_enabled = false;
    else if (option == "--with-cruise-candidates")
      cruise_candidates_enabled = true;
    else if (option == "--with-cruise-slowdown") {
      cruise_candidates_enabled = true;
      cruise_slowdown_enabled = true;
    }
    else if (option == "--cruise-callback-snapshot")
      cruise_callback_snapshot = true;
    else if (option == "--use-recorded-external-limit")
      computed_cruise_limit_enabled = false;
    else if (option == "--strict-cruise")
      strict_cruise = true;
    else if (option == "--package-cruise-hysteresis")
      package_cruise_hysteresis = true;
    else if (option == "--cruise-stop-to-cruise-speed" && index + 1 < argc)
      cruise_stop_to_cruise_speed_mps = std::stod(argv[++index]);
    else if (option == "--cruise-cruise-to-stop-speed" && index + 1 < argc)
      cruise_cruise_to_stop_speed_mps = std::stod(argv[++index]);
    else if (option == "--enable-outside-stop-classes")
      enable_outside_stop_classes = true;
    else if (option == "--cruise-outside-predicted-path-count" && index + 1 < argc)
      outside_predicted_path_count = std::stoul(argv[++index]);
    else if (option == "--cruise-max-lateral-time-margin" && index + 1 < argc)
      cruise_maximum_lateral_time_s = std::stod(argv[++index]);
    else if (option == "--cruise-stop-collision-time-margin" && index + 1 < argc)
      cruise_stop_collision_time_margin_s = std::stod(argv[++index]);
    else if (option == "--cruise-outside-stop-predicted-path-count" && index + 1 < argc)
      outside_stop_predicted_path_count = std::stoul(argv[++index]);
    else if (option == "--cruise-outside-stop-lateral-time" && index + 1 < argc)
      outside_stop_lateral_time_s = std::stod(argv[++index]);
    else if (option == "--cruise-pedestrian-deceleration" && index + 1 < argc)
      pedestrian_deceleration_mps2 = std::stod(argv[++index]);
    else if (option == "--cruise-bicycle-deceleration" && index + 1 < argc)
      bicycle_deceleration_mps2 = std::stod(argv[++index]);
    else if (option == "--from-route")
      from_route = true;
    else if (option == "--experimental-route-sequence-clip")
      experimental_route_sequence_clip = true;
    else if (option == "--simulation")
      simulation = true;
    else if (option == "--crosswalk-auto-activation")
      crosswalk_auto_activation = true;
    else if (option == "--with-recorded-behavior-control")
      recorded_behavior_control = true;
    else if (option == "--disable-velocity-module" && index + 1 < argc)
      disabled_velocity_modules.emplace_back(argv[++index]);
    else if (option == "--diagnose-velocity-frame" && index + 1 < argc)
      diagnose_velocity_frames.push_back(std::stoul(argv[++index]));
    else if (option == "--route-backward-path-length" && index + 1 < argc)
      route_backward_path_length_m = std::stod(argv[++index]);
    else if (option == "--cruise-safe-distance-margin" && index + 1 < argc)
      cruise_pid_parameters.safe_distance_margin = std::stod(argv[++index]);
    else return 2;
  }
  if (!std::isfinite(route_backward_path_length_m) ||
      route_backward_path_length_m < 0.0) return 2;
  if (!std::isfinite(cruise_pid_parameters.safe_distance_margin) ||
      cruise_pid_parameters.safe_distance_margin < 0.0) return 2;
  if (!std::isfinite(cruise_maximum_lateral_time_s) ||
      cruise_maximum_lateral_time_s < 0.0) return 2;
  if (!std::isfinite(cruise_stop_collision_time_margin_s) ||
      cruise_stop_collision_time_margin_s < 0.0) return 2;
  std::ifstream map_file(argv[1], std::ios::binary);
  if (!map_file) return 3;
  const std::vector<std::uint8_t> map_data{
      std::istreambuf_iterator<char>(map_file),
      std::istreambuf_iterator<char>()};
  using Module = planning::BehaviorVelocityModule;
  planning::CruiseCandidateParameters cruise_parameters;
  cruise_parameters.stop_safe_distance_margin_m =
      cruise_pid_parameters.safe_distance_margin;
  cruise_parameters.outside_predicted_path_count = outside_predicted_path_count;
  cruise_parameters.maximum_lateral_time_s = cruise_maximum_lateral_time_s;
  cruise_parameters.stop_collision_time_margin_s =
      cruise_stop_collision_time_margin_s;
  cruise_parameters.outside_stop_predicted_path_count =
      outside_stop_predicted_path_count;
  cruise_parameters.outside_stop_lateral_time_s =
      outside_stop_lateral_time_s;
  cruise_parameters.pedestrian_deceleration_mps2 =
      pedestrian_deceleration_mps2;
  cruise_parameters.bicycle_deceleration_mps2 =
      bicycle_deceleration_mps2;
  if (enable_outside_stop_classes) {
    cruise_parameters.outside_stop_classes.fill(true);
    cruise_parameters.outside_stop_classes[0] = false;
  }
  if (strict_cruise) {
    cruise_parameters.crossing_angle_threshold_rad = 1.22;
    cruise_parameters.minimum_outside_speed_mps = 3.5;
    cruise_parameters.minimum_outside_overlap_s = 2.0;
  }
  if (package_cruise_hysteresis) {
    cruise_parameters.inside_cruise_entry_speed_mps = 3.5;
    cruise_parameters.inside_cruise_keep_speed_mps = 3.0;
  }
  if (cruise_stop_to_cruise_speed_mps)
    cruise_parameters.inside_cruise_entry_speed_mps =
        *cruise_stop_to_cruise_speed_mps;
  if (cruise_cruise_to_stop_speed_mps)
    cruise_parameters.inside_cruise_keep_speed_mps =
        *cruise_cruise_to_stop_speed_mps;
  if (!std::isfinite(cruise_parameters.inside_cruise_entry_speed_mps) ||
      !std::isfinite(cruise_parameters.inside_cruise_keep_speed_mps) ||
      cruise_parameters.inside_cruise_keep_speed_mps < 0.0 ||
      cruise_parameters.inside_cruise_entry_speed_mps <
          cruise_parameters.inside_cruise_keep_speed_mps) return 2;
  std::vector<Module> velocity_modules{
      // default_preset.yaml + behavior_planning.launch.xml plugin order.
      Module::CrosswalkDefault, Module::Walkway, Module::TrafficLight,
      Module::IntersectionStuck, Module::IntersectionCollision,
      Module::MergeFromPrivate, Module::BlindSpot,
      Module::DetectionArea, Module::NoStoppingArea,
      Module::StopSign, Module::RunOut};
  for (const auto &name : disabled_velocity_modules) {
    const auto module = name == "crosswalk" ? Module::CrosswalkDefault :
        name == "blind_spot" ? Module::BlindSpot :
        name == "intersection" ? Module::IntersectionCollision :
        name == "run_out" ? Module::RunOut :
        name == "traffic_light" ? Module::TrafficLight :
        Module::StopSign;
    if (name != "crosswalk" && name != "blind_spot" &&
        name != "intersection" && name != "run_out" &&
        name != "traffic_light") return 2;
    velocity_modules.erase(std::remove(velocity_modules.begin(),
                                       velocity_modules.end(), module),
                           velocity_modules.end());
  }
  planning::PlanningPipelineMapAdapter pipeline(map_data, velocity_modules, {}, {},
      planning::InvalidTrajectoryPolicy::UsePreviousResult,
      {}, cruise_parameters, {}, experimental_route_sequence_clip,
      route_backward_path_length_m, {11.1}, cruise_pid_parameters);
  planning::PlanningFinalPipelineCore final_pipeline;
  std::unique_ptr<planning::LaneletBinAdapter> scenario_map;
  if (from_route)
    scenario_map = std::make_unique<planning::LaneletBinAdapter>(map_data);
  std::size_t frame_count = 0;
  if (!(std::cin >> frame_count)) return 4;
  planning::MissionRoute route;
  planning::BehaviorGoalPose goal;
  std::string route_id;
  if (from_route) {
    std::size_t section_count = 0;
    if (!(std::cin >> goal.x >> goal.y >> goal.yaw_rad >>
          route.goal_height_m >> section_count >> route_id) ||
        section_count == 0)
      return 21;
    route.sections.resize(section_count);
    for (auto &section : route.sections) {
      std::size_t primitive_count = 0;
      if (!(std::cin >> section.preferred_lanelet_id >> primitive_count))
        return 22;
      section.primitive_lanelet_ids.resize(primitive_count);
      for (auto &id : section.primitive_lanelet_ids)
        if (!(std::cin >> id)) return 23;
    }
  }
  std::cout << std::setprecision(17);
  for (std::size_t frame_index = 0; frame_index < frame_count; ++frame_index) {
    planning::PlanningPipelineFrame frame;
    planning::PlanningFinalFrame final_frame;
    bool route_lane_change_approved = false;
    bool behavior_autonomous_control = true;
    planning::BehaviorVelocityMapInput context;
    planning::PlanningCruisePerceptionFrame cruise_perception;
    cruise_perception.slowdown_enabled = cruise_slowdown_enabled;
    std::size_t path_count = 0;
    if (!(std::cin >> frame.timestamp_s >> frame.ego.x >> frame.ego.y >>
          frame.ego.z >> frame.ego.orientation_x >>
          frame.ego.orientation_y >> frame.ego.orientation_z >>
          frame.ego.orientation_w >> frame.ego_speed_mps >>
          context.ego_acceleration_mps2 >>
          cruise_perception.stamp_s >>
          path_count)) return 5;
    if (from_route && path_count != 0) return 24;
    if (from_route) {
      int autonomous = 0, control_enabled = 0;
      int external_active = 0, use_constraints = 0;
      int lane_change_approved = 0;
      double node_maximum = 0.0;
      if (!(std::cin >> final_frame.odometry_stamp_s >> autonomous >>
            control_enabled >> final_frame.maximum_velocity_mps >>
            node_maximum >> external_active >> use_constraints >>
            final_frame.external_min_acceleration_mps2 >>
            final_frame.external_max_jerk_mps3 >>
            final_frame.external_min_jerk_mps3 >>
            lane_change_approved)) return 25;
      final_frame.autonomous = autonomous != 0;
      final_frame.control_enabled = control_enabled != 0;
      final_frame.node_maximum_velocity_mps = node_maximum;
      final_frame.external_limit_active = external_active != 0;
      final_frame.external_limit_use_constraints = use_constraints != 0;
      route_lane_change_approved = lane_change_approved != 0;
      if (recorded_behavior_control) {
        int autonomous_control = 0;
        if (!(std::cin >> autonomous_control)) return 28;
        behavior_autonomous_control = autonomous_control != 0;
      }
    }
    for (std::size_t index = 0; index < path_count; ++index) {
      planning::TrajectoryPoint point;
      std::size_t lane_count = 0;
      if (!(std::cin >> point.x >> point.y >> point.z >>
            point.orientation_x >> point.orientation_y >>
            point.orientation_z >> point.orientation_w >>
            point.longitudinal_velocity_mps >>
            point.lateral_velocity_mps >> point.heading_rate_rps >>
            lane_count)) return 6;
      std::vector<std::int64_t> ids(lane_count);
      for (auto &id : ids) if (!(std::cin >> id)) return 7;
      frame.behavior_path.points.push_back(point);
      frame.behavior_path.lane_ids.push_back(std::move(ids));
    }
    for (int side = 0; side < 2; ++side) {
      std::size_t count = 0;
      if (!(std::cin >> count)) return 8;
      auto &bound = side == 0 ? frame.left_bound : frame.right_bound;
      for (std::size_t index = 0; index < count; ++index) {
        planning::PathOptimizerPoint2D point;
        if (!(std::cin >> point.x >> point.y)) return 9;
        bound.push_back(point);
      }
    }
    std::size_t signal_count = 0;
    if (!(std::cin >> signal_count)) return 14;
    for (std::size_t index = 0; index < signal_count; ++index) {
      std::int64_t id = 0;
      double stamp_s = 0.0;
      std::size_t element_count = 0;
      if (!(std::cin >> id >> stamp_s >> element_count)) return 15;
      planning::TrafficLightSignalObservation observation;
      observation.stamp_s = stamp_s;
      for (std::size_t element_index = 0;
           element_index < element_count; ++element_index) {
        int color = 0, shape = 0;
        if (!(std::cin >> color >> shape)) return 16;
        planning::TrafficLampElement element;
        element.color = color == 1 ? planning::TrafficLampColor::Red :
            color == 2 ? planning::TrafficLampColor::Yellow :
            color == 3 ? planning::TrafficLampColor::Green :
                         planning::TrafficLampColor::Unknown;
        element.shape = shape == 1 ? planning::TrafficLampShape::Circle :
            shape == 2 ? planning::TrafficLampShape::LeftArrow :
            shape == 3 ? planning::TrafficLampShape::RightArrow :
            shape == 4 ? planning::TrafficLampShape::UpArrow :
                         planning::TrafficLampShape::Unknown;
        observation.elements.push_back(element);
      }
      context.traffic_signal_observations.emplace(id, observation);
      if (!observation.elements.empty()) {
        context.intersection_signal_colors.emplace(
            id, observation.elements.front().color);
      }
      planning::CrosswalkPedestrianSignal pedestrian;
      pedestrian.stamp_s = stamp_s;
      pedestrian.elements = observation.elements;
      context.pedestrian_signals.emplace(id, pedestrian);
    }
    int grid_present = 0;
    if (!(std::cin >> grid_present)) return 17;
    if (grid_present) {
      planning::CrosswalkOccupancyGrid grid;
      if (!(std::cin >> grid.width >> grid.height >> grid.resolution_m >>
            grid.origin_x >> grid.origin_y >> grid.origin_yaw_rad)) return 18;
      if (grid.width == 0 || grid.height == 0 ||
          grid.width > 10000 || grid.height > 10000 ||
          grid.width * grid.height > 2'000'000) return 19;
      grid.values.reserve(grid.width * grid.height);
      for (std::size_t index = 0; index < grid.width * grid.height;
           ++index) {
        int value = 0;
        if (!(std::cin >> value) || value < -128 || value > 127) return 20;
        grid.values.push_back(static_cast<std::int8_t>(value));
      }
      context.crosswalk_occupancy_grid = grid;
      context.intersection_occupancy_grid = std::move(grid);
      context.crosswalk_occlusion_enabled = true;
      context.intersection_occlusion_enabled = true;
    }
    std::optional<planning::PlanningCruisePerceptionFrame> callback_perception;
    if (cruise_callback_snapshot) {
      planning::PlanningPipelineFrame::CruiseCallback callback;
      if (!(std::cin >> callback.timestamp_s >> callback.ego.x >>
            callback.ego.y >> callback.ego.z >>
            callback.ego.orientation_x >> callback.ego.orientation_y >>
            callback.ego.orientation_z >> callback.ego.orientation_w >>
            callback.ego_speed_mps)) return 26;
      double callback_acceleration = 0.0;
      if (!(std::cin >> callback_acceleration)) return 26;
      callback.ego_acceleration_mps2 = callback_acceleration;
      frame.cruise_callback = callback;
      callback_perception.emplace();
      callback_perception->slowdown_enabled = cruise_slowdown_enabled;
      if (!(std::cin >> callback_perception->stamp_s) ||
          !readObjects(callback_perception->objects)) return 27;
    }
    std::vector<planning::PlanningPerceivedObject> objects;
    if (!readObjects(objects)) return 10;
    planning::applyBehaviorVelocityPerceptionBatch(context, objects);
    context.is_simulation = simulation;
    // With crosswalk RTC disabled, the original module syncs activation to
    // safety; a clear crossing takes the go branch.
    context.crosswalk_auto_activate_when_safe = crosswalk_auto_activation;
    if (!dynamic_stop_enabled) context.dynamic_stop_objects.clear();
    if (cruise_candidates_enabled)
      cruise_perception.objects = std::move(objects);
    const auto result = from_route ?
        pipeline.onRouteFrame(frame, route, context, cruise_perception,
                              {}, goal, std::nullopt, 0.0, std::nullopt,
                              route_lane_change_approved,
                              callback_perception, std::nullopt,
                              recorded_behavior_control ?
                                  planning::PlanningRouteControlState{
                                      behavior_autonomous_control &&
                                          final_frame.control_enabled,
                                      route_lane_change_approved} :
                                  planning::PlanningRouteControlState{}) :
        pipeline.onFrame(frame, context, cruise_perception, {},
                         callback_perception);
    if (result.behavior_velocity &&
        std::find(diagnose_velocity_frames.begin(),
                  diagnose_velocity_frames.end(), frame_index) !=
            diagnose_velocity_frames.end()) {
      std::cerr << "velocity frame=" << frame_index << " crosswalks=";
      for (const auto id : result.behavior_velocity->crosswalk_lanelet_ids)
        std::cerr << id << ',';
      std::cerr << " states=";
      for (const auto &[id, objects] : result.behavior_velocity->crosswalk_object_states) {
        std::cerr << id << ':';
        for (const auto &[object_id, state] : objects)
          std::cerr << object_id << '=' << static_cast<int>(state) << ',';
      }
      std::cerr << "\n";
      std::cerr << "traffic frame=" << frame_index << " ids=";
      for (const auto id : result.behavior_velocity->traffic_light_regulatory_ids)
        std::cerr << id << ',';
      std::cerr << " observations=";
      for (const auto &[id, observation] : context.traffic_signal_observations) {
        std::cerr << id << ':' << observation.stamp_s << ':';
        for (const auto &element : observation.elements)
          std::cerr << static_cast<int>(element.color) << ',';
      }
      std::cerr << "\n";
    }
    planning::PlanningFinalResult final_result;
    if (from_route && result.pipeline.status ==
                          planning::PlanningPipelineStatus::Complete) {
      if (std::find(diagnose_velocity_frames.begin(),
                    diagnose_velocity_frames.end(), frame_index) !=
          diagnose_velocity_frames.end()) {
        std::cerr << "cruise limit frame=" << frame_index
                  << " pid_obstacles="
                  << result.cruise_pid_obstacle_count
                  << " selected="
                  << (result.selected_velocity_limit ?
                      result.selected_velocity_limit->max_velocity : -1.0F)
                  << " distance="
                  << (result.cruise_pid_limit ?
                      result.cruise_pid_limit->obstacle_distance : -1.0)
                  << " target="
                  << (result.cruise_pid_limit ?
                      result.cruise_pid_limit->target_distance : -1.0)
                  << " error="
                  << (result.cruise_pid_limit ?
                      result.cruise_pid_limit->error_distance : -1.0)
                  << " recorded="
                  << (final_frame.external_limit_active ?
                      final_frame.maximum_velocity_mps : -1.0)
                  << "\n";
        for (const auto &diagnostic : result.cruise_diagnostics)
          std::cerr << "cruise candidate frame=" << frame_index
                    << " id=" << diagnostic.id
                    << " lateral=" << diagnostic.lateral_distance_m
                    << " overlap=" << diagnostic.collision_overlap_s
                    << " first_arc=" << diagnostic.first_collision_arc_m
                    << " velocity=" << diagnostic.projected_velocity_mps
                    << " approach=" << diagnostic.approach_velocity_mps
                    << " rank=" << diagnostic.prediction_rank << "\n";
      }
      if (computed_cruise_limit_enabled &&
          !final_frame.external_limit_active &&
          result.selected_velocity_limit) {
        planning::applySelectedVelocityLimit(
            final_frame, *result.selected_velocity_limit);
      }
      final_frame.now_s = frame.timestamp_s;
      final_frame.route_id = route_id;
      final_frame.ego = frame.ego;
      final_frame.ego_speed_mps = frame.ego_speed_mps;
      final_frame.ego_acceleration_mps2 = context.ego_acceleration_mps2;
      final_frame.lane_driving = result.pipeline.cruise_stopped;
      std::vector<std::int64_t> route_lane_ids;
      for (const auto &section : route.sections)
        route_lane_ids.push_back(section.preferred_lanelet_id);
      final_frame.position = scenario_map->scenarioPosition(
          frame.ego.x, frame.ego.y,
          std::atan2(2.0 * (frame.ego.orientation_w *
                            frame.ego.orientation_z +
                            frame.ego.orientation_x *
                            frame.ego.orientation_y),
                     1.0 - 2.0 * (frame.ego.orientation_y *
                                  frame.ego.orientation_y +
                                  frame.ego.orientation_z *
                                  frame.ego.orientation_z)),
          goal.x, goal.y, route_lane_ids);
      final_result = final_pipeline.onFrame(final_frame);
      if (std::find(diagnose_velocity_frames.begin(),
                    diagnose_velocity_frames.end(), frame_index) !=
          diagnose_velocity_frames.end()) {
        std::cerr << "smoother frame=" << frame_index
                  << " solved=" << final_result.smoother_solved
                  << " ego_velocity=" << frame.ego_speed_mps
                  << " ego_acceleration=" << context.ego_acceleration_mps2
                  << "\n";
      }
    }
    const auto validated_output = final_result.validation ?
        final_result.validation->selected_trajectory :
        std::optional<planning::Trajectory>{};
    double slowdown_start_arc = -1.0;
    double slowdown_end_arc = -1.0;
    double slowdown_target_velocity = -1.0;
    if (!result.cruise_slowdown_intervals.empty() &&
        result.pipeline.motion_velocity.size() >= 2) {
      const auto &interval = result.cruise_slowdown_intervals.front();
      slowdown_start_arc = planning::dynamicStopArc(
          result.pipeline.motion_velocity, interval.from);
      slowdown_end_arc = planning::dynamicStopArc(
          result.pipeline.motion_velocity, interval.to);
      slowdown_target_velocity = interval.velocity;
    }
    std::cout << frame_index << ' '
              << static_cast<int>(result.pipeline.status) << ' '
              << (result.behavior_velocity ?
                  result.behavior_velocity->trajectory.size() : 0) << ' '
              << result.pipeline.elastic_band.size() << ' '
              << result.pipeline.optimized.size() << ' '
              << result.pipeline.motion_velocity.size() << ' '
              << result.pipeline.cruise_stopped.size() << ' '
              << (result.pipeline.validation ? 1 : 0) << ' '
              << result.pipeline.cruise_candidate_count << ' '
              << (result.pipeline.cruise_stop_distance_from_start_m ? 1 : 0)
              << ' ' << (final_result.selected ? final_result.selected->size() : 0)
              << ' ' << (final_result.smoothed ? final_result.smoothed->size() : 0)
              << ' ' << (validated_output ? validated_output->size() : 0)
              << ' ' << (final_result.scenario ?
                         static_cast<int>(final_result.scenario->current) : 0)
              << ' ' << result.route_behavior_path.points.size()
              << ' ' << result.route_reference_path.points.size()
              << ' ' << result.route_goal_refined_path.points.size()
              << ' ' << result.cruise_pid_obstacle_count
              << ' ' << (result.cruise_pid_limit ?
                            result.cruise_pid_limit->obstacle_distance : -1.0)
              << ' ' << result.cruise_slowdown_candidate_count
              << ' ' << slowdown_start_arc << ' ' << slowdown_end_arc
              << ' ' << slowdown_target_velocity
              << ' ' << (result.behavior_action ?
                  static_cast<int>(result.behavior_action->kind) : -1)
              << ' ' << (result.behavior_action ?
                  result.behavior_action->lane_change_candidate_count : 0)
              << ' ' << (result.behavior_action &&
                  result.behavior_action->lane_change_first_safe_index ?
                  static_cast<int>(*result.behavior_action->lane_change_first_safe_index) : -1)
              << ' ' << (result.behavior_action &&
                  result.behavior_action->lane_change_decision ?
                  static_cast<int>(result.behavior_action->lane_change_decision->module_state) : -1)
              << ' ' << (result.behavior_action &&
                  result.behavior_action->lane_change_candidate_outside_lanes ? 1 : 0)
              << ' ' << (result.behavior_action ?
                  result.behavior_action->lane_change_collision_count : 0)
              << ' ' << (result.behavior_action ?
                  result.behavior_action->lane_change_other_unsafe_count : 0)
              << '\n';
    if (result.behavior_velocity) {
      for (const auto &point : result.behavior_velocity->trajectory) {
        std::cout << point.x << ' ' << point.y << ' '
                  << point.longitudinal_velocity_mps << '\n';
      }
    }
    for (const auto &point : result.pipeline.motion_velocity) {
      std::cout << point.x << ' ' << point.y << ' '
                << point.longitudinal_velocity_mps << '\n';
    }
    for (const auto &point : result.pipeline.cruise_stopped) {
      std::cout << point.x << ' ' << point.y << ' '
                << point.longitudinal_velocity_mps << '\n';
    }
    if (final_result.selected)
      for (const auto &point : *final_result.selected)
        std::cout << point.x << ' ' << point.y << ' '
                  << point.longitudinal_velocity_mps << '\n';
    if (final_result.smoothed)
      for (const auto &point : *final_result.smoothed)
        std::cout << point.x << ' ' << point.y << ' '
                  << point.longitudinal_velocity_mps << '\n';
    if (validated_output)
      for (const auto &point : *validated_output)
        std::cout << point.x << ' ' << point.y << ' '
                  << point.longitudinal_velocity_mps << '\n';
    for (const auto &point : result.route_behavior_path.points)
      std::cout << point.x << ' ' << point.y << ' '
                << point.longitudinal_velocity_mps << '\n';
    if (from_route) {
      for (const auto &point : result.pipeline.elastic_band)
        std::cout << point.x << ' ' << point.y << ' '
                  << point.longitudinal_velocity_mps << '\n';
      for (const auto &point : result.pipeline.optimized)
        std::cout << point.x << ' ' << point.y << ' '
                  << point.longitudinal_velocity_mps << '\n';
    }
    for (const auto &point : result.route_reference_path.points)
      std::cout << point.x << ' ' << point.y << ' '
                << point.longitudinal_velocity_mps << '\n';
    for (const auto &point : result.route_goal_refined_path.points)
      std::cout << point.x << ' ' << point.y << ' '
                << point.longitudinal_velocity_mps << '\n';
  }
  return 0;
}
