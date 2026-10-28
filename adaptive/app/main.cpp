// SPDX-License-Identifier: Apache-2.0
#include "ara/core/initialization.h"
#include "ara/exec/execution_client.h"
#include "ara/log/logger.h"
#include "oss/srv/SCr2MP/mp_si_servicecreator_proxy.h"
#include "oss/srv/MP2SCr/scr_si_missionplanner_skeleton.h"
#include "oss/srv/MP2RS/rs_si_missionplanner_skeleton.h"
#include "oss/srv/PV2SCr/scr_si_planningvalidator_skeleton.h"
#include "oss/srv/SS2VS/vs_si_scenarioselector_skeleton.h"
#include "oss/srv/AA2SCrTurn/scr_si_turncommand_skeleton.h"
#include "oss/srv/AA2SCrHazard/scr_si_hazardcommand_skeleton.h"
#include "oss/srv/AA2SCrMetrics/scr_si_planningmetrics_skeleton.h"
#include "oss/srv/AA2SCrRemaining/scr_si_remainingdistance_skeleton.h"
#include "planning/behavior_velocity_perception_bridge.hpp"
#include "planning/lanelet_bin_adapter.hpp"
#include "planning/planning_final_pipeline_core.hpp"
#include "planning/planning_pipeline_map_adapter.hpp"
#include "planning/planning_route_coordinator.hpp"
#include "planning/planning_evaluator_map_adapter.hpp"
#include "planning/remaining_distance_map_adapter.hpp"
#include "diagnostic_output.hpp"
#include "route_visualization_output.hpp"
#include "trajectory_output.hpp"
#include "behavior_signal_output.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

namespace {
using InputProxy = oss::srv::SCr2MP::proxy::MP_SI_ServiceCreatorProxy;
using RouteSkeleton = oss::srv::MP2SCr::skeleton::SCr_SI_missionplannerSkeleton;
using RouteStateSkeleton = oss::srv::MP2RS::skeleton::RS_SI_missionplannerSkeleton;
using TrajectorySkeleton = oss::srv::PV2SCr::skeleton::SCr_SI_planningvalidatorSkeleton;
using ScenarioSkeleton = oss::srv::SS2VS::skeleton::VS_SI_scenarioselectorSkeleton;
using TurnSkeleton = oss::srv::AA2SCrTurn::skeleton::SCr_SI_turncommandSkeleton;
using HazardSkeleton = oss::srv::AA2SCrHazard::skeleton::SCr_SI_hazardcommandSkeleton;
using MetricsSkeleton = oss::srv::AA2SCrMetrics::skeleton::SCr_SI_planningmetricsSkeleton;
using RemainingSkeleton = oss::srv::AA2SCrRemaining::skeleton::SCr_SI_remainingdistanceSkeleton;

#define INPUT_EVENTS(X) \
  X(clock) \
  X(localization_acceleration) \
  X(localization_kinematicstate) \
  X(map_vectormap) \
  X(perception_objectrecognition_objects) \
  X(perception_obstaclesegmentation_pointcloud) \
  X(perception_occupancygridmap_map) \
  X(perception_trafficlightrecognition_trafficsignals) \
  X(planning_scenarioplanning_lanedriving_behaviorplanning_behaviorpathplanner_input_lateraloffset) \
  X(planning_scenarioplanning_maxvelocitydefault) \
  X(system_operationmode_state)

double nowSeconds() {
  return std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

planning::RouteCheckpoint checkpoint(const oss::type::geometry_msgs::msg::Pose &pose) {
  const auto &q = pose.orientation;
  planning::RouteCheckpoint result{pose.position.x, pose.position.y,
          std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
          pose.position.z};
  result.orientation_x = q.x;
  result.orientation_y = q.y;
  result.orientation_z = q.z;
  result.orientation_w = q.w;
  result.has_orientation = true;
  return result;
}

template <class Uuid>
planning::RouteUuid routeUuid(const Uuid &input) {
  planning::RouteUuid result{};
  std::copy(input.uuid.begin(), input.uuid.end(), result.begin());
  return result;
}

oss::type::autoware_adapi_v1_msgs::msg::ResponseStatus responseStatus(
    const planning::RouteSelectorStatus &status) {
  oss::type::autoware_adapi_v1_msgs::msg::ResponseStatus result{};
  result.success = status.success;
  result.code = static_cast<oss::type::ResponseStatus_code>(status.code);
  result.message = status.message;
  return result;
}

struct SharedState {
  SharedState() : routes([this](planning::RouteCheckpoint start,
                         const std::vector<planning::RouteCheckpoint> &waypoints,
                         planning::RouteCheckpoint goal)
      -> std::optional<planning::MissionRoute> {
    route_failure.clear();
    if (!route_map) {
      route_failure = "Lanelet map is not ready.";
      return std::nullopt;
    }
    std::vector<planning::RouteCheckpoint> checkpoints;
    checkpoints.reserve(waypoints.size() + 2);
    checkpoints.push_back(start);
    checkpoints.insert(checkpoints.end(), waypoints.begin(), waypoints.end());
    checkpoints.push_back(goal);
    return route_map->routeThroughCheckpoints(checkpoints, false,
                                             &route_failure);
  }) {}

  std::mutex mutex;
  std::condition_variable changed;
  bool stopping = false;
  std::uint64_t map_generation = 0;
  std::uint64_t odometry_generation = 0;
  std::shared_ptr<planning::LaneletBinAdapter> route_map;
  std::string route_failure;
  planning::PlanningRouteCoordinator routes;
  std::optional<oss::type::rosgraph_msgs::msg::Clock> clock;
  std::optional<oss::type::geometry_msgs::msg::AccelWithCovarianceStamped> acceleration;
  std::optional<oss::type::nav_msgs::msg::Odometry> odometry;
  std::optional<oss::type::autoware_map_msgs::msg::LaneletMapBin> map;
  std::optional<oss::type::autoware_perception_msgs::msg::PredictedObjects> objects;
  std::optional<oss::type::sensor_msgs::msg::PointCloud2> pointcloud;
  std::optional<oss::type::nav_msgs::msg::OccupancyGrid> occupancy_grid;
  std::optional<oss::type::autoware_msgs::autoware_perception_msgs::msg::TrafficLightGroupArray> traffic_signals;
  std::optional<oss::type::tier4_planning_msgs::msg::LateralOffset> lateral_offset;
  std::optional<oss::type::tier4_planning_msgs::msg::VelocityLimit> velocity_limit;
  std::optional<oss::type::autoware_adapi_v1_msgs::msg::OperationModeState> operation_mode;
};

oss::type::autoware_adapi_v1_msgs::msg::ResponseStatus handleWaypointRoute(
    SharedState &state, planning::RouteSelectorRequest request) {
  state.route_failure.clear();
  auto status = state.routes.handle(
      planning::RouteChannel::Main, std::move(request), nowSeconds());
  if (status.code == static_cast<int>(planning::MissionPlannerCode::PlannerFailed) &&
      !state.route_failure.empty())
    status.message = state.route_failure;
  return responseStatus(status);
}

struct FinalFrameTiming {
  double pipeline_ms = 0.0;
  double conversion_ms = 0.0;
  double send_ms = 0.0;
  double smoother_ms = 0.0;
  double qp_first_solve_ms = 0.0;
  double qp_retry_ms = 0.0;
  int qp_first_iterations = 0;
  int qp_iterations = 0;
  std::size_t qp_points = 0;
  planning::JerkQpBackend qp_backend = planning::JerkQpBackend::Admm;
  bool qp_retried = false;
  bool qp_solved = false;
};

bool publishFinalFrame(planning::PlanningFinalPipelineCore &pipeline,
                       TrajectorySkeleton &trajectory,
                       ScenarioSkeleton &scenario_trajectory,
                       MetricsSkeleton &metrics_skeleton,
                       RemainingSkeleton &remaining_skeleton,
                       planning::PlanningEvaluatorMapAdapter &evaluator,
                       planning::RemainingDistanceMapAdapter &remaining,
                       const planning::MissionRoute &route,
                       const std::vector<planning::PlanningPerceivedObject> &objects,
                       adaptive::DiagnosticOutput &diagnostic_output,
                       ara::log::Logger &log,
                       const planning::PlanningFinalFrame &frame,
                       const std::string &frame_id,
                       const planning::RouteCheckpoint &goal,
                       bool log_details,
                       bool emit_auxiliary,
                       FinalFrameTiming *timing = nullptr) {
  const auto started = std::chrono::steady_clock::now();
  const auto result = pipeline.onFrame(frame);
  const auto planned = std::chrono::steady_clock::now();
  if (emit_auxiliary) {
  remaining.setRoute(route);
  remaining.core().onEgo({frame.ego.x, frame.ego.y, 0.0, frame.ego.z});
  remaining.core().onGoal(goal);
  remaining.core().onVelocityLimit(frame.maximum_velocity_mps);
  remaining.core().onScenario(result.scenario && result.scenario->current == planning::Scenario::Parking
      ? planning::RemainingDistanceScenario::Parking
      : planning::RemainingDistanceScenario::LaneDriving);
  if (const auto value = remaining.core().tick()) {
    oss::type::autoware_internal_msgs::msg::MissionRemainingDistanceTime sample{};
    sample.remaining_distance = value->distance_m;
    sample.remaining_time = value->time_s;
    if (!remaining_skeleton.planning_mission_remaining_distance_time.Send(sample).HasValue())
      log.LogError() << "Failed to send remaining distance Event 12521";
  }
  planning::PlanningEvaluatorInput metric_input;
  metric_input.ego = frame.ego;
  metric_input.ego_speed_mps = frame.ego_speed_mps;
  metric_input.ego_acceleration_mps2 = frame.ego_acceleration_mps2;
  metric_input.acceleration_stamp_s = frame.now_s;
  if (result.validation && result.validation->selected_trajectory)
    metric_input.trajectory = *result.validation->selected_trajectory;
  metric_input.reference_trajectory = frame.lane_driving;
  std::vector<std::pair<double, double>> object_centers;
  object_centers.reserve(objects.size());
  for (const auto &object : objects) object_centers.emplace_back(object.x, object.y);
  metric_input.object_centers = std::move(object_centers);
  const auto metric_result = evaluator.tick(std::move(metric_input), route);
  oss::type::tier4_metric_msgs::msg::MetricArray metric_sample{};
  metric_sample.stamp.sec = static_cast<std::int32_t>(frame.now_s);
  metric_sample.stamp.nanosec = static_cast<std::uint32_t>(
      (frame.now_s - static_cast<double>(metric_sample.stamp.sec)) * 1.0e9);
  const auto add_metric = [&metric_sample](const std::string &name, double value,
                                           bool scalar) {
    oss::type::tier4_metric_msgs::msg::Metric metric{};
    metric.name = name;
    if (name == "ego_lane_info/lane_id") {
      metric.value = std::to_string(static_cast<std::int64_t>(value));
    } else if (scalar) {
      metric.value = std::to_string(value);
    } else {
      std::ostringstream formatted;
      formatted << std::setprecision(std::numeric_limits<double>::max_digits10)
                << value;
      metric.value = formatted.str();
    }
    metric_sample.metric_array.push_back(std::move(metric));
  };
  for (const auto &entry : metric_result.scalar_metrics)
    add_metric(entry.first, entry.second, true);
  for (const auto &entry : metric_result.metrics) {
    add_metric(entry.first + "/min", entry.second.minimum, false);
    add_metric(entry.first + "/max", entry.second.maximum, false);
    add_metric(entry.first + "/mean", entry.second.mean, false);
  }
  if (!metrics_skeleton.planning_evaluator_metrics.Send(metric_sample).HasValue())
    log.LogError() << "Failed to send evaluator metrics Event 12520";
  if (const auto selected = adaptive::makeSelectedTrajectoryEvent(result, frame.now_s, frame_id)) {
    const auto sent = scenario_trajectory.planning_scenarioplanning_scenarioselector_trajectory.Send(*selected);
    if (!sent.HasValue()) log.LogError() << "Failed to send scenario trajectory Event 12908";
  }
  }
  if (result.validation)
    diagnostic_output.send(*result.validation, frame.now_s);
  const auto event = adaptive::makeTrajectoryEvent(result, frame.now_s, frame_id);
  const auto converted = std::chrono::steady_clock::now();
  if (timing) {
    timing->pipeline_ms = std::chrono::duration<double, std::milli>(planned - started).count();
    timing->conversion_ms = std::chrono::duration<double, std::milli>(converted - planned).count();
    timing->smoother_ms = result.smoother_ms;
    timing->qp_first_solve_ms = result.qp_first_solve_ms;
    timing->qp_retry_ms = result.qp_retry_ms;
    timing->qp_first_iterations = result.qp_first_iterations;
    timing->qp_iterations = result.qp_iterations;
    timing->qp_points = result.qp_points;
    timing->qp_backend = result.qp_backend;
    timing->qp_retried = result.qp_retried;
    timing->qp_solved = result.smoother_solved;
  }
  if (!event) return false;
  const auto sent = trajectory.planning_scenarioplanning_trajectory.Send(*event);
  if (timing)
    timing->send_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - converted).count();
  if (!sent.HasValue()) {
    log.LogError() << "Failed to send final trajectory Event 12506";
    return false;
  }
  if (log_details) {
    const auto &full_end = frame.lane_driving.back();
    const auto &local_end = event->points.back().pose.position;
    double local_length_m = 0.0;
    for (std::size_t i = 1; i < event->points.size(); ++i) {
      const auto &a = event->points[i - 1].pose.position;
      const auto &b = event->points[i].pose.position;
      local_length_m += std::hypot(b.x - a.x, b.y - a.y);
    }
    log.LogInfo() << "Trajectory geometry: goal=(" << goal.x << ", "
                  << goal.y << ") full_path_end=(" << full_end.x << ", "
                  << full_end.y << ") local_end=(" << local_end.x << ", "
                  << local_end.y << ") local_length_m=" << local_length_m
                  << " local_goal_distance_m="
                  << std::hypot(local_end.x - goal.x, local_end.y - goal.y)
                  << " points=" << event->points.size();
  }
  return true;
}

template <class Bytes>
std::string routeId(const Bytes &uuid) {
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : uuid)
    stream << std::setw(2) << static_cast<unsigned>(byte);
  return stream.str();
}

template <class Time>
double stampSeconds(const Time &stamp) {
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1.0e-9;
}

planning::TrajectoryPoint egoPoint(const oss::type::nav_msgs::msg::Odometry &odom) {
  planning::TrajectoryPoint point;
  const auto &pose = odom.pose.pose;
  point.x = pose.position.x;
  point.y = pose.position.y;
  point.z = pose.position.z;
  point.orientation_x = pose.orientation.x;
  point.orientation_y = pose.orientation.y;
  point.orientation_z = pose.orientation.z;
  point.orientation_w = pose.orientation.w;
  point.longitudinal_velocity_mps = odom.twist.twist.linear.x;
  point.lateral_velocity_mps = odom.twist.twist.linear.y;
  return point;
}

std::vector<planning::PlanningPerceivedObject> perceivedObjects(
    const oss::type::autoware_perception_msgs::msg::PredictedObjects &message) {
  std::vector<planning::PlanningPerceivedObject> result;
  result.reserve(message.objects.size());
  for (const auto &source : message.objects) {
    planning::PlanningPerceivedObject target;
    target.id = routeId(source.object_id.uuid);
    if (!source.classification.empty()) {
      const auto best = std::max_element(
          source.classification.begin(), source.classification.end(),
          [](const auto &a, const auto &b) {
            return a.probability < b.probability;
          });
      target.classification = static_cast<std::uint8_t>(best->label);
    }
    target.shape_type = static_cast<int>(source.shape.type);
    target.length_m = source.shape.dimensions.x;
    target.width_m = source.shape.dimensions.y;
    const auto &kinematics = source.kinematics;
    const auto &pose = kinematics.initial_pose_with_covariance.pose;
    target.x = pose.position.x;
    target.y = pose.position.y;
    target.yaw_rad = checkpoint(pose).yaw_rad;
    target.velocity_x_mps =
        kinematics.initial_twist_with_covariance.twist.linear.x;
    target.velocity_y_mps =
        kinematics.initial_twist_with_covariance.twist.linear.y;
    for (const auto &point : source.shape.footprint.points)
      target.footprint.push_back({point.x, point.y, 0.0});
    for (const auto &path : kinematics.predicted_paths) {
      planning::CruisePredictedPath converted;
      converted.time_step_s = stampSeconds(path.time_step);
      converted.confidence = path.confidence;
      for (const auto &path_pose : path.path) {
        converted.poses.push_back({path_pose.position.x,
                                   path_pose.position.y,
                                   checkpoint(path_pose).yaw_rad});
      }
      target.predicted_paths.push_back(std::move(converted));
    }
    result.push_back(std::move(target));
  }
  return result;
}

planning::TrafficLampColor trafficColor(oss::type::TrafficLightElement_color color) {
  switch (color) {
    case oss::type::TrafficLightElement_color::RED:
      return planning::TrafficLampColor::Red;
    case oss::type::TrafficLightElement_color::AMBER:
      return planning::TrafficLampColor::Yellow;
    case oss::type::TrafficLightElement_color::GREEN:
      return planning::TrafficLampColor::Green;
    default:
      return planning::TrafficLampColor::Unknown;
  }
}

planning::TrafficLampShape trafficShape(oss::type::TrafficLightElement_shape shape) {
  switch (shape) {
    case oss::type::TrafficLightElement_shape::CIRCLE:
      return planning::TrafficLampShape::Circle;
    case oss::type::TrafficLightElement_shape::LEFT_ARROW:
      return planning::TrafficLampShape::LeftArrow;
    case oss::type::TrafficLightElement_shape::RIGHT_ARROW:
      return planning::TrafficLampShape::RightArrow;
    case oss::type::TrafficLightElement_shape::UP_ARROW:
      return planning::TrafficLampShape::UpArrow;
    default:
      return planning::TrafficLampShape::Unknown;
  }
}

void addTrafficSignals(
    planning::BehaviorVelocityMapInput &context,
    const oss::type::autoware_msgs::autoware_perception_msgs::msg::TrafficLightGroupArray &message) {
  const double stamp = stampSeconds(message.stamp);
  context.signal_stamp_s = stamp;
  for (const auto &group : message.traffic_light_groups) {
    planning::TrafficLightSignalObservation observation;
    observation.stamp_s = stamp;
    for (const auto &element : group.elements) {
      if (element.status == oss::type::TrafficLightElement_status::SOLID_OFF)
        continue;
      observation.elements.push_back({trafficColor(element.color),
                                      trafficShape(element.shape)});
    }
    if (!observation.elements.empty())
      context.intersection_signal_colors.emplace(
          group.traffic_light_group_id, observation.elements.front().color);
    planning::CrosswalkPedestrianSignal pedestrian;
    pedestrian.stamp_s = stamp;
    pedestrian.elements = observation.elements;
    context.pedestrian_signals.emplace(group.traffic_light_group_id,
                                       std::move(pedestrian));
    context.traffic_signal_observations.emplace(
        group.traffic_light_group_id, std::move(observation));
  }
}

class RouteService final : public RouteSkeleton {
 public:
  explicit RouteService(SharedState &state)
      : RouteSkeleton(ara::core::InstanceSpecifier{"PlanningAA/AA/PPort_MP2SCr"}),
        state_(state) {}

  ara::core::Future<planning_missionplanning_missionplanner_clearrouteOutput>
  planning_missionplanning_missionplanner_clearroute() override {
    std::lock_guard<std::mutex> lock(state_.mutex);
    planning::RouteSelectorRequest request;
    request.kind = planning::RouteCommandKind::Clear;
    planning_missionplanning_missionplanner_clearrouteOutput output{};
    output.res.status = responseStatus(
        state_.routes.handle(planning::RouteChannel::Main, request, nowSeconds()));
    ara::core::Promise<planning_missionplanning_missionplanner_clearrouteOutput> promise;
    promise.set_value(output);
    return promise.get_future();
  }

  ara::core::Future<planning_missionplanning_missionplanner_setlaneletrouteOutput>
  planning_missionplanning_missionplanner_setlaneletroute(
      const oss::type::tier4_planning_msgs::srv::SetLaneletRoute_Request &req) override {
    planning::RouteSelectorRequest request;
    request.kind = planning::RouteCommandKind::SetLanelet;
    request.uuid = routeUuid(req.uuid);
    request.goal = checkpoint(req.goal_pose);
    request.allow_modification = req.allow_modification;
    request.stamp_s = req.header.stamp.sec + req.header.stamp.nanosec * 1e-9;
    request.lanelet_route.goal_height_m = req.goal_pose.position.z;
    for (const auto &segment : req.segments) {
      planning::MissionRouteSection section;
      section.preferred_lanelet_id = segment.primitive_type.id;
      for (const auto &primitive : segment.primitives)
        section.primitive_lanelet_ids.push_back(primitive.id);
      request.lanelet_route.sections.push_back(std::move(section));
    }
    std::lock_guard<std::mutex> lock(state_.mutex);
    planning_missionplanning_missionplanner_setlaneletrouteOutput output{};
    output.res.status = responseStatus(state_.routes.handle(
        planning::RouteChannel::Main, std::move(request), nowSeconds()));
    ara::core::Promise<planning_missionplanning_missionplanner_setlaneletrouteOutput> promise;
    promise.set_value(output);
    return promise.get_future();
  }

  ara::core::Future<planning_missionplanning_missionplanner_setwaypointrouteOutput>
  planning_missionplanning_missionplanner_setwaypointroute(
      const oss::type::tier4_planning_msgs::srv::SetWaypointRoute_Request &req) override {
    planning::RouteSelectorRequest request;
    request.kind = planning::RouteCommandKind::SetWaypoint;
    request.uuid = routeUuid(req.uuid);
    request.goal = checkpoint(req.goal_pose);
    request.allow_modification = req.allow_modification;
    request.stamp_s = req.header.stamp.sec + req.header.stamp.nanosec * 1e-9;
    for (const auto &pose : req.waypoints)
      request.waypoints.push_back(checkpoint(pose));
    std::lock_guard<std::mutex> lock(state_.mutex);
    planning_missionplanning_missionplanner_setwaypointrouteOutput output{};
    output.res.status = handleWaypointRoute(state_, std::move(request));
    ara::core::Promise<planning_missionplanning_missionplanner_setwaypointrouteOutput> promise;
    promise.set_value(output);
    return promise.get_future();
  }

 private:
  SharedState &state_;
};

// Route selector's original state interface (Service 1814, Event 10604).
// The methods on this interface are implemented against the same coordinator
// so a caller using either route service sees one authoritative state.
class RouteStateService final : public RouteStateSkeleton {
 public:
  explicit RouteStateService(SharedState &state)
      : RouteStateSkeleton(ara::core::InstanceSpecifier{"PlanningAA/AA/PPort_MP2RS"}),
        state_(state) {}

  ara::core::Future<planning_missionplanning_missionplanner_clearrouteOutput>
  planning_missionplanning_missionplanner_clearroute() override {
    planning::RouteSelectorRequest request;
    request.kind = planning::RouteCommandKind::Clear;
    planning_missionplanning_missionplanner_clearrouteOutput output{};
    {
      std::lock_guard<std::mutex> lock(state_.mutex);
      output.res.status = responseStatus(state_.routes.handle(
          planning::RouteChannel::Main, std::move(request), nowSeconds()));
    }
    ara::core::Promise<planning_missionplanning_missionplanner_clearrouteOutput> promise;
    promise.set_value(output);
    return promise.get_future();
  }

  ara::core::Future<planning_missionplanning_missionplanner_setlaneletrouteOutput>
  planning_missionplanning_missionplanner_setlaneletroute(
      const oss::type::tier4_planning_msgs::srv::SetLaneletRoute_Request &req) override {
    planning::RouteSelectorRequest request;
    request.kind = planning::RouteCommandKind::SetLanelet;
    request.uuid = routeUuid(req.uuid);
    request.goal = checkpoint(req.goal_pose);
    request.allow_modification = req.allow_modification;
    request.stamp_s = stampSeconds(req.header.stamp);
    request.lanelet_route.goal_height_m = req.goal_pose.position.z;
    for (const auto &segment : req.segments) {
      planning::MissionRouteSection section;
      section.preferred_lanelet_id = segment.primitive_type.id;
      for (const auto &primitive : segment.primitives)
        section.primitive_lanelet_ids.push_back(primitive.id);
      request.lanelet_route.sections.push_back(std::move(section));
    }
    planning_missionplanning_missionplanner_setlaneletrouteOutput output{};
    {
      std::lock_guard<std::mutex> lock(state_.mutex);
      output.res.status = responseStatus(state_.routes.handle(
          planning::RouteChannel::Main, std::move(request), nowSeconds()));
    }
    ara::core::Promise<planning_missionplanning_missionplanner_setlaneletrouteOutput> promise;
    promise.set_value(output);
    return promise.get_future();
  }

  ara::core::Future<planning_missionplanning_missionplanner_setwaypointrouteOutput>
  planning_missionplanning_missionplanner_setwaypointroute(
      const oss::type::tier4_planning_msgs::srv::SetWaypointRoute_Request &req) override {
    planning::RouteSelectorRequest request;
    request.kind = planning::RouteCommandKind::SetWaypoint;
    request.uuid = routeUuid(req.uuid);
    request.goal = checkpoint(req.goal_pose);
    request.allow_modification = req.allow_modification;
    request.stamp_s = stampSeconds(req.header.stamp);
    for (const auto &pose : req.waypoints)
      request.waypoints.push_back(checkpoint(pose));
    planning_missionplanning_missionplanner_setwaypointrouteOutput output{};
    {
      std::lock_guard<std::mutex> lock(state_.mutex);
      output.res.status = handleWaypointRoute(state_, std::move(request));
    }
    ara::core::Promise<planning_missionplanning_missionplanner_setwaypointrouteOutput> promise;
    promise.set_value(output);
    return promise.get_future();
  }

 private:
  SharedState &state_;
};

std::uint8_t routeStateValue(planning::RouteLifecycleState value) {
  switch (value) {
    case planning::RouteLifecycleState::Initializing: return 1;
    case planning::RouteLifecycleState::Unset: return 2;
    case planning::RouteLifecycleState::Routing: return 3;
    case planning::RouteLifecycleState::Set: return 4;
    case planning::RouteLifecycleState::Rerouting: return 5;
    case planning::RouteLifecycleState::Arrived: return 6;
    case planning::RouteLifecycleState::Interrupted: return 8;
    default: return 0;
  }
}

oss::type::autoware_planning_msgs::msg::LaneletRoute routeMessage(
    const planning::RouteSelectorRoute &route) {
  oss::type::autoware_planning_msgs::msg::LaneletRoute message{};
  const double stamp = route.stamp_s > 0.0 ? route.stamp_s : nowSeconds();
  message.header.stamp.sec = static_cast<decltype(message.header.stamp.sec)>(stamp);
  message.header.stamp.nanosec = static_cast<decltype(message.header.stamp.nanosec)>(
      (stamp - static_cast<double>(message.header.stamp.sec)) * 1e9);
  message.header.frame_id = "map";
  const auto set_pose = [](const planning::RouteCheckpoint &checkpoint, auto &pose,
                           bool preserve_orientation) {
    pose.position.x = checkpoint.x;
    pose.position.y = checkpoint.y;
    pose.position.z = checkpoint.z;
    if (preserve_orientation && checkpoint.has_orientation) {
      pose.orientation.x = checkpoint.orientation_x;
      pose.orientation.y = checkpoint.orientation_y;
      pose.orientation.z = checkpoint.orientation_z;
      pose.orientation.w = checkpoint.orientation_w;
    } else {
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = std::sin(checkpoint.yaw_rad / 2.0);
      pose.orientation.w = std::cos(checkpoint.yaw_rad / 2.0);
    }
  };
  set_pose(route.start, message.start_pose, true);
  set_pose(route.goal, message.goal_pose, false);
  std::copy(route.uuid.begin(), route.uuid.end(), message.uuid.uuid.begin());
  message.allow_modification = route.allow_modification;
  for (const auto &section : route.route.sections) {
    oss::type::autoware_planning_msgs::msg::LaneletSegment segment{};
    segment.primitive_type.id = section.preferred_lanelet_id;
    segment.primitive_type.primitive_type = "lane";
    for (const auto id : section.primitive_lanelet_ids) {
      oss::type::autoware_planning_msgs::msg::LaneletPrimitive primitive{};
      primitive.id = id;
      primitive.primitive_type = "lane";
      segment.primitives.push_back(std::move(primitive));
    }
    message.segments.push_back(std::move(segment));
  }
  return message;
}

class RouteStatePublisher {
 public:
  RouteStatePublisher(SharedState &state, RouteStateService &service,
                      ara::log::Logger &log)
      : state_(state), service_(service), log_(log), worker_([this] { run(); }) {}
  ~RouteStatePublisher() { stop(); }

  void stop() {
    stopping_ = true;
    if (worker_.joinable()) worker_.join();
  }

 private:
  void run() {
    std::optional<planning::RouteLifecycleState> last;
    unsigned ticks = 0;
    unsigned route_ticks = 0;
    std::optional<planning::RouteSelectorRoute> last_route;
    while (!stopping_) {
      planning::RouteSelectorState current;
      std::optional<planning::RouteSelectorRoute> route;
      {
        std::lock_guard<std::mutex> lock(state_.mutex);
        current = state_.routes.state(planning::RouteChannel::Main);
        route = state_.routes.route(planning::RouteChannel::Main);
      }
      if (!last || *last != current.value || ++ticks >= 10) {
        const double stamp = current.stamp_s > 0.0 ? current.stamp_s : nowSeconds();
        oss::type::tier4_planning_msgs::msg::RouteState event{};
        event.stamp.sec = static_cast<decltype(event.stamp.sec)>(stamp);
        event.stamp.nanosec = static_cast<decltype(event.stamp.nanosec)>(
            (stamp - static_cast<double>(event.stamp.sec)) * 1e9);
        event.state = routeStateValue(current.value);
        const auto sent = service_.planning_missionplanning_state.Send(event);
        if (!sent.HasValue() && (!last || *last != current.value))
          log_.LogWarn() << "Route state Event 10604 has no subscriber yet";
        last = current.value;
        ticks = 0;
      }
      if (route && !route->route.sections.empty()) {
        const bool changed = !last_route || last_route->uuid != route->uuid ||
            last_route->stamp_s != route->stamp_s ||
            last_route->route.sections.size() != route->route.sections.size();
        if (changed || ++route_ticks >= 10) {
          const auto event = routeMessage(*route);
          const auto sent = service_.planning_missionplanning_route.Send(event);
          if (!sent.HasValue() && changed)
            log_.LogWarn() << "LaneletRoute Event 10602 has no subscriber yet";
          last_route = std::move(route);
          route_ticks = 0;
        }
      } else {
        last_route.reset();
        route_ticks = 0;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  SharedState &state_;
  RouteStateService &service_;
  ara::log::Logger &log_;
  std::atomic<bool> stopping_{false};
  std::thread worker_;
};

class InputService {
 public:
  InputService(SharedState &state, ara::log::Logger &log)
      : state_(state), log_(log) {}

  bool start() {
    const auto gate = gate_;
    const auto found = InputProxy::StartFindService(
        [this, gate](ara::com::ServiceHandleContainer<InputProxy::HandleType> handles,
               ara::com::FindServiceHandle handle) {
          std::lock_guard<std::recursive_mutex> guard(gate->mutex);
          if (!gate->active) return;
          if (handles.empty() || proxy_) return;
          proxy_ = std::make_unique<InputProxy>(handles.front());
          find_handle_ = std::make_unique<ara::com::FindServiceHandle>(handle);
#define SUBSCRIBE(name) \
          { auto handler = proxy_->name.SetReceiveHandler([this, gate] { \
              std::lock_guard<std::recursive_mutex> callback_guard(gate->mutex); \
              if (gate->active) receive##name(); }); \
            auto subscription = proxy_->name.Subscribe(1); \
            if (!handler.HasValue() || !subscription.HasValue()) \
              log_.LogError() << "Failed to subscribe to " #name; }
          INPUT_EVENTS(SUBSCRIBE)
#undef SUBSCRIBE
          log_.LogInfo() << "Subscribed to Service Creator 4308";
        }, ara::core::InstanceSpecifier{"PlanningAA/AA/RPort_SCr2MP"});
    if (!found.HasValue()) {
      log_.LogError() << "Could not discover Service Creator";
      return false;
    }
    find_started_ = true;
    return true;
  }

  void stop() {
    {
      std::lock_guard<std::recursive_mutex> guard(gate_->mutex);
      gate_->active = false;
    }
    if (!proxy_) {
      if (find_started_) {
        static_cast<void>(InputProxy::StartFindService(
            [](ara::com::ServiceHandleContainer<InputProxy::HandleType>,
               ara::com::FindServiceHandle) {},
            ara::core::InstanceSpecifier{"PlanningAA/AA/RPort_SCr2MP"}));
      }
      return;
    }
    if (find_handle_) proxy_->StopFindService(*find_handle_);
#define UNSUBSCRIBE(name) proxy_->name.Unsubscribe();
    INPUT_EVENTS(UNSUBSCRIBE)
#undef UNSUBSCRIBE
    proxy_.reset();
    find_handle_.reset();
  }

 private:
  void onSample(const oss::type::rosgraph_msgs::msg::Clock &sample) {
    std::lock_guard<std::mutex> lock(state_.mutex);
    state_.clock = sample;
  }
  void onSample(const oss::type::geometry_msgs::msg::AccelWithCovarianceStamped &sample) {
    std::lock_guard<std::mutex> lock(state_.mutex);
    state_.acceleration = sample;
  }
  void onSample(const oss::type::autoware_map_msgs::msg::LaneletMapBin &map) {
    std::lock_guard<std::mutex> lock(state_.mutex);
    const bool duplicate = state_.map && state_.map->data == map.data &&
        state_.map->header.frame_id == map.header.frame_id;
    const char *static_map_env = std::getenv("PLANNING_AA_STATIC_MAP");
    const bool static_map = static_map_env && std::string(static_map_env) == "1";
    // The sample Lanelet map is fixed for the lifetime of this process. Its
    // serialized bytes can vary between publications, and a second sample can
    // arrive before the first decode finishes. Decode only the first sample.
    const bool same_static_map = static_map && state_.map &&
        !state_.map->data.empty();
    std::fprintf(stderr, "PLAA_MAP received bytes=%zu duplicate=%d static_duplicate=%d\n",
                 map.data.size(), duplicate ? 1 : 0, same_static_map ? 1 : 0);
    if (duplicate || same_static_map) return;
    state_.map = map;
    ++state_.map_generation;
    state_.changed.notify_one();
  }
  void onSample(const oss::type::nav_msgs::msg::Odometry &odom) {
    planning::MissionPlannerEgo ego;
    ego.pose = checkpoint(odom.pose.pose);
    ego.speed_mps = odom.twist.twist.linear.x;
    ego.lateral_speed_mps = odom.twist.twist.linear.y;
    ego.vertical_speed_mps = odom.twist.twist.linear.z;
    std::lock_guard<std::mutex> lock(state_.mutex);
    state_.odometry = odom;
    ++state_.odometry_generation;
    state_.routes.onEgo(ego, nowSeconds());
    state_.changed.notify_one();
  }
  void onSample(const oss::type::autoware_perception_msgs::msg::PredictedObjects &sample) {
    std::lock_guard<std::mutex> lock(state_.mutex);
    state_.objects = sample;
  }
  void onSample(const oss::type::sensor_msgs::msg::PointCloud2 &sample) {
    std::lock_guard<std::mutex> lock(state_.mutex);
    state_.pointcloud = sample;
  }
  void onSample(const oss::type::nav_msgs::msg::OccupancyGrid &sample) {
    std::lock_guard<std::mutex> lock(state_.mutex);
    state_.occupancy_grid = sample;
  }
  void onSample(const oss::type::autoware_msgs::autoware_perception_msgs::msg::TrafficLightGroupArray &sample) {
    std::lock_guard<std::mutex> lock(state_.mutex);
    state_.traffic_signals = sample;
  }
  void onSample(const oss::type::tier4_planning_msgs::msg::LateralOffset &sample) {
    std::lock_guard<std::mutex> lock(state_.mutex);
    state_.lateral_offset = sample;
  }
  void onSample(const oss::type::tier4_planning_msgs::msg::VelocityLimit &sample) {
    std::lock_guard<std::mutex> lock(state_.mutex);
    state_.velocity_limit = sample;
  }
  void onSample(const oss::type::autoware_adapi_v1_msgs::msg::OperationModeState &mode) {
    std::lock_guard<std::mutex> lock(state_.mutex);
    state_.operation_mode = mode;
    state_.routes.onOperationMode({
        mode.mode == oss::type::OperationModeState_mode::AUTONOMOUS,
        mode.is_autoware_control_enabled});
  }
#define RECEIVE(name) \
  void receive##name() { \
    auto result = proxy_->name.GetNewSamples([this](auto sample) { onSample(*sample); }); \
    if (!result.HasValue()) log_.LogError() << "Failed to receive " #name; \
  }
  INPUT_EVENTS(RECEIVE)
#undef RECEIVE

  SharedState &state_;
  ara::log::Logger &log_;
  struct Gate {
    std::recursive_mutex mutex;
    bool active{true};
  };
  std::shared_ptr<Gate> gate_ = std::make_shared<Gate>();
  bool find_started_{false};
  std::unique_ptr<InputProxy> proxy_;
  std::unique_ptr<ara::com::FindServiceHandle> find_handle_;
};

class FrameProducer {
 public:
  FrameProducer(SharedState &state, TrajectorySkeleton &trajectory,
                ScenarioSkeleton &scenario_trajectory,
                TurnSkeleton &turn, HazardSkeleton &hazard,
                MetricsSkeleton &metrics, RemainingSkeleton &remaining,
                adaptive::DiagnosticOutput &diagnostic_output,
                adaptive::RouteVisualizationOutput &route_visualization,
                ara::log::Logger &log)
      : state_(state), trajectory_(trajectory), scenario_trajectory_(scenario_trajectory),
        turn_(turn), hazard_(hazard),
        metrics_(metrics), remaining_(remaining),
        diagnostic_output_(diagnostic_output),
        route_visualization_(route_visualization), log_(log),
        worker_([this] { run(); }) {}

  void stop() {
    {
      std::lock_guard<std::mutex> lock(state_.mutex);
      state_.stopping = true;
    }
    state_.changed.notify_one();
    if (worker_.joinable()) worker_.join();
  }

  ~FrameProducer() { stop(); }

 private:
  struct Snapshot {
    oss::type::nav_msgs::msg::Odometry odometry;
    std::optional<oss::type::geometry_msgs::msg::AccelWithCovarianceStamped> acceleration;
    std::optional<oss::type::autoware_perception_msgs::msg::PredictedObjects> objects;
    std::optional<oss::type::autoware_msgs::autoware_perception_msgs::msg::TrafficLightGroupArray> traffic_signals;
    std::optional<oss::type::tier4_planning_msgs::msg::VelocityLimit> velocity_limit;
    oss::type::autoware_adapi_v1_msgs::msg::OperationModeState mode;
    planning::RouteSelectorRoute route;
  };

  void run() {
    const char *timing_env = std::getenv("PLANNING_AA_TIMING");
    const bool trace_timing = timing_env && std::string(timing_env) == "1";
    const char *simulation_env = std::getenv("PLANNING_AA_SIMULATION");
    const bool simulation = simulation_env && std::string(simulation_env) == "1";
    std::fprintf(stderr, "PLAA_CONFIG simulation=%d\n", simulation ? 1 : 0);
    planning::PlanningFinalPipelineCore final_pipeline;
    final_pipeline.reset();
    std::unique_ptr<planning::PlanningPipelineMapAdapter> pipeline;
    std::shared_ptr<planning::LaneletBinAdapter> scenario_map;
    std::unique_ptr<planning::PlanningEvaluatorMapAdapter> evaluator;
    std::unique_ptr<planning::RemainingDistanceMapAdapter> remaining_calculator;
    std::uint64_t map_generation = 0;
    std::uint64_t odometry_generation = 0;
    std::uint64_t frame_count = 0;
    double last_aux_stamp_s = -std::numeric_limits<double>::infinity();
    std::uint64_t incomplete_count = 0;
    std::uint64_t rejected_count = 0;
    bool sent_once = false;
    bool avoidance_active = false;
    std::size_t last_avoidance_objects = 0;
    std::size_t last_avoidance_inputs = 0;
    std::string visualized_route_id;
    auto last_visualized = std::chrono::steady_clock::time_point{};
    std::string map_frame_id;
    for (;;) {
      std::vector<std::uint8_t> map_data;
      bool map_changed = false;
      std::string candidate_map_frame_id;
      std::optional<Snapshot> snapshot;
      std::chrono::steady_clock::time_point snapshot_taken;
      {
        std::unique_lock<std::mutex> lock(state_.mutex);
        state_.changed.wait(lock, [&] {
          return state_.stopping || state_.map_generation != map_generation ||
                 state_.odometry_generation != odometry_generation;
        });
        if (state_.stopping) return;
        map_changed = state_.map_generation != map_generation;
        if (map_changed) {
          map_generation = state_.map_generation;
          if (state_.map) {
            map_data = state_.map->data;
            candidate_map_frame_id = state_.map->header.frame_id.c_str();
          }
        }
        if (state_.odometry_generation != odometry_generation) {
          odometry_generation = state_.odometry_generation;
          const auto route = state_.routes.route(planning::RouteChannel::Main);
          if (state_.odometry && state_.operation_mode && route &&
              !route->route.sections.empty()) {
            snapshot.emplace(Snapshot{*state_.odometry, state_.acceleration,
                                      state_.objects, state_.traffic_signals,
                                      state_.velocity_limit,
                                      *state_.operation_mode, *route});
            snapshot_taken = std::chrono::steady_clock::now();
          }
        }
      }
      if (map_changed) {
        if (!map_data.empty()) {
          try {
            std::fprintf(stderr, "PLAA_MAP decode_start bytes=%zu generation=%llu\n",
                         map_data.size(),
                         static_cast<unsigned long long>(map_generation));
            auto next_pipeline = std::make_unique<planning::PlanningPipelineMapAdapter>(
                map_data, std::vector<planning::BehaviorVelocityModule>{
                    planning::BehaviorVelocityModule::CrosswalkDefault,
                    planning::BehaviorVelocityModule::Walkway,
                    planning::BehaviorVelocityModule::TrafficLight,
                    planning::BehaviorVelocityModule::IntersectionStuck,
                    planning::BehaviorVelocityModule::IntersectionCollision,
                    planning::BehaviorVelocityModule::MergeFromPrivate,
                    planning::BehaviorVelocityModule::BlindSpot,
                    planning::BehaviorVelocityModule::DetectionArea,
                    planning::BehaviorVelocityModule::NoStoppingArea,
                    planning::BehaviorVelocityModule::StopSign,
                    planning::BehaviorVelocityModule::RunOut});
            auto next_map = std::make_shared<planning::LaneletBinAdapter>(map_data);
            auto next_evaluator = std::make_unique<planning::PlanningEvaluatorMapAdapter>(*next_map);
            auto next_remaining = std::make_unique<planning::RemainingDistanceMapAdapter>(*next_map);
            pipeline = std::move(next_pipeline);
            scenario_map = std::move(next_map);
            evaluator = std::move(next_evaluator);
            remaining_calculator = std::move(next_remaining);
            map_frame_id = std::move(candidate_map_frame_id);
            final_pipeline.reset();
            {
              std::lock_guard<std::mutex> lock(state_.mutex);
              if (state_.map_generation == map_generation) {
                state_.route_map = scenario_map;
                state_.routes.setMapReady(true, nowSeconds());
              }
            }
            log_.LogInfo() << "Lanelet map ready for live planning";
            std::fprintf(stderr, "PLAA_MAP ready generation=%llu\n",
                         static_cast<unsigned long long>(map_generation));
          } catch (const std::exception &error) {
            log_.LogError() << "Cannot decode Lanelet map: " << error.what();
            std::fprintf(stderr, "PLAA_MAP decode_failed error=%s\n", error.what());
          }
        } else {
          pipeline.reset();
          evaluator.reset();
          remaining_calculator.reset();
          scenario_map.reset();
          final_pipeline.reset();
          map_frame_id.clear();
          std::lock_guard<std::mutex> lock(state_.mutex);
          state_.route_map.reset();
          state_.routes.setMapReady(false, nowSeconds());
        }
      }
      if (!snapshot || !pipeline || !scenario_map || !evaluator || !remaining_calculator) continue;
      const auto &odom = snapshot->odometry;
      const double stamp = stampSeconds(odom.header.stamp);
      if (!std::isfinite(stamp) || stamp <= 0.0 || odom.header.frame_id.empty() ||
          map_frame_id.empty() || odom.header.frame_id != map_frame_id.c_str())
        continue;
      const auto ego = egoPoint(odom);
      if (!std::isfinite(ego.x) || !std::isfinite(ego.y) ||
          !std::isfinite(ego.longitudinal_velocity_mps)) continue;
      planning::PlanningPipelineFrame source;
      source.timestamp_s = stamp;
      source.ego = ego;
      source.ego_speed_mps = ego.longitudinal_velocity_mps;
      planning::BehaviorVelocityMapInput context;
      context.is_simulation = simulation;
      context.ego_x = ego.x;
      context.ego_y = ego.y;
      context.ego_velocity_mps = source.ego_speed_mps;
      context.ego_acceleration_mps2 = snapshot->acceleration &&
          std::abs(stamp - stampSeconds(snapshot->acceleration->header.stamp)) <= 0.5 ?
          snapshot->acceleration->accel.accel.linear.x : 0.0;
      if (!std::isfinite(context.ego_acceleration_mps2))
        context.ego_acceleration_mps2 = 0.0;
      context.now_s = stamp;
      context.crosswalk_auto_activate_when_safe = true;
      planning::PlanningCruisePerceptionFrame cruise;
      if (snapshot->objects &&
          std::abs(stamp - stampSeconds(snapshot->objects->header.stamp)) <= 0.5) {
        const auto objects = perceivedObjects(*snapshot->objects);
        planning::applyBehaviorVelocityPerceptionBatch(context, objects);
        cruise.objects = objects;
        cruise.stamp_s = stampSeconds(snapshot->objects->header.stamp);
      }
      if (snapshot->traffic_signals &&
          std::abs(stamp - stampSeconds(snapshot->traffic_signals->stamp)) <= 1.0)
        addTrafficSignals(context, *snapshot->traffic_signals);
      const auto &route = snapshot->route;
      const planning::BehaviorGoalPose goal{
          route.goal.x, route.goal.y, route.goal.yaw_rad};
      try {
        const auto planning_started = std::chrono::steady_clock::now();
        const auto planned = pipeline->onRouteFrame(
            source, route.route, context, cruise, {}, goal);
        if (cruise.objects.size() != last_avoidance_objects ||
            planned.avoidance_perception_input_count != last_avoidance_inputs) {
          const auto *first_object = cruise.objects.empty() ?
              nullptr : &cruise.objects.front();
          std::fprintf(stderr,
              "PLAA_AVOIDANCE_INPUT objects=%zu selected=%zu candidates=%zu "
              "first_x=%.3f first_y=%.3f first_class=%u first_speed_mps=%.3f "
              "odom_s=%.6f\n",
              cruise.objects.size(), planned.avoidance_perception_input_count,
              planned.behavior_action ?
                  planned.behavior_action->avoidance_candidate_count : 0,
              first_object ? first_object->x : 0.0,
              first_object ? first_object->y : 0.0,
              first_object ? static_cast<unsigned>(first_object->classification) : 0U,
              first_object ? std::hypot(
                  first_object->velocity_x_mps,
                  first_object->velocity_y_mps) : 0.0,
              stamp);
          last_avoidance_objects = cruise.objects.size();
          last_avoidance_inputs = planned.avoidance_perception_input_count;
        }
        const bool avoiding = planned.behavior_action &&
            planned.behavior_action->kind ==
                planning::BehaviorRouteActionKind::StaticAvoidance;
        if (avoiding != avoidance_active) {
          std::fprintf(stderr,
              "PLAA_AVOIDANCE active=%d objects=%zu candidates=%zu "
              "route_id=%s odom_s=%.6f\n",
              avoiding ? 1 : 0, planned.avoidance_perception_input_count,
              planned.behavior_action ?
                  planned.behavior_action->avoidance_candidate_count : 0,
              routeId(route.uuid).c_str(), stamp);
          avoidance_active = avoiding;
        }
        const auto route_planned = std::chrono::steady_clock::now();
        if (planned.pipeline.status != planning::PlanningPipelineStatus::Complete ||
            planned.pipeline.cruise_stopped.size() < 2) {
          if (trace_timing) {
            std::fprintf(stderr,
                "PLAA_TIMING result=incomplete wall_s=%.6f odom_s=%.6f "
                "odom_age_ms=%.1f prep_ms=%.1f route_ms=%.1f "
                "ego_speed_mps=%.3f\n",
                nowSeconds(), stamp, (nowSeconds() - stamp) * 1000.0,
                std::chrono::duration<double, std::milli>(
                    planning_started - snapshot_taken).count(),
                std::chrono::duration<double, std::milli>(
                    route_planned - planning_started).count(),
                source.ego_speed_mps);
          }
          if (++incomplete_count == 1 || incomplete_count % 100 == 0)
            log_.LogWarn() << "Route frame did not produce a lane trajectory";
          continue;
        }
        const bool emit_auxiliary = stamp - last_aux_stamp_s >= 0.1;
        if (emit_auxiliary) {
          last_aux_stamp_s = stamp;
          const auto signal = adaptive::decideBehaviorSignals(
              *scenario_map, route.route, checkpoint(odom.pose.pose),
              source.ego_speed_mps, planned.behavior_action);
          oss::type::adaptive_planning::msg::TurnCommand turn_sample{};
          turn_sample.stamp = odom.header.stamp;
          turn_sample.command = signal.turn;
          oss::type::adaptive_planning::msg::HazardCommand hazard_sample{};
          hazard_sample.stamp = odom.header.stamp;
          hazard_sample.command = signal.hazard;
          if (!turn_.planning_turn_indicators_cmd.Send(turn_sample).HasValue() ||
              !hazard_.planning_hazard_lights_cmd.Send(hazard_sample).HasValue())
            log_.LogWarn() << "Failed to send behavior signal commands";
        }
        planning::PlanningFinalFrame frame;
        frame.now_s = stamp;
        frame.odometry_stamp_s = stamp;
        frame.route_id = routeId(route.uuid);
        frame.ego = ego;
        frame.ego_speed_mps = source.ego_speed_mps;
        frame.ego_acceleration_mps2 = context.ego_acceleration_mps2;
        frame.lane_driving = planned.pipeline.cruise_stopped;
        const auto visualized_at = std::chrono::steady_clock::now();
        if (frame.route_id != visualized_route_id ||
            visualized_at - last_visualized >= std::chrono::seconds(1)) {
          // Keep the global route marker on the unshifted mission path while
          // the selected local path and final trajectory avoid an object.
          if (route_visualization_.send(
                  planned.route_goal_refined_path.points)) {
            visualized_route_id = frame.route_id;
            last_visualized = visualized_at;
            const auto &end = planned.route_goal_refined_path.points.back();
            std::fprintf(stderr,
                "PLAA_ROUTE points=%zu end_x=%.3f end_y=%.3f "
                "goal_x=%.3f goal_y=%.3f end_goal_distance_m=%.3f\n",
              planned.route_goal_refined_path.points.size(), end.x, end.y,
                route.goal.x, route.goal.y,
                std::hypot(end.x - route.goal.x, end.y - route.goal.y));
          }
        }
        frame.autonomous = snapshot->mode.mode ==
            oss::type::OperationModeState_mode::AUTONOMOUS;
        frame.control_enabled = snapshot->mode.is_autoware_control_enabled;
        if (snapshot->velocity_limit &&
            std::isfinite(snapshot->velocity_limit->max_velocity) &&
            snapshot->velocity_limit->max_velocity >= 0.0F) {
          const auto &limit = *snapshot->velocity_limit;
          frame.maximum_velocity_mps = limit.max_velocity;
          if (limit.use_constraints &&
              std::isfinite(limit.constraints.min_acceleration) &&
              limit.constraints.min_acceleration < 0.0F &&
              std::isfinite(limit.constraints.max_jerk) &&
              limit.constraints.max_jerk > 0.0F &&
              std::isfinite(limit.constraints.min_jerk) &&
              limit.constraints.min_jerk < 0.0F) {
            frame.external_limit_use_constraints = true;
            frame.external_min_acceleration_mps2 = limit.constraints.min_acceleration;
            frame.external_max_jerk_mps3 = limit.constraints.max_jerk;
            frame.external_min_jerk_mps3 = limit.constraints.min_jerk;
          }
        }
        std::vector<std::int64_t> lane_ids;
        for (const auto &section : route.route.sections)
          lane_ids.push_back(section.preferred_lanelet_id);
        frame.position = scenario_map->scenarioPosition(
            ego.x, ego.y, checkpoint(odom.pose.pose).yaw_rad,
            route.goal.x, route.goal.y,
            lane_ids);
        ++frame_count;
        FinalFrameTiming timing;
        const bool published = publishFinalFrame(final_pipeline, trajectory_, scenario_trajectory_,
                              metrics_, remaining_, *evaluator, *remaining_calculator,
                              route.route, cruise.objects,
                              diagnostic_output_, log_, frame,
                              std::string(odom.header.frame_id.c_str()),
                              route.goal, !sent_once || frame_count % 100 == 0,
                              emit_auxiliary,
                              &timing);
        if (trace_timing) {
          const double wall_s = nowSeconds();
          std::fprintf(stderr,
              "PLAA_TIMING result=%s frame=%llu wall_s=%.6f odom_s=%.6f "
              "odom_age_ms=%.1f prep_ms=%.1f route_ms=%.1f final_ms=%.1f "
              "conversion_ms=%.1f send_ms=%.1f total_ms=%.1f "
              "smoother_ms=%.1f qp_first_ms=%.1f qp_retry_ms=%.1f "
              "qp_first_iter=%d qp_iter=%d qp_points=%zu "
              "qp_backend=%s qp_retried=%d qp_solved=%d "
              "ego_speed_mps=%.3f ego_x=%.3f ego_y=%.3f\n",
              published ? "sent" : "rejected",
              static_cast<unsigned long long>(frame_count), wall_s, stamp,
              (wall_s - stamp) * 1000.0,
              std::chrono::duration<double, std::milli>(
                  planning_started - snapshot_taken).count(),
              std::chrono::duration<double, std::milli>(
                  route_planned - planning_started).count(),
              timing.pipeline_ms, timing.conversion_ms, timing.send_ms,
              std::chrono::duration<double, std::milli>(
                  std::chrono::steady_clock::now() - snapshot_taken).count(),
              timing.smoother_ms, timing.qp_first_solve_ms,
              timing.qp_retry_ms, timing.qp_first_iterations,
              timing.qp_iterations, timing.qp_points,
              timing.qp_backend == planning::JerkQpBackend::ProxQP ? "proxqp" : "admm",
              timing.qp_retried ? 1 : 0, timing.qp_solved ? 1 : 0,
              source.ego_speed_mps, ego.x, ego.y);
        }
        if (frame_count == 1 || frame_count % 50 == 0) {
          const auto now = std::chrono::steady_clock::now();
          log_.LogWarn() << "Planning frame timing: route_ms=" <<
              std::chrono::duration<double, std::milli>(route_planned - planning_started).count()
              << " final_ms=" << timing.pipeline_ms
              << " conversion_ms=" << timing.conversion_ms
              << " send_ms=" << timing.send_ms
              << " total_ms=" <<
              std::chrono::duration<double, std::milli>(now - planning_started).count();
        }
        if (published) {
          if (!sent_once || frame_count % 100 == 0)
            log_.LogInfo() << "Published final trajectory Event 12506";
          sent_once = true;
        } else if (++rejected_count == 1 || rejected_count % 100 == 0) {
          log_.LogWarn() << "Final trajectory was not published; check validation and subscriber";
        }
      } catch (const std::exception &error) {
        log_.LogError() << "Live planning frame failed: " << error.what();
      }
    }
  }

  SharedState &state_;
  TrajectorySkeleton &trajectory_;
  ScenarioSkeleton &scenario_trajectory_;
  TurnSkeleton &turn_;
  HazardSkeleton &hazard_;
  MetricsSkeleton &metrics_;
  RemainingSkeleton &remaining_;
  adaptive::DiagnosticOutput &diagnostic_output_;
  adaptive::RouteVisualizationOutput &route_visualization_;
  ara::log::Logger &log_;
  std::thread worker_;
};

bool runApplication(const sigset_t &shutdown_signals) {
  auto &log = ara::log::CreateLogger("PLAA", "Planning AA communication");
  SharedState state;
  RouteService routes(state);
  RouteStateService route_state(state);
  TrajectorySkeleton trajectory(ara::core::InstanceSpecifier{"PlanningAA/AA/PPort_PV2SCr"});
  ScenarioSkeleton scenario_trajectory(ara::core::InstanceSpecifier{"PlanningAA/AA/PPort_SS2VS"});
  TurnSkeleton turn(ara::core::InstanceSpecifier{"PlanningAA/AA/PPort_AA2SCrTurn"});
  HazardSkeleton hazard(ara::core::InstanceSpecifier{"PlanningAA/AA/PPort_AA2SCrHazard"});
  MetricsSkeleton metrics(ara::core::InstanceSpecifier{"PlanningAA/AA/PPort_AA2SCrMetrics"});
  RemainingSkeleton remaining(ara::core::InstanceSpecifier{"PlanningAA/AA/PPort_AA2SCrRemaining"});
  auto route_offer = routes.OfferService();
  auto state_offer = route_state.OfferService();
  auto trajectory_offer = trajectory.OfferService();
  auto scenario_offer = scenario_trajectory.OfferService();
  auto turn_offer = turn.OfferService();
  auto hazard_offer = hazard.OfferService();
  auto metrics_offer = metrics.OfferService();
  auto remaining_offer = remaining.OfferService();
  if (!route_offer.HasValue() || !state_offer.HasValue() ||
      !trajectory_offer.HasValue() || !scenario_offer.HasValue() ||
      !turn_offer.HasValue() || !hazard_offer.HasValue() ||
      !metrics_offer.HasValue() || !remaining_offer.HasValue()) {
    log.LogError() << "Could not offer planning services";
    if (trajectory_offer.HasValue()) trajectory.StopOfferService();
    if (scenario_offer.HasValue()) scenario_trajectory.StopOfferService();
    if (turn_offer.HasValue()) turn.StopOfferService();
    if (hazard_offer.HasValue()) hazard.StopOfferService();
    if (metrics_offer.HasValue()) metrics.StopOfferService();
    if (remaining_offer.HasValue()) remaining.StopOfferService();
    if (state_offer.HasValue()) route_state.StopOfferService();
    if (route_offer.HasValue()) routes.StopOfferService();
    return false;
  }
  RouteStatePublisher state_publisher(state, route_state, log);
  InputService inputs(state, log);
  if (!inputs.start()) {
    state_publisher.stop();
    trajectory.StopOfferService();
    scenario_trajectory.StopOfferService();
    turn.StopOfferService();
    hazard.StopOfferService();
    metrics.StopOfferService();
    remaining.StopOfferService();
    route_state.StopOfferService();
    routes.StopOfferService();
    return false;
  }
  const char *diagnostic_target = std::getenv("PLANNING_AA_DIAGNOSTIC_TARGET");
  adaptive::DiagnosticOutput diagnostic_output(
      diagnostic_target ? diagnostic_target : "");
  const char *route_visualization_target =
      std::getenv("PLANNING_AA_ROUTE_VISUALIZATION_TARGET");
  adaptive::RouteVisualizationOutput route_visualization(
      route_visualization_target ? route_visualization_target : "");
  if (diagnostic_target && !diagnostic_output.enabled())
    log.LogWarn() << "Invalid PLANNING_AA_DIAGNOSTIC_TARGET; ROS diagnostics disabled";
  FrameProducer producer(state, trajectory, scenario_trajectory, turn, hazard,
                         metrics, remaining, diagnostic_output,
                         route_visualization, log);
  bool reported = false;
  {
    ara::exec::ExecutionClient execution;
    reported = execution.ReportExecutionState(ara::exec::ExecutionState::kRunning).HasValue();
  }
  if (!reported) {
    log.LogError() << "Could not report Running";
    inputs.stop();
    producer.stop();
    state_publisher.stop();
    trajectory.StopOfferService();
    scenario_trajectory.StopOfferService();
    turn.StopOfferService();
    hazard.StopOfferService();
    metrics.StopOfferService();
    remaining.StopOfferService();
    route_state.StopOfferService();
    routes.StopOfferService();
    return false;
  }
  log.LogInfo() << "Planning services active";
  log.LogInfo() << "Waiting for map, Lanelet route, odometry and operation mode";
  int signal_number = 0;
  const int wait_result = sigwait(&shutdown_signals, &signal_number);
  inputs.stop();
  producer.stop();
  state_publisher.stop();
  trajectory.StopOfferService();
  scenario_trajectory.StopOfferService();
  turn.StopOfferService();
  hazard.StopOfferService();
  metrics.StopOfferService();
  remaining.StopOfferService();
  route_state.StopOfferService();
  routes.StopOfferService();
  return wait_result == 0;
}
}  // namespace

int main() {
  sigset_t shutdown_signals;
  sigemptyset(&shutdown_signals);
  sigaddset(&shutdown_signals, SIGINT);
  sigaddset(&shutdown_signals, SIGTERM);
  if (sigprocmask(SIG_BLOCK, &shutdown_signals, nullptr) != 0) return EXIT_FAILURE;
  if (!ara::core::Initialize().HasValue()) return EXIT_FAILURE;
  const bool ran = runApplication(shutdown_signals);
  const bool deinitialized = ara::core::Deinitialize().HasValue();
  return ran && deinitialized ? EXIT_SUCCESS : EXIT_FAILURE;
}
