// SPDX-License-Identifier: Apache-2.0
#include "ara/core/initialization.h"
#include "ara/exec/execution_client.h"
#include "ara/log/logger.h"
#include "oss/srv/SCr2MP/mp_si_servicecreator_proxy.h"
#include "oss/srv/MP2SCr/scr_si_missionplanner_skeleton.h"
#include "oss/srv/MP2RS/rs_si_missionplanner_skeleton.h"
#include "oss/srv/PV2SCr/scr_si_planningvalidator_skeleton.h"
#include "planning/behavior_velocity_perception_bridge.hpp"
#include "planning/lanelet_bin_adapter.hpp"
#include "planning/planning_final_pipeline_core.hpp"
#include "planning/planning_pipeline_map_adapter.hpp"
#include "planning/planning_route_coordinator.hpp"
#include "trajectory_output.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>

namespace {
using InputProxy = oss::srv::SCr2MP::proxy::MP_SI_ServiceCreatorProxy;
using RouteSkeleton = oss::srv::MP2SCr::skeleton::SCr_SI_missionplannerSkeleton;
using RouteStateSkeleton = oss::srv::MP2RS::skeleton::RS_SI_missionplannerSkeleton;
using TrajectorySkeleton = oss::srv::PV2SCr::skeleton::SCr_SI_planningvalidatorSkeleton;

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
  return {pose.position.x, pose.position.y,
          std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
          pose.position.z};
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
    if (!route_map) return std::nullopt;
    std::vector<planning::RouteCheckpoint> checkpoints;
    checkpoints.reserve(waypoints.size() + 2);
    checkpoints.push_back(start);
    checkpoints.insert(checkpoints.end(), waypoints.begin(), waypoints.end());
    checkpoints.push_back(goal);
    return route_map->routeThroughCheckpoints(checkpoints);
  }) {}

  std::mutex mutex;
  std::condition_variable changed;
  bool stopping = false;
  std::uint64_t map_generation = 0;
  std::uint64_t odometry_generation = 0;
  std::shared_ptr<planning::LaneletBinAdapter> route_map;
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

bool publishFinalFrame(planning::PlanningFinalPipelineCore &pipeline,
                       TrajectorySkeleton &trajectory,
                       ara::log::Logger &log,
                       const planning::PlanningFinalFrame &frame,
                       const std::string &frame_id) {
  const auto result = pipeline.onFrame(frame);
  const auto event = adaptive::makeTrajectoryEvent(result, frame.now_s, frame_id);
  if (!event) return false;
  const auto sent = trajectory.planning_scenarioplanning_trajectory.Send(*event);
  if (!sent.HasValue()) {
    log.LogError() << "Failed to send final trajectory Event 12506";
    return false;
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
    output.res.status = responseStatus(state_.routes.handle(
        planning::RouteChannel::Main, std::move(request), nowSeconds()));
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
      output.res.status = responseStatus(state_.routes.handle(
          planning::RouteChannel::Main, std::move(request), nowSeconds()));
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
    while (!stopping_) {
      planning::RouteSelectorState current;
      {
        std::lock_guard<std::mutex> lock(state_.mutex);
        current = state_.routes.state(planning::RouteChannel::Main);
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
                ara::log::Logger &log)
      : state_(state), trajectory_(trajectory), log_(log), worker_([this] { run(); }) {}

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
    planning::PlanningFinalPipelineCore final_pipeline;
    final_pipeline.reset();
    std::unique_ptr<planning::PlanningPipelineMapAdapter> pipeline;
    std::shared_ptr<planning::LaneletBinAdapter> scenario_map;
    std::uint64_t map_generation = 0;
    std::uint64_t odometry_generation = 0;
    std::uint64_t frame_count = 0;
    std::uint64_t incomplete_count = 0;
    std::uint64_t rejected_count = 0;
    bool sent_once = false;
    std::string map_frame_id;
    for (;;) {
      std::vector<std::uint8_t> map_data;
      bool map_changed = false;
      std::optional<Snapshot> snapshot;
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
            map_frame_id = state_.map->header.frame_id.c_str();
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
          }
        }
      }
      if (map_changed) {
        pipeline.reset();
        scenario_map.reset();
        final_pipeline.reset();
        {
          std::lock_guard<std::mutex> lock(state_.mutex);
          state_.route_map.reset();
          state_.routes.setMapReady(false, nowSeconds());
        }
        if (!map_data.empty()) {
          try {
            pipeline = std::make_unique<planning::PlanningPipelineMapAdapter>(
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
            scenario_map = std::make_shared<planning::LaneletBinAdapter>(map_data);
            {
              std::lock_guard<std::mutex> lock(state_.mutex);
              if (state_.map_generation == map_generation) {
                state_.route_map = scenario_map;
                state_.routes.setMapReady(true, nowSeconds());
              }
            }
            log_.LogInfo() << "Lanelet map ready for live planning";
          } catch (const std::exception &error) {
            pipeline.reset();
            scenario_map.reset();
            log_.LogError() << "Cannot decode Lanelet map: " << error.what();
          }
        }
      }
      if (!snapshot || !pipeline || !scenario_map) continue;
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
        const auto planned = pipeline->onRouteFrame(
            source, route.route, context, cruise, {}, goal);
        if (planned.pipeline.status != planning::PlanningPipelineStatus::Complete ||
            planned.pipeline.cruise_stopped.size() < 2) {
          if (++incomplete_count == 1 || incomplete_count % 100 == 0)
            log_.LogWarn() << "Route frame did not produce a lane trajectory";
          continue;
        }
        planning::PlanningFinalFrame frame;
        frame.now_s = stamp;
        frame.odometry_stamp_s = stamp;
        frame.route_id = routeId(route.uuid);
        frame.ego = ego;
        frame.ego_speed_mps = source.ego_speed_mps;
        frame.ego_acceleration_mps2 = context.ego_acceleration_mps2;
        frame.lane_driving = planned.pipeline.cruise_stopped;
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
        if (publishFinalFrame(final_pipeline, trajectory_, log_, frame,
                              std::string(odom.header.frame_id.c_str()))) {
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
  ara::log::Logger &log_;
  std::thread worker_;
};

bool runApplication(const sigset_t &shutdown_signals) {
  auto &log = ara::log::CreateLogger("PLAA", "Planning AA communication");
  SharedState state;
  RouteService routes(state);
  RouteStateService route_state(state);
  TrajectorySkeleton trajectory(ara::core::InstanceSpecifier{"PlanningAA/AA/PPort_PV2SCr"});
  auto route_offer = routes.OfferService();
  auto state_offer = route_state.OfferService();
  auto trajectory_offer = trajectory.OfferService();
  if (!route_offer.HasValue() || !state_offer.HasValue() || !trajectory_offer.HasValue()) {
    log.LogError() << "Could not offer planning services";
    if (trajectory_offer.HasValue()) trajectory.StopOfferService();
    if (state_offer.HasValue()) route_state.StopOfferService();
    if (route_offer.HasValue()) routes.StopOfferService();
    return false;
  }
  RouteStatePublisher state_publisher(state, route_state, log);
  InputService inputs(state, log);
  if (!inputs.start()) {
    state_publisher.stop();
    trajectory.StopOfferService();
    route_state.StopOfferService();
    routes.StopOfferService();
    return false;
  }
  FrameProducer producer(state, trajectory, log);
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
