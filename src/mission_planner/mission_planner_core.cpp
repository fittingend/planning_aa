// SPDX-License-Identifier: Apache-2.0

#include "planning/mission_planner_core.hpp"

#include <cmath>
#include <utility>

namespace planning {
namespace {
RouteSelectorStatus status(MissionPlannerCode code,
                           const char *message = "") {
  return {code == MissionPlannerCode::Success,
          static_cast<int>(code), message};
}
}  // namespace

MissionPlannerCore::MissionPlannerCore(
    MissionRoutePlan planner, MissionRerouteSafety safety,
    MissionArrivalCheck arrival, MissionPlannerOptions options)
    : planner_(std::move(planner)), safety_(std::move(safety)),
      arrival_(std::move(arrival)), options_(options) {}

void MissionPlannerCore::checkReady(double now_s) {
  if (ready_ || !map_ready_ || !ego_) return;
  ready_ = true;
  changeState(RouteLifecycleState::Unset, now_s);
}

void MissionPlannerCore::setMapReady(bool ready, double now_s) {
  map_ready_ = ready;
  checkReady(now_s);
}

void MissionPlannerCore::onEgo(MissionPlannerEgo ego, double now_s) {
  ego_ = ego;
  checkReady(now_s);
  // VehicleStopCheckerBase retains odometry samples for ten seconds and
  // checks every sample in the requested stopped interval, including the
  // first sample at or before the interval boundary.
  if (std::isfinite(now_s) && std::isfinite(ego.speed_mps) &&
      std::isfinite(ego.lateral_speed_mps) &&
      std::isfinite(ego.vertical_speed_mps)) {
    stop_samples_.push_front({
        now_s, ego.speed_mps * ego.speed_mps +
                   ego.lateral_speed_mps * ego.lateral_speed_mps +
                   ego.vertical_speed_mps * ego.vertical_speed_mps});
    while (!stop_samples_.empty() &&
           now_s - stop_samples_.back().stamp_s > 10.0)
      stop_samples_.pop_back();
  } else {
    stop_samples_.clear();
  }
  bool stopped = !stop_samples_.empty() &&
      now_s - stop_samples_.back().stamp_s >=
          options_.arrival_stopped_duration_s;
  if (stopped) {
    for (const auto &sample : stop_samples_) {
      if (sample.speed_squared >= 1.0e-6) {
        stopped = false;
        break;
      }
      if (now_s - sample.stamp_s >=
          options_.arrival_stopped_duration_s)
        break;
    }
  }
  const bool default_arrived = route_ && stopped &&
      std::hypot(ego.pose.x - route_->goal.x,
                 ego.pose.y - route_->goal.y) <=
          options_.arrival_distance_m &&
      std::abs(std::remainder(ego.pose.yaw_rad -
                             route_->goal.yaw_rad,
                             2.0 * std::acos(-1.0))) <=
          options_.arrival_angle_rad;
  if (state_.value == RouteLifecycleState::Set && route_ &&
      (arrival_ ? arrival_(*ego_, *route_) : default_arrived))
    changeState(RouteLifecycleState::Arrived, now_s);
}

void MissionPlannerCore::onOperationMode(MissionPlannerOperationMode mode) {
  mode_ = mode;
}

void MissionPlannerCore::onRerouteAvailability(bool available) {
  reroute_available_ = available;
}

void MissionPlannerCore::changeState(RouteLifecycleState state,
                                     double now_s) {
  state_ = {state, now_s};
  events_.push_back({state_, std::nullopt});
}

RouteSelectorStatus MissionPlannerCore::handle(
    const RouteSelectorRequest &request, double now_s) {
  if (request.kind == RouteCommandKind::Clear) {
    if (!ready_)
      return status(MissionPlannerCode::NoEffect,
                    "The mission planner is not ready.");
    route_.reset();
    changeState(RouteLifecycleState::Unset, now_s);
    return status(MissionPlannerCode::Success);
  }
  return setRoute(request, now_s);
}

RouteSelectorStatus MissionPlannerCore::setRoute(
    const RouteSelectorRequest &request, double now_s) {
  const bool reroute = state_.value == RouteLifecycleState::Set;
  if (state_.value != RouteLifecycleState::Unset && !reroute)
    return status(MissionPlannerCode::InvalidState,
                  "The route cannot be set in the current state.");
  if (!ready_)
    return status(MissionPlannerCode::PlannerUnready,
                  "The mission planner is not ready.");
  if (reroute && !mode_)
    return status(MissionPlannerCode::PlannerUnready,
                  "Operation mode state is not received.");

  const bool autonomous = reroute && mode_->autonomous &&
                          mode_->autoware_control_enabled;
  // 원본 구현은 lanelet route에서만 이 설정을 검사한다.
  if (autonomous && request.kind == RouteCommandKind::SetLanelet &&
      !options_.allow_reroute_in_autonomous_mode)
    return status(MissionPlannerCode::InvalidState,
                  "Reroute is not allowed in autonomous mode.");
  if (autonomous && (!reroute_available_ || !*reroute_available_))
    return status(MissionPlannerCode::InvalidState,
                  "Cannot reroute as the planner is not in lane following.");

  changeState(reroute ? RouteLifecycleState::Rerouting :
                        RouteLifecycleState::Routing,
              now_s);
  std::optional<MissionRoute> planned;
  if (request.kind == RouteCommandKind::SetLanelet)
    planned = request.lanelet_route;
  else if (planner_ && ego_)
    planned = planner_(ego_->pose, request.waypoints, request.goal);

  if (!planned || planned->sections.empty()) {
    changeState(reroute ? RouteLifecycleState::Set :
                          RouteLifecycleState::Unset,
                now_s);
    return status(MissionPlannerCode::PlannerFailed,
                  "The planned route is empty.");
  }

  RouteSelectorRoute candidate;
  candidate.uuid = request.uuid;
  candidate.start = ego_->pose;
  candidate.goal = request.goal;
  candidate.route = std::move(*planned);
  candidate.allow_modification = request.allow_modification;
  candidate.stamp_s = request.stamp_s;
  if (autonomous && ego_ && ego_->speed_mps >= 0.01 &&
      (!safety_ || !safety_(*route_, candidate, *ego_))) {
    changeState(RouteLifecycleState::Set, now_s);
    return status(MissionPlannerCode::RerouteFailed,
                  "New route is not safe. Reroute failed.");
  }

  route_ = std::move(candidate);
  events_.push_back({state_, route_});
  changeState(RouteLifecycleState::Set, now_s);
  return status(MissionPlannerCode::Success);
}

bool MissionPlannerCore::onModifiedGoal(
    RouteUuid uuid, RouteCheckpoint goal, double now_s) {
  if (state_.value != RouteLifecycleState::Set || !ready_ ||
      !route_ || route_->uuid != uuid || !ego_ || !planner_)
    return false;
  changeState(RouteLifecycleState::Rerouting, now_s);
  const auto planned = planner_(route_->start, {ego_->pose}, goal);
  if (!planned || planned->sections.empty()) {
    changeState(RouteLifecycleState::Set, now_s);
    return false;
  }
  route_->route = *planned;
  route_->goal = goal;
  events_.push_back({state_, route_});
  changeState(RouteLifecycleState::Set, now_s);
  return true;
}

std::vector<MissionPlannerEvent> MissionPlannerCore::takeEvents() {
  auto result = std::move(events_);
  events_.clear();
  return result;
}

}  // namespace planning
