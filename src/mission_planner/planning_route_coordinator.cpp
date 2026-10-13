// SPDX-License-Identifier: Apache-2.0

#include "planning/planning_route_coordinator.hpp"

#include <utility>

namespace planning {

PlanningRouteCoordinator::PlanningRouteCoordinator(
    MissionRoutePlan planner, MissionRerouteSafety safety,
    MissionArrivalCheck arrival, MissionPlannerOptions options)
    : mission_(std::move(planner), std::move(safety),
               std::move(arrival), options) {}

void PlanningRouteCoordinator::forwardMissionEvents() {
  for (const auto &event : mission_.takeEvents()) {
    if (event.route) selector_.onPlannerRoute(*event.route);
    else selector_.onPlannerState(event.state);
  }
}

void PlanningRouteCoordinator::setMapReady(bool ready, double now_s) {
  mission_.setMapReady(ready, now_s);
  forwardMissionEvents();
}

void PlanningRouteCoordinator::onEgo(MissionPlannerEgo ego, double now_s) {
  mission_.onEgo(std::move(ego), now_s);
  forwardMissionEvents();
}

void PlanningRouteCoordinator::onOperationMode(MissionPlannerOperationMode mode) {
  mission_.onOperationMode(mode);
}

void PlanningRouteCoordinator::onRerouteAvailability(bool available) {
  mission_.onRerouteAvailability(available);
}

RouteSelectorStatus PlanningRouteCoordinator::handle(
    RouteChannel channel, RouteSelectorRequest request, double now_s) {
  const auto status = selector_.handle(
      channel, std::move(request), now_s,
      [&](const RouteSelectorRequest &forwarded) {
        return mission_.handle(forwarded, now_s);
      });
  // The original selector receives route/state publications after its
  // service callback updates mrm_operating_. Forwarding inside planner_call
  // would attribute a new MRM route to the main channel, or vice versa.
  forwardMissionEvents();
  return status;
}

bool PlanningRouteCoordinator::onModifiedGoal(
    RouteUuid uuid, RouteCheckpoint goal, double now_s) {
  const bool changed = mission_.onModifiedGoal(uuid, goal, now_s);
  forwardMissionEvents();
  return changed;
}

RouteSelectorState PlanningRouteCoordinator::state(RouteChannel channel) const {
  return selector_.state(channel);
}

std::optional<RouteSelectorRoute> PlanningRouteCoordinator::route(
    RouteChannel channel) const {
  return selector_.route(channel);
}

bool PlanningRouteCoordinator::mrmOperating() const {
  return selector_.mrmOperating();
}

std::vector<RouteSelectorEvent> PlanningRouteCoordinator::takeEvents() {
  return selector_.takeEvents();
}

}  // namespace planning
