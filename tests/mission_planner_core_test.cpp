// SPDX-License-Identifier: Apache-2.0

#include "planning/mission_planner_core.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}
}  // namespace

int main() {
  using namespace planning;
  try {
    check(static_cast<int>(MissionPlannerCode::InvalidState) == 1 &&
              static_cast<int>(MissionPlannerCode::PlannerUnready) == 2 &&
              static_cast<int>(MissionPlannerCode::PlannerFailed) == 3 &&
              static_cast<int>(MissionPlannerCode::RerouteFailed) == 4 &&
              static_cast<int>(MissionPlannerCode::NoEffect) == 60001,
          "mission response codes must match the source ADAPI messages");
    int plan_count = 0;
    bool permit_safety = false;
    MissionPlannerCore planner(
        [&](RouteCheckpoint, const std::vector<RouteCheckpoint> &,
            RouteCheckpoint goal) -> std::optional<MissionRoute> {
          ++plan_count;
          if (goal.x < 0.0) return std::nullopt;
          MissionRoute route;
          route.sections.push_back({1, {1}});
          route.goal_height_m = 5.0;
          return route;
        },
        [&](const RouteSelectorRoute &, const RouteSelectorRoute &,
            const MissionPlannerEgo &) { return permit_safety; },
        [&](const MissionPlannerEgo &ego, const RouteSelectorRoute &route) {
          return ego.pose.x >= route.goal.x;
        },
        {true});

    RouteSelectorRequest request;
    request.kind = RouteCommandKind::SetWaypoint;
    request.goal = {10.0, 0.0, 0.0, 0.0};
    request.uuid.front() = 7;
    check(planner.handle(request, 1.0).code == 1,
          "route must be rejected before map and odometry");
    check(planner.handle({}, 1.1).code == 60001,
          "clear before readiness must return ADAPI NO_EFFECT");
    planner.setMapReady(true, 2.0);
    check(!planner.ready(), "odometry is required for readiness");
    planner.onEgo({{0.0, 0.0, 0.0, 0.0}, 3.0}, 3.0);
    check(planner.ready() && planner.state().value ==
              RouteLifecycleState::Unset,
          "map and odometry must initialize route API");
    check(planner.handle(request, 4.0).success &&
              planner.state().value == RouteLifecycleState::Set &&
              planner.route()->uuid.front() == 7 &&
              planner.route()->goal.z == 5.0,
          "first waypoint route must use the map-refined goal height");

    request.goal.x = 20.0;
    check(planner.handle(request, 5.0).code ==
              static_cast<int>(MissionPlannerCode::PlannerUnready),
          "reroute requires operation mode input");
    planner.onOperationMode({true, true});
    check(planner.handle(request, 6.0).code ==
              static_cast<int>(MissionPlannerCode::InvalidState),
          "autonomous reroute requires lane-follow availability");
    planner.onRerouteAvailability(true);
    check(planner.handle(request, 7.0).code ==
              static_cast<int>(MissionPlannerCode::RerouteFailed) &&
              planner.route()->goal.x == 10.0,
          "unsafe reroute must preserve previous route");
    permit_safety = true;
    check(planner.handle(request, 8.0).success &&
              planner.route()->goal.x == 20.0,
          "safe autonomous reroute must replace route");
    planner.takeEvents();
    request.goal.x = -1.0;
    check(planner.handle(request, 9.0).code ==
              static_cast<int>(MissionPlannerCode::PlannerFailed) &&
              planner.route()->goal.x == 20.0,
          "failed planner output must preserve previous route");
    const auto rejected_events = planner.takeEvents();
    check(rejected_events.size() == 2 &&
              rejected_events[0].state.value ==
                  RouteLifecycleState::Rerouting &&
              rejected_events[1].state.value ==
                  RouteLifecycleState::Set &&
              !rejected_events[0].route &&
              !rejected_events[1].route,
          "failed reroute must publish REROUTING then SET without a new route");

    RouteUuid wrong_uuid{};
    check(!planner.onModifiedGoal(wrong_uuid,
                                  {25.0, 0.0, 0.0, 0.0}, 10.0),
          "modified goal must match route UUID");
    check(planner.onModifiedGoal(request.uuid,
                                 {25.0, 0.0, 0.0, 0.0}, 11.0) &&
              planner.route()->goal.x == 25.0,
          "modified goal should replan current route");
    planner.onEgo({{25.0, 0.0, 0.0, 0.0}, 0.0}, 12.0);
    check(planner.state().value == RouteLifecycleState::Arrived,
          "arrival should be emitted on odometry input");
    check(!planner.handle(request, 13.0).success,
          "new route is rejected from arrived state until cleared");
    check(planner.handle({}, 14.0).success && !planner.route() &&
              planner.state().value == RouteLifecycleState::Unset,
          "clear must reset route and state");
    check(plan_count >= 4, "planning callback must be used");

    // The original node published UNSET, ROUTING, route, SET, UNSET for
    // the HPC S-curve set/clear service sequence.
    MissionPlannerCore lifecycle(
        [](RouteCheckpoint, const std::vector<RouteCheckpoint> &,
           RouteCheckpoint) -> std::optional<MissionRoute> {
          MissionRoute route;
          route.sections.push_back({29, {29}});
          return route;
        });
    lifecycle.setMapReady(true, 0.0);
    lifecycle.onEgo({{0.0, 0.0, 0.0, 0.0}, 0.0}, 0.1);
    RouteSelectorRequest lifecycle_request;
    lifecycle_request.kind = RouteCommandKind::SetWaypoint;
    lifecycle_request.goal = {10.0, 0.0, 0.0, 0.0};
    check(lifecycle.handle(lifecycle_request, 0.2).success,
          "reference lifecycle set must succeed");
    check(lifecycle.handle({}, 0.3).success,
          "reference lifecycle clear must succeed");
    const auto lifecycle_events = lifecycle.takeEvents();
    check(lifecycle_events.size() == 5 &&
              lifecycle_events[0].state.value == RouteLifecycleState::Unset &&
              lifecycle_events[1].state.value == RouteLifecycleState::Routing &&
              lifecycle_events[2].route.has_value() &&
              lifecycle_events[3].state.value == RouteLifecycleState::Set &&
              lifecycle_events[4].state.value == RouteLifecycleState::Unset,
          "set/clear events must match original route and state order");

    MissionPlannerCore default_arrival(
        [](RouteCheckpoint, const std::vector<RouteCheckpoint> &,
           RouteCheckpoint) -> std::optional<MissionRoute> {
          MissionRoute route;
          route.sections.push_back({1, {1}});
          return route;
        });
    default_arrival.setMapReady(true, 0.0);
    default_arrival.onEgo({{0.0, 0.0, 0.0, 0.0}, 0.0}, 0.0);
    RouteSelectorRequest arrival_request;
    arrival_request.kind = RouteCommandKind::SetWaypoint;
    arrival_request.goal = {1.0, 0.0, 0.0, 0.0};
    check(default_arrival.handle(arrival_request, 0.1).success,
          "default arrival route must be set");
    default_arrival.onEgo({{1.0, 0.0, 1.57079632679, 0.0}, 0.0}, 1.1);
    check(default_arrival.state().value == RouteLifecycleState::Set,
          "stopped ego with wrong heading must not arrive");
    default_arrival.onEgo({{1.0, 0.0, 0.0, 0.0}, 0.0}, 1.2);
    check(default_arrival.state().value == RouteLifecycleState::Arrived,
          "default arrival requires goal pose and one second stopped");

    MissionPlannerCore interrupted_stop(
        [](RouteCheckpoint, const std::vector<RouteCheckpoint> &,
           RouteCheckpoint) -> std::optional<MissionRoute> {
          MissionRoute route;
          route.sections.push_back({1, {1}});
          return route;
        });
    interrupted_stop.setMapReady(true, 0.0);
    interrupted_stop.onEgo({{0.0, 0.0, 0.0, 0.0}, 0.0}, 0.0);
    check(interrupted_stop.handle(arrival_request, 0.1).success,
          "interrupted stop route must be set");
    interrupted_stop.onEgo({{1.0, 0.0, 0.0, 0.0}, 0.0, 0.1}, 0.6);
    interrupted_stop.onEgo({{1.0, 0.0, 0.0, 0.0}, 0.0}, 1.1);
    interrupted_stop.onEgo({{1.0, 0.0, 0.0, 0.0}, 0.0}, 1.6);
    check(interrupted_stop.state().value == RouteLifecycleState::Set,
          "moving odometry inside one-second buffer must block arrival");
    interrupted_stop.onEgo({{1.0, 0.0, 0.0, 0.0}, 0.0}, 2.2);
    check(interrupted_stop.state().value == RouteLifecycleState::Arrived,
          "arrival must use the newest one-second odometry window");
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
