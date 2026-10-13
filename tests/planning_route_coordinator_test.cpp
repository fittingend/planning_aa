// SPDX-License-Identifier: Apache-2.0

#include "planning/planning_route_coordinator.hpp"

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
    PlanningRouteCoordinator coordinator(
        [](RouteCheckpoint, const std::vector<RouteCheckpoint> &,
           RouteCheckpoint goal) -> std::optional<MissionRoute> {
          if (goal.x < 0.0) return std::nullopt;
          MissionRoute route;
          route.sections.push_back({1, {1}});
          return route;
        });
    coordinator.setMapReady(true, 0.0);
    coordinator.onEgo({{0.0, 0.0, 0.0, 0.0}, 2.0}, 0.1);
    coordinator.onOperationMode({false, false});
    check(coordinator.state(RouteChannel::Main).value ==
              RouteLifecycleState::Unset &&
              coordinator.state(RouteChannel::Mrm).value ==
              RouteLifecycleState::Unset,
          "mission readiness must initialize both selector channels");
    coordinator.takeEvents();

    RouteSelectorRequest main;
    main.kind = RouteCommandKind::SetWaypoint;
    main.goal.x = 10.0;
    main.stamp_s = 0.2;
    check(coordinator.handle(RouteChannel::Main, main, 0.2).success &&
              coordinator.route(RouteChannel::Main) &&
              coordinator.state(RouteChannel::Main).value ==
                  RouteLifecycleState::Set,
          "main route request must reach mission planner and publish route");
    const auto main_uuid = coordinator.route(RouteChannel::Main)->uuid;
    coordinator.takeEvents();

    RouteSelectorRequest mrm = main;
    mrm.goal.x = 20.0;
    check(coordinator.handle(RouteChannel::Mrm, mrm, 0.3).success &&
              coordinator.mrmOperating() &&
              coordinator.route(RouteChannel::Mrm) &&
              coordinator.route(RouteChannel::Mrm)->goal.x == 20.0 &&
              coordinator.state(RouteChannel::Main).value ==
                  RouteLifecycleState::Interrupted,
          "successful MRM route must be attributed to MRM and interrupt main");
    coordinator.takeEvents();

    RouteSelectorRequest queued_main = main;
    queued_main.goal.x = 30.0;
    check(coordinator.handle(RouteChannel::Main, queued_main, 0.4).success &&
              !coordinator.route(RouteChannel::Main) &&
              coordinator.route(RouteChannel::Mrm)->goal.x == 20.0,
          "main request during MRM must be queued without replacing mission route");
    check(coordinator.handle(RouteChannel::Mrm, {}, 0.5).success &&
              !coordinator.mrmOperating() &&
              coordinator.route(RouteChannel::Main) &&
              coordinator.route(RouteChannel::Main)->goal.x == 30.0 &&
              coordinator.state(RouteChannel::Mrm).value ==
                  RouteLifecycleState::Unset,
          "clearing MRM must restore queued main goal and publish it to main");
    check(coordinator.route(RouteChannel::Main)->uuid != main_uuid,
          "new main route must retain its own generated UUID");

    const auto active_uuid = coordinator.route(RouteChannel::Main)->uuid;
    check(coordinator.onModifiedGoal(active_uuid,
                                     {31.0, 0.0, 0.0, 0.0}, 0.6) &&
              coordinator.route(RouteChannel::Main)->goal.x == 31.0,
          "modified goal route must flow through the selected main channel");
    coordinator.takeEvents();
    check(coordinator.handle(RouteChannel::Main, {}, 0.7).success &&
              !coordinator.route(RouteChannel::Main) &&
              coordinator.state(RouteChannel::Main).value ==
                  RouteLifecycleState::Unset,
          "clear request must remove active main route");
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
