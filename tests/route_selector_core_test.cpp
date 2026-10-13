// SPDX-License-Identifier: Apache-2.0

#include "planning/route_selector_core.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
void check(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

bool nonzero(const planning::RouteUuid &uuid) {
  return std::any_of(uuid.begin(), uuid.end(),
                     [](std::uint8_t value) { return value != 0; });
}
}  // namespace

int main() {
  using namespace planning;
  try {
    RouteSelectorCore selector;
    check(selector.state(RouteChannel::Main).value ==
              RouteLifecycleState::Initializing,
          "main must start initializing");
    selector.takeEvents();
    selector.onPlannerState({RouteLifecycleState::Unset, 1.0});
    check(selector.state(RouteChannel::Main).value ==
              RouteLifecycleState::Unset &&
              selector.state(RouteChannel::Mrm).value ==
              RouteLifecycleState::Unset,
          "planner readiness must initialize both channels");
    selector.takeEvents();

    std::vector<RouteSelectorRequest> calls;
    const auto success = [&](const RouteSelectorRequest &request) {
      calls.push_back(request);
      return RouteSelectorStatus{true, 0, {}};
    };
    RouteSelectorRequest main;
    main.kind = RouteCommandKind::SetWaypoint;
    main.stamp_s = 2.0;
    main.goal = {10.0, 0.0, 0.0, 0.0};
    main.waypoints = {{5.0, 0.0, 0.0, 0.0}};
    check(selector.handle(RouteChannel::Main, main, 2.0, success).success,
          "main request failed");
    check(calls.size() == 1 && nonzero(calls.back().uuid),
          "main request should be forwarded with generated UUID");
    RouteSelectorRoute main_route;
    main_route.uuid = calls.back().uuid;
    main_route.goal = main.goal;
    main_route.stamp_s = 2.1;
    main_route.route.sections.push_back({1, {1}});
    selector.onPlannerRoute(main_route);
    selector.onPlannerState({RouteLifecycleState::Set, 2.1});
    check(selector.route(RouteChannel::Main).has_value(),
          "main route must be retained");

    RouteSelectorRequest invalid_mrm;
    invalid_mrm.kind = RouteCommandKind::SetWaypoint;
    invalid_mrm.goal = {10000.0, 0.0, 0.0, 0.0};
    const auto reject_mrm = [&](const RouteSelectorRequest &request) {
      calls.push_back(request);
      return RouteSelectorStatus{false, 3, "planned route is empty"};
    };
    check(selector.handle(RouteChannel::Mrm, invalid_mrm,
                          2.5, reject_mrm).code == 3 &&
              !selector.mrmOperating() &&
              selector.state(RouteChannel::Main).value ==
                  RouteLifecycleState::Set &&
              selector.route(RouteChannel::Main).has_value(),
          "failed MRM request must leave main route active");

    RouteSelectorRequest mrm;
    mrm.kind = RouteCommandKind::SetLanelet;
    mrm.goal = {20.0, 0.0, 0.0, 0.0};
    check(selector.handle(RouteChannel::Mrm, mrm, 3.0, success).success,
          "MRM request failed");
    check(selector.mrmOperating() &&
              selector.state(RouteChannel::Main).value ==
                  RouteLifecycleState::Interrupted,
          "MRM must interrupt main route");
    selector.onPlannerState({RouteLifecycleState::Set, 3.1});
    check(selector.state(RouteChannel::Mrm).value ==
              RouteLifecycleState::Set,
          "planner state must be attributed to MRM");

    RouteSelectorRequest clear_mrm;
    clear_mrm.stamp_s = 4.0;
    check(selector.handle(RouteChannel::Mrm,
                          clear_mrm, 4.0, success).success,
          "MRM clear failed");
    check(!selector.mrmOperating() && calls.back().kind ==
              RouteCommandKind::SetLanelet &&
              calls.back().uuid == main_route.uuid,
          "MRM clear must first restore saved main route");
    check(calls.back().stamp_s == main_route.stamp_s,
          "restored route must retain its published header stamp");

    // A main request made during MRM is stored but not forwarded. When MRM
    // ends, the source replans from the goal and drops old waypoints.
    selector.handle(RouteChannel::Mrm, mrm, 5.0, success);
    const auto before = calls.size();
    main.goal = {30.0, 0.0, 0.0, 0.0};
    main.stamp_s = 5.1;
    check(selector.handle(RouteChannel::Main, main, 5.1, success).success &&
              calls.size() == before,
          "main request must wait during MRM");
    check(selector.state(RouteChannel::Main).value ==
              RouteLifecycleState::Interrupted &&
              !selector.route(RouteChannel::Main),
          "new main request must clear stale route");
    selector.handle(RouteChannel::Mrm, RouteSelectorRequest{}, 6.0, success);
    check(calls.back().kind == RouteCommandKind::SetWaypoint &&
              calls.back().waypoints.empty() &&
              calls.back().goal.x == 30.0 &&
              calls.back().stamp_s == 5.1,
          "MRM clear must replan current main goal without waypoints");

    selector.handle(RouteChannel::Main, RouteSelectorRequest{}, 6.1,
                    success);
    selector.handle(RouteChannel::Mrm, mrm, 6.2, success);
    clear_mrm.stamp_s = 6.3;
    selector.handle(RouteChannel::Mrm, clear_mrm, 6.3, success);
    check(calls.back().kind == RouteCommandKind::Clear &&
              calls.back().stamp_s == 6.3,
          "MRM clear must forward its request when no main route is saved");

    selector.handle(RouteChannel::Mrm, mrm, 7.0, success);
    const auto failure = [&](const RouteSelectorRequest &request) {
      calls.push_back(request);
      return RouteSelectorStatus{false, 1, "planner unavailable"};
    };
    check(!selector.handle(RouteChannel::Mrm, RouteSelectorRequest{},
                           8.0, failure).success &&
              selector.mrmOperating(),
          "failed main restoration must keep MRM active");

    // Original live probe: main success followed by an invalid main goal
    // returns code 3, clears the selector's stored route, and leaves SET.
    RouteSelectorCore failed_main;
    failed_main.onPlannerState({RouteLifecycleState::Unset, 1.0});
    failed_main.takeEvents();
    RouteSelectorRequest initial_main;
    initial_main.kind = RouteCommandKind::SetWaypoint;
    initial_main.goal = {10.0, 0.0, 0.0, 0.0};
    check(failed_main.handle(RouteChannel::Main, initial_main,
                             2.0, success).success,
          "main route setup for failure case failed");
    failed_main.onPlannerRoute(main_route);
    failed_main.onPlannerState({RouteLifecycleState::Set, 2.1});
    failed_main.takeEvents();
    RouteSelectorRequest invalid_main = initial_main;
    invalid_main.goal.x += 10000.0;
    check(failed_main.handle(RouteChannel::Main, invalid_main,
                             2.5, reject_mrm).code == 3 &&
              !failed_main.route(RouteChannel::Main) &&
              failed_main.state(RouteChannel::Main).value ==
                  RouteLifecycleState::Set &&
              failed_main.takeEvents().empty(),
          "failed main request must clear stored route without selector state event");
    failed_main.onPlannerState({RouteLifecycleState::Rerouting, 2.6});
    failed_main.onPlannerState({RouteLifecycleState::Set, 2.7});
    const auto recovery_events = failed_main.takeEvents();
    check(recovery_events.size() == 2 &&
              recovery_events[0].state.value ==
                  RouteLifecycleState::Rerouting &&
              recovery_events[1].state.value ==
                  RouteLifecycleState::Set,
          "failed main request must forward planner rerouting recovery states");

    // The live selector saves a failed main request before the planner client
    // reports SERVICE_UNREADY. MRM clear retries that goal as a waypoint route.
    RouteSelectorCore unready;
    std::vector<RouteCommandKind> unavailable_calls;
    const auto unavailable = [&](const RouteSelectorRequest &request) {
      unavailable_calls.push_back(request.kind);
      return RouteSelectorStatus{false, 50001,
                                 "/probe/missing_planner/set_waypoint_route"};
    };
    RouteSelectorRequest unready_main;
    unready_main.kind = RouteCommandKind::SetWaypoint;
    unready_main.goal = {10.0, 0.0, 0.0, 0.0};
    check(unready.handle(RouteChannel::Main, unready_main,
                         1.0, unavailable).code == 50001 &&
              unready.handle(RouteChannel::Mrm, unready_main,
                             1.1, unavailable).code == 50001 &&
              !unready.mrmOperating(),
          "unready planner must reject both route requests");
    check(unready.handle(RouteChannel::Mrm, RouteSelectorRequest{},
                         1.2, unavailable).code == 50001 &&
              unavailable_calls.size() == 3 &&
              unavailable_calls.back() == RouteCommandKind::SetWaypoint,
          "MRM clear must retry the saved main goal even after service failure");
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
