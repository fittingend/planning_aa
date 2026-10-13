// SPDX-License-Identifier: Apache-2.0

#include "planning/mission_route_map_adapter.hpp"
#include "planning/remaining_distance_map_adapter.hpp"
#include "planning/planning_evaluator_map_adapter.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>

namespace {
void check(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}
}  // namespace

int main(int argc, char **argv) {
  using namespace planning;
  if (argc != 2) return EXIT_FAILURE;
  try {
    std::ifstream file(argv[1], std::ios::binary);
    check(file.good(), "map file unavailable");
    const std::vector<std::uint8_t> bytes{
        std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
    MissionPlannerOptions mission_options;
    mission_options.allow_reroute_in_autonomous_mode = true;
    MissionRouteMapAdapter session(bytes, {}, {}, mission_options);
    RouteCheckpoint start{2584.8986435446886, 23942.111196147092,
                          2.0 * std::atan2(0.5386095721815716,
                                           0.8425286082270071), 0.0};
    RouteCheckpoint goal{2569.02154365407, 24006.877820415717,
                         2.0 * std::atan2(-0.9999988599877392,
                                          0.00010565657690745384), 0.0};
    session.onEgo({start, 0.0}, 1.0);
    check(session.planner().ready(), "map and ego should ready planner");
    check(session.selector().state(RouteChannel::Main).value ==
              RouteLifecycleState::Unset,
          "selector should receive planner readiness");

    RouteSelectorRequest request;
    request.kind = RouteCommandKind::SetWaypoint;
    request.goal = goal;
    check(session.handle(RouteChannel::Main, request, 2.0).success,
          "main route planning failed");
    const auto main = session.selector().route(RouteChannel::Main);
    check(main && main->route.sections.size() == 3 &&
              main->route.sections[0].preferred_lanelet_id == 24747 &&
              main->route.sections[1].preferred_lanelet_id == 25001 &&
              main->route.sections[2].preferred_lanelet_id == 24857,
          "planned route does not match reference segments");
    check(session.selector().state(RouteChannel::Main).value ==
              RouteLifecycleState::Set,
          "main route state must follow the planned route");
    const auto echoed = session.takeGoalPoseEchoes();
    check(echoed.size() == 1 && echoed.front().pose.x == goal.x &&
              echoed.front().pose.y == goal.y &&
              echoed.front().frame_id == "map" &&
              session.takeGoalPoseEchoes().empty(),
          "goal pose visualizer should echo each published route");

    session.onOperationMode({true, true});
    session.onRerouteAvailability(true);
    session.onEgo({start, 2.0}, 2.2);
    RouteSelectorRequest reroute;
    reroute.kind = RouteCommandKind::SetLanelet;
    reroute.goal = goal;
    reroute.lanelet_route = main->route;
    check(session.handle(RouteChannel::Main, reroute, 2.5).success,
          "map adapter should evaluate safe autonomous reroute");

    RouteSelectorRequest emergency;
    emergency.kind = RouteCommandKind::SetLanelet;
    emergency.goal = goal;
    emergency.lanelet_route = main->route;
    check(session.handle(RouteChannel::Mrm, emergency, 3.0).success &&
              session.selector().mrmOperating(),
          "MRM route must take control");
    check(session.handle(RouteChannel::Mrm, {}, 4.0).success &&
              !session.selector().mrmOperating(),
          "main route must be restored after MRM clear");
    check(session.selector().route(RouteChannel::Main).has_value() &&
              session.selector().state(RouteChannel::Main).value ==
                  RouteLifecycleState::Set,
          "restored route must return to main channel");
    session.onOperationMode({false, false});

    PlanningEvaluatorMapAdapter evaluator(bytes);
    PlanningEvaluatorInput evaluation;
    TrajectoryPoint ego_point{};
    ego_point.x = start.x;
    ego_point.y = start.y;
    ego_point.orientation_w = std::cos(start.yaw_rad / 2.0);
    ego_point.orientation_z = std::sin(start.yaw_rad / 2.0);
    evaluation.ego = ego_point;
    const auto lane_metric = evaluator.tick(evaluation, main->route);
    check(lane_metric.scalar_metrics.count("ego_lane_info/lane_id") == 1 &&
              lane_metric.scalar_metrics.count("ego_lane_info/s") == 1 &&
              lane_metric.scalar_metrics.count("ego_lane_info/t") == 1,
          "route/map evaluator should add ego lane metrics");

    RemainingDistanceMapAdapter remaining(bytes);
    remaining.core().onEgo(start);
    remaining.core().onGoal(goal);
    remaining.core().onScenario(RemainingDistanceScenario::LaneDriving);
    remaining.core().onVelocityLimit(10.0);
    const auto distance_time = remaining.core().tick();
    check(distance_time && std::isfinite(distance_time->distance_m) &&
              distance_time->distance_m > 0.0 &&
              distance_time->time_s == distance_time->distance_m / 10.0,
          "map route should produce finite distance and time");
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
