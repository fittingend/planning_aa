// SPDX-License-Identifier: Apache-2.0

#include "planning/mission_route_map_adapter.hpp"
#include "planning/mission_reroute_safety.hpp"

#include <utility>

namespace planning {

MissionRouteMapAdapter::MissionRouteMapAdapter(
    const std::vector<std::uint8_t> &map_data,
    MissionRerouteSafety safety, MissionArrivalCheck arrival,
    MissionPlannerOptions options)
    : map_(map_data),
      planner_(
          [this](RouteCheckpoint start,
                 const std::vector<RouteCheckpoint> &waypoints,
                 RouteCheckpoint goal) -> std::optional<MissionRoute> {
            std::vector<RouteCheckpoint> checkpoints;
            checkpoints.reserve(waypoints.size() + 2);
            checkpoints.push_back(start);
            checkpoints.insert(checkpoints.end(), waypoints.begin(),
                               waypoints.end());
            checkpoints.push_back(goal);
            return map_.routeThroughCheckpoints(checkpoints);
          },
          safety ? std::move(safety) :
              MissionRerouteSafety{[this, options](
                  const RouteSelectorRoute &original,
                  const RouteSelectorRoute &target,
                  const MissionPlannerEgo &ego) {
                const MissionRerouteGeometry geometry{
                    [this](std::int64_t id) {
                      return map_.laneCenterline(id);
                    },
                    [this](std::int64_t id, RouteCheckpoint pose) {
                      return map_.pointInRoadLanelet(id, pose.x, pose.y);
                    }};
                return checkMissionRerouteSafety(
                    original, target, ego, geometry,
                    options.reroute_time_threshold_s,
                    options.minimum_reroute_length_m);
              }},
          std::move(arrival), options) {
  planner_.setMapReady(true, 0.0);
}

void MissionRouteMapAdapter::forwardPlannerEvents() {
  for (const auto &event : planner_.takeEvents()) {
    if (event.route) {
      selector_.onPlannerRoute(*event.route);
      goal_pose_echoes_.push_back(
          {event.route->goal, event.route->stamp_s, "map"});
    } else
      selector_.onPlannerState(event.state);
  }
}

void MissionRouteMapAdapter::onEgo(MissionPlannerEgo ego, double now_s) {
  planner_.onEgo(ego, now_s);
  forwardPlannerEvents();
}

void MissionRouteMapAdapter::onOperationMode(
    MissionPlannerOperationMode mode) {
  planner_.onOperationMode(mode);
}

void MissionRouteMapAdapter::onRerouteAvailability(bool available) {
  planner_.onRerouteAvailability(available);
}

RouteSelectorStatus MissionRouteMapAdapter::handle(
    RouteChannel channel, RouteSelectorRequest request, double now_s) {
  const auto result = selector_.handle(
      channel, std::move(request), now_s,
      [this, now_s](const RouteSelectorRequest &forwarded) {
        return planner_.handle(forwarded, now_s);
      });
  forwardPlannerEvents();
  return result;
}

bool MissionRouteMapAdapter::onModifiedGoal(
    RouteUuid uuid, RouteCheckpoint goal, double now_s) {
  const bool changed = planner_.onModifiedGoal(uuid, goal, now_s);
  forwardPlannerEvents();
  return changed;
}

std::vector<RouteSelectorEvent>
MissionRouteMapAdapter::takeSelectorEvents() {
  return selector_.takeEvents();
}

std::vector<GoalPoseEcho>
MissionRouteMapAdapter::takeGoalPoseEchoes() {
  auto result = std::move(goal_pose_echoes_);
  goal_pose_echoes_.clear();
  return result;
}

}  // namespace planning
