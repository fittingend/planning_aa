// SPDX-License-Identifier: Apache-2.0
// ROS 서비스 없이 route selector와 mission planner의 요청·상태 흐름을 연결합니다.

#pragma once

#include "planning/mission_planner_core.hpp"

namespace planning {

class PlanningRouteCoordinator {
 public:
  explicit PlanningRouteCoordinator(
      MissionRoutePlan planner = {}, MissionRerouteSafety safety = {},
      MissionArrivalCheck arrival = {}, MissionPlannerOptions options = {});

  void setMapReady(bool ready, double now_s);
  void onEgo(MissionPlannerEgo ego, double now_s);
  void onOperationMode(MissionPlannerOperationMode mode);
  void onRerouteAvailability(bool available);
  RouteSelectorStatus handle(RouteChannel channel,
                             RouteSelectorRequest request, double now_s);
  bool onModifiedGoal(RouteUuid uuid, RouteCheckpoint goal, double now_s);

  RouteSelectorState state(RouteChannel channel) const;
  std::optional<RouteSelectorRoute> route(RouteChannel channel) const;
  bool mrmOperating() const;
  std::vector<RouteSelectorEvent> takeEvents();

 private:
  MissionPlannerCore mission_;
  RouteSelectorCore selector_;

  void forwardMissionEvents();
};

}  // namespace planning
