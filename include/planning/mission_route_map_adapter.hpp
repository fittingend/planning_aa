// SPDX-License-Identifier: Apache-2.0
// 이진 지도 경로 계산과 mission planner / route selector 상태를 연결합니다.

#pragma once

#include "planning/lanelet_bin_adapter.hpp"
#include "planning/mission_planner_core.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace planning {

struct GoalPoseEcho {
  RouteCheckpoint pose;
  double stamp_s = 0.0;
  std::string frame_id = "map";
};

class MissionRouteMapAdapter {
 public:
  explicit MissionRouteMapAdapter(
      const std::vector<std::uint8_t> &map_data,
      MissionRerouteSafety safety = {}, MissionArrivalCheck arrival = {},
      MissionPlannerOptions options = {});

  void onEgo(MissionPlannerEgo ego, double now_s);
  void onOperationMode(MissionPlannerOperationMode mode);
  void onRerouteAvailability(bool available);
  RouteSelectorStatus handle(RouteChannel channel,
                             RouteSelectorRequest request,
                             double now_s);
  bool onModifiedGoal(RouteUuid uuid, RouteCheckpoint goal, double now_s);

  const MissionPlannerCore &planner() const { return planner_; }
  const RouteSelectorCore &selector() const { return selector_; }
  std::vector<RouteSelectorEvent> takeSelectorEvents();
  std::vector<GoalPoseEcho> takeGoalPoseEchoes();

 private:
  LaneletBinAdapter map_;
  MissionPlannerCore planner_;
  RouteSelectorCore selector_;
  std::vector<GoalPoseEcho> goal_pose_echoes_;

  void forwardPlannerEvents();
};

}  // namespace planning
