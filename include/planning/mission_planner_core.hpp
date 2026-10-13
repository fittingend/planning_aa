// SPDX-License-Identifier: Apache-2.0
// ROS 서비스와 지도 계산을 분리한 mission planner의 경로 상태 정책.

#pragma once

#include "planning/route_selector_core.hpp"

#include <functional>
#include <deque>
#include <optional>
#include <vector>

namespace planning {

enum class MissionPlannerCode {
  Success = 0,
  InvalidState = 1,
  PlannerUnready = 2,
  PlannerFailed = 3,
  RerouteFailed = 4,
  NoEffect = 60001,
};

struct MissionPlannerOptions {
  bool allow_reroute_in_autonomous_mode = true;
  double reroute_time_threshold_s = 10.0;
  double minimum_reroute_length_m = 30.0;
  double arrival_distance_m = 1.0;
  double arrival_angle_rad = 0.7853981633974483;
  double arrival_stopped_duration_s = 1.0;
};

struct MissionPlannerEgo {
  RouteCheckpoint pose;
  double speed_mps = 0.0;
  double lateral_speed_mps = 0.0;
  double vertical_speed_mps = 0.0;
};

struct MissionPlannerOperationMode {
  bool autonomous = false;
  bool autoware_control_enabled = false;
};

// 계획 함수는 start, 경유점, goal을 지도 좌표계로 받아 경로를 반환한다.
// safety 함수가 없으면 움직이는 자율주행 중 재계획은 거부한다.
using MissionRoutePlan = std::function<std::optional<MissionRoute>(
    RouteCheckpoint, const std::vector<RouteCheckpoint> &,
    RouteCheckpoint)>;
using MissionRerouteSafety = std::function<bool(
    const RouteSelectorRoute &, const RouteSelectorRoute &,
    const MissionPlannerEgo &)>;
using MissionArrivalCheck = std::function<bool(
    const MissionPlannerEgo &, const RouteSelectorRoute &)>;

struct MissionPlannerEvent {
  RouteSelectorState state;
  std::optional<RouteSelectorRoute> route;
};

class MissionPlannerCore {
 public:
  explicit MissionPlannerCore(
      MissionRoutePlan planner = {}, MissionRerouteSafety safety = {},
      MissionArrivalCheck arrival = {}, MissionPlannerOptions options = {});

  void setMapReady(bool ready, double now_s);
  void onEgo(MissionPlannerEgo ego, double now_s);
  void onOperationMode(MissionPlannerOperationMode mode);
  void onRerouteAvailability(bool available);
  RouteSelectorStatus handle(const RouteSelectorRequest &request,
                             double now_s);
  bool onModifiedGoal(RouteUuid uuid, RouteCheckpoint goal, double now_s);

  RouteSelectorState state() const { return state_; }
  std::optional<RouteSelectorRoute> route() const { return route_; }
  bool ready() const { return ready_; }
  std::vector<MissionPlannerEvent> takeEvents();

 private:
  MissionRoutePlan planner_;
  MissionRerouteSafety safety_;
  MissionArrivalCheck arrival_;
  MissionPlannerOptions options_;
  RouteSelectorState state_{RouteLifecycleState::Initializing, 0.0};
  std::optional<RouteSelectorRoute> route_;
  std::optional<MissionPlannerEgo> ego_;
  std::optional<MissionPlannerOperationMode> mode_;
  std::optional<bool> reroute_available_;
  struct StopSample {
    double stamp_s = 0.0;
    double speed_squared = 0.0;
  };
  std::deque<StopSample> stop_samples_;
  bool map_ready_ = false;
  bool ready_ = false;
  std::vector<MissionPlannerEvent> events_;

  void checkReady(double now_s);
  void changeState(RouteLifecycleState state, double now_s);
  RouteSelectorStatus setRoute(const RouteSelectorRequest &request,
                               double now_s);
};

}  // namespace planning
