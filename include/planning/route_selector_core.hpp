// SPDX-License-Identifier: Apache-2.0
// Mission planner의 main/MRM 경로 선택 정책. 전송과 지도 계산은 호출자가 제공한다.

#pragma once

#include "planning/lanelet_bin_adapter.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace planning {

using RouteUuid = std::array<std::uint8_t, 16>;

enum class RouteChannel { Main, Mrm };
enum class RouteCommandKind { Clear, SetWaypoint, SetLanelet };
enum class RouteLifecycleState {
  Unknown, Initializing, Unset, Routing, Set, Arrived, Rerouting,
  Interrupted,
};

struct RouteSelectorRequest {
  RouteCommandKind kind = RouteCommandKind::Clear;
  RouteUuid uuid{};
  RouteCheckpoint goal;
  std::vector<RouteCheckpoint> waypoints;
  MissionRoute lanelet_route;
  bool allow_modification = false;
  double stamp_s = 0.0;
};

struct RouteSelectorRoute {
  RouteUuid uuid{};
  RouteCheckpoint start;
  RouteCheckpoint goal;
  MissionRoute route;
  bool allow_modification = false;
  double stamp_s = 0.0;
};

struct RouteSelectorStatus {
  bool success = false;
  int code = 0;
  std::string message;
};

struct RouteSelectorState {
  RouteLifecycleState value = RouteLifecycleState::Unknown;
  double stamp_s = 0.0;
};

struct RouteSelectorEvent {
  RouteChannel channel = RouteChannel::Main;
  RouteSelectorState state;
  std::optional<RouteSelectorRoute> route;
};

// 동기 서비스 호출의 성공/실패를 반환한다. planner의 route/state 발행은
// onPlannerRoute/onPlannerState로 별도로 전달해야 원본의 순서를 재현할 수 있다.
using RouteSelectorPlannerCall =
    std::function<RouteSelectorStatus(const RouteSelectorRequest &)>;

class RouteSelectorCore {
 public:
  RouteSelectorCore();

  RouteSelectorStatus handle(
      RouteChannel channel, RouteSelectorRequest request,
      double now_s, const RouteSelectorPlannerCall &planner_call);
  void onPlannerState(RouteSelectorState state);
  void onPlannerRoute(const RouteSelectorRoute &route);

  RouteSelectorState state(RouteChannel channel) const;
  std::optional<RouteSelectorRoute> route(RouteChannel channel) const;
  bool mrmOperating() const { return mrm_operating_; }
  std::vector<RouteSelectorEvent> takeEvents();

 private:
  struct ChannelData {
    RouteSelectorState state;
    std::optional<RouteSelectorRoute> route;
  };
  ChannelData main_;
  ChannelData mrm_;
  std::optional<RouteSelectorRequest> main_request_;
  bool initialized_ = false;
  bool mrm_operating_ = false;
  std::vector<RouteSelectorEvent> events_;

  ChannelData &data(RouteChannel channel);
  const ChannelData &data(RouteChannel channel) const;
  void changeState(RouteChannel channel, RouteLifecycleState state,
                   double now_s);
  RouteSelectorStatus resumeMain(
      const RouteSelectorRequest &clear_request,
      const RouteSelectorPlannerCall &planner_call);
};

}  // namespace planning
