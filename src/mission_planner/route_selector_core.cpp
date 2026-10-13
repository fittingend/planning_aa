// SPDX-License-Identifier: Apache-2.0

#include "planning/route_selector_core.hpp"

#include <algorithm>
#include <random>
#include <stdexcept>
#include <utility>

namespace planning {
namespace {
bool emptyUuid(const RouteUuid &uuid) {
  return std::all_of(uuid.begin(), uuid.end(),
                     [](std::uint8_t value) { return value == 0; });
}

void generateUuidIfEmpty(RouteUuid &uuid) {
  if (!emptyUuid(uuid)) return;
  static std::independent_bits_engine<std::mt19937, 8,
                                      std::uint8_t> source(
      std::random_device{}());
  for (auto &value : uuid)
    value = source();
}
}  // namespace

RouteSelectorCore::RouteSelectorCore() {
  main_.state.value = RouteLifecycleState::Initializing;
  mrm_.state.value = RouteLifecycleState::Initializing;
  events_.push_back({RouteChannel::Main, main_.state, std::nullopt});
  events_.push_back({RouteChannel::Mrm, mrm_.state, std::nullopt});
}

RouteSelectorCore::ChannelData &RouteSelectorCore::data(
    RouteChannel channel) {
  return channel == RouteChannel::Main ? main_ : mrm_;
}

const RouteSelectorCore::ChannelData &RouteSelectorCore::data(
    RouteChannel channel) const {
  return channel == RouteChannel::Main ? main_ : mrm_;
}

void RouteSelectorCore::changeState(
    RouteChannel channel, RouteLifecycleState state, double now_s) {
  auto &target = data(channel);
  target.state = {state, now_s};
  events_.push_back({channel, target.state, std::nullopt});
}

RouteSelectorStatus RouteSelectorCore::resumeMain(
    const RouteSelectorRequest &clear_request,
    const RouteSelectorPlannerCall &planner_call) {
  if (!main_request_) {
    return planner_call(clear_request);
  }
  if (main_.route) {
    RouteSelectorRequest request;
    request.kind = RouteCommandKind::SetLanelet;
    request.uuid = main_.route->uuid;
    request.goal = main_.route->goal;
    request.lanelet_route = main_.route->route;
    request.allow_modification = main_.route->allow_modification;
    request.stamp_s = main_.route->stamp_s;
    const auto status = planner_call(request);
    if (status.success) return status;
  }
  // 원본 selector는 저장된 구간 복구에 실패하면 경유점을 버리고
  // 원래 목표만 사용해 waypoint route를 다시 계산한다.
  RouteSelectorRequest request = *main_request_;
  request.kind = RouteCommandKind::SetWaypoint;
  request.waypoints.clear();
  return planner_call(request);
}

RouteSelectorStatus RouteSelectorCore::handle(
    RouteChannel channel, RouteSelectorRequest request,
    double now_s, const RouteSelectorPlannerCall &planner_call) {
  if (!planner_call) throw std::invalid_argument("planner_call is empty");
  if (request.kind != RouteCommandKind::Clear)
    generateUuidIfEmpty(request.uuid);

  if (channel == RouteChannel::Main) {
    if (request.kind == RouteCommandKind::Clear)
      main_request_.reset();
    else
      main_request_ = request;
    main_.route.reset();
    if (mrm_operating_) {
      changeState(channel,
                  request.kind == RouteCommandKind::Clear ?
                      RouteLifecycleState::Unset :
                      RouteLifecycleState::Interrupted,
                  now_s);
      return {true, 0, {}};
    }
    return planner_call(request);
  }

  if (request.kind == RouteCommandKind::Clear) {
    const auto status = resumeMain(request, planner_call);
    if (status.success) {
      mrm_operating_ = false;
      changeState(RouteChannel::Mrm, RouteLifecycleState::Unset, now_s);
    }
    return status;
  }

  const auto status = planner_call(request);
  if (status.success) {
    mrm_operating_ = true;
    if (main_.state.value != RouteLifecycleState::Unset)
      changeState(RouteChannel::Main, RouteLifecycleState::Interrupted,
                  now_s);
  }
  return status;
}

void RouteSelectorCore::onPlannerState(RouteSelectorState state) {
  if (state.value == RouteLifecycleState::Unset && !initialized_) {
    changeState(RouteChannel::Main, RouteLifecycleState::Unset,
                state.stamp_s);
    changeState(RouteChannel::Mrm, RouteLifecycleState::Unset,
                state.stamp_s);
    initialized_ = true;
  }
  const auto channel = mrm_operating_ ? RouteChannel::Mrm : RouteChannel::Main;
  data(channel).state = state;
  events_.push_back({channel, state, std::nullopt});
}

void RouteSelectorCore::onPlannerRoute(const RouteSelectorRoute &route) {
  const auto channel = mrm_operating_ ? RouteChannel::Mrm : RouteChannel::Main;
  data(channel).route = route;
  events_.push_back({channel, data(channel).state, route});
}

RouteSelectorState RouteSelectorCore::state(RouteChannel channel) const {
  return data(channel).state;
}

std::optional<RouteSelectorRoute> RouteSelectorCore::route(
    RouteChannel channel) const {
  return data(channel).route;
}

std::vector<RouteSelectorEvent> RouteSelectorCore::takeEvents() {
  auto result = std::move(events_);
  events_.clear();
  return result;
}

}  // namespace planning
