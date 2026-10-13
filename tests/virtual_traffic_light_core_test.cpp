// 가상 신호등의 승인·지연·완료 분기와 완료 상태 유지를 검사합니다.

#include "planning/virtual_traffic_light_core.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

int main() {
  using namespace planning;
  VirtualTrafficLightCore core({1.0, true});
  VirtualTrafficLightInput input;
  input.now_s = 10.0;
  input.before_start_line = true;
  assert(core.apply(input).state == VirtualTrafficLightState::None);
  input.before_start_line = false;
  assert(core.apply(input).stop == VirtualTrafficLightStop::StopLine);
  input.message = VirtualTrafficLightMessage{9.0, false, false};
  assert(core.apply(input).stop == VirtualTrafficLightStop::StopLine);
  input.message->approved = true;
  assert(core.apply(input).stop == VirtualTrafficLightStop::None);
  input.now_s = 10.001;
  assert(core.apply(input).stop == VirtualTrafficLightStop::StopLine);
  input.before_stop_line = false;
  assert(core.apply(input).stop == VirtualTrafficLightStop::StopLine);
  input.message->stamp_s = 10.0;
  assert(core.apply(input).stop == VirtualTrafficLightStop::EndLine);
  input.near_end_line = true;
  input.vehicle_stopped = true;
  auto result = core.apply(input);
  assert(result.state == VirtualTrafficLightState::Finalizing);
  assert(result.stop == VirtualTrafficLightStop::EndLine);
  input.message->finalized = true;
  result = core.apply(input);
  assert(result.state == VirtualTrafficLightState::Passing);
  assert(result.stop == VirtualTrafficLightStop::None);
  input.after_end_line = true;
  assert(core.apply(input).state == VirtualTrafficLightState::Finalized);
  input.after_end_line = false;
  assert(core.apply(input).state == VirtualTrafficLightState::Finalized);

  VirtualTrafficLightCore without_line;
  input.has_stop_line = false;
  assert(without_line.apply(input).stop == VirtualTrafficLightStop::None);

  VirtualTrafficLightPathInput path_input;
  path_input.trajectory.resize(11);
  for (std::size_t index = 0; index < path_input.trajectory.size(); ++index) {
    path_input.trajectory[index].x = static_cast<double>(index);
    path_input.trajectory[index].longitudinal_velocity_mps = 5.0F;
  }
  path_input.decision.stop = VirtualTrafficLightStop::StopLine;
  path_input.stop_line_arc_m = 7.5;
  path_input.front_offset_m = 2.0;
  auto stopped = applyVirtualTrafficLightStopVelocity(path_input);
  assert(stopped.stop_arc_m == 5.5);
  assert(stopped.trajectory.size() == 12);
  assert(stopped.trajectory[6].x == 5.5);
  assert(stopped.trajectory[5].longitudinal_velocity_mps == 5.0F);
  assert(stopped.trajectory[6].longitudinal_velocity_mps == 0.0F);
  path_input.ego_arc_m = 6.2;
  path_input.vehicle_stopped = true;
  path_input.hold_stop_margin_m = 0.1;
  stopped = applyVirtualTrafficLightStopVelocity(path_input);
  assert(stopped.stop_arc_m == 6.2);
  assert(std::abs(stopped.trajectory[7].x - 6.2) < 1.0e-9);
  path_input.stop_line_arc_m.reset();
  stopped = applyVirtualTrafficLightStopVelocity(path_input);
  assert(stopped.stop_arc_m == 0.0);
  assert(stopped.trajectory.front().longitudinal_velocity_mps == 0.0F);
  path_input.decision.stop = VirtualTrafficLightStop::EndLine;
  path_input.end_line_arc_m.reset();
  stopped = applyVirtualTrafficLightStopVelocity(path_input);
  assert(!stopped.stop_arc_m);
  path_input.before_stop_line = false;
  stopped = applyVirtualTrafficLightStopVelocity(path_input);
  assert(stopped.stop_arc_m == 0.0);
  path_input.end_line_arc_m = 9.0;
  stopped = applyVirtualTrafficLightStopVelocity(path_input);
  assert(stopped.stop_arc_m == 7.0);
  path_input.decision.stop = VirtualTrafficLightStop::None;
  stopped = applyVirtualTrafficLightStopVelocity(path_input);
  assert(!stopped.stop_arc_m);

  const std::vector<StopLineSegment> starts{
      {2.0, -1.0, 2.0, 1.0}, {4.0, -1.0, 4.0, 1.0}};
  const std::vector<StopLineSegment> stops{{5.0, -1.0, 5.0, 1.0}};
  const std::vector<StopLineSegment> ends{{9.0, -1.0, 9.0, 1.0}};
  const auto arcs = findVirtualTrafficLightLineArcs(
      path_input.trajectory, starts, stops, ends);
  assert(arcs.start_arc_m == 4.0);
  assert(arcs.stop_arc_m == 5.0);
  assert(arcs.end_arc_m == 9.0);
  auto no_end = findVirtualTrafficLightLineArcs(
      path_input.trajectory, starts, stops, {});
  assert(!no_end.start_arc_m && !no_end.stop_arc_m && !no_end.end_arc_m);
  auto early_end = findVirtualTrafficLightLineArcs(
      path_input.trajectory, starts, stops,
      {{3.0, -1.0, 3.0, 1.0}, {9.0, -1.0, 9.0, 1.0}});
  assert(early_end.start_arc_m == 2.0);
  assert(!early_end.stop_arc_m);
  assert(early_end.end_arc_m == 3.0);

  VirtualTrafficLightArcInput arc_input;
  arc_input.arcs = arcs;
  arc_input.ego_arc_m = 1.0;
  arc_input.front_offset_m = 0.5;
  auto geometry = makeVirtualTrafficLightInputFromArcs(arc_input);
  assert(geometry.before_start_line);
  assert(geometry.before_stop_line);
  assert(!geometry.after_end_line);
  arc_input.ego_arc_m = 5.0;
  geometry = makeVirtualTrafficLightInputFromArcs(arc_input);
  assert(!geometry.before_start_line);
  assert(geometry.before_stop_line);
  arc_input.ego_arc_m = 8.5;
  geometry = makeVirtualTrafficLightInputFromArcs(arc_input);
  assert(!geometry.before_stop_line);
  assert(geometry.near_end_line);
  arc_input.ego_arc_m = 9.6;
  geometry = makeVirtualTrafficLightInputFromArcs(arc_input);
  assert(geometry.after_end_line);
  VirtualTrafficLightCommandMetadata metadata;
  metadata.instrument_type = "virtual_traffic_light";
  metadata.instrument_id = "시험 신호등";
  metadata.custom_tags = {{"lane_id", "42"}};
  geometry.now_s = 12.5;
  const auto command = makeVirtualTrafficLightCommand(
      geometry, {VirtualTrafficLightState::Finalized,
                 VirtualTrafficLightStop::None}, metadata);
  assert(command && command->stamp_s == 12.5 && command->state == 4 &&
         command->metadata.instrument_id == "시험 신호등" &&
         command->metadata.custom_tags.size() == 1);
  geometry.has_end_line = false;
  assert(!makeVirtualTrafficLightCommand(
      geometry, {VirtualTrafficLightState::None,
                 VirtualTrafficLightStop::None}, metadata));
  bool invalid_rejected = false;
  try {
    findVirtualTrafficLightLineArcs(
        path_input.trajectory, starts, stops,
        {{std::numeric_limits<double>::quiet_NaN(), -1.0, 9.0, 1.0}});
  } catch (const std::invalid_argument &) {
    invalid_rejected = true;
  }
  assert(invalid_rejected);
  arc_input.arcs.end_arc_m = std::numeric_limits<double>::infinity();
  invalid_rejected = false;
  try {
    makeVirtualTrafficLightInputFromArcs(arc_input);
  } catch (const std::invalid_argument &) {
    invalid_rejected = true;
  }
  assert(invalid_rejected);
}
