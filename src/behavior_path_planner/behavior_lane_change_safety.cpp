// SPDX-License-Identifier: Apache-2.0
// 원본 차로 변경 후보의 객체 경로 시간 필터와 전체 자차 경로 판정을 묶습니다.

#include "planning/behavior_lane_change_safety.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <utility>

namespace planning {
namespace {

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
using Line = bg::model::linestring<Point>;

Polygon checkedPolygon(const BehaviorSafetyPolygon &points) {
  if (points.size() < 3) {
    throw std::invalid_argument("차로 변경 객체 분류 다각형이 유효하지 않습니다");
  }
  Polygon polygon;
  for (const auto &[x, y] : points) {
    if (!std::isfinite(x) || !std::isfinite(y)) {
      throw std::invalid_argument("차로 변경 객체 분류 좌표가 유효하지 않습니다");
    }
    polygon.outer().emplace_back(x, y);
  }
  bg::correct(polygon);
  // The original lane-change object filter calls Boost.Geometry intersects
  // on Lanelet2's expanded polygon without an is_valid gate. Curved offsets
  // may fold locally; rejecting the whole candidate before collision checking
  // changes the module's safety decision.
  return polygon;
}

bool predictedPathOverlapsLane(
    const std::vector<BehaviorSafetyPredictedPath> &paths,
    const Polygon &lane) {
  for (const auto &path : paths) {
    Line line;
    for (const auto &pose : path.poses) {
      if (!std::isfinite(pose.x) || !std::isfinite(pose.y)) {
        throw std::invalid_argument("차로 변경 객체 예측 좌표가 유효하지 않습니다");
      }
      line.emplace_back(pose.x, pose.y);
    }
    if (line.size() == 1 && bg::intersects(line.front(), lane)) return true;
    if (line.size() > 1 && bg::intersects(line, lane)) return true;
  }
  return false;
}

std::vector<BehaviorSafetyPredictedPath> filteredPaths(
    const BehaviorLaneChangeTarget &target,
    const BehaviorLaneChangeSafetyParameters &parameters) {
  std::vector<BehaviorSafetyPredictedPath> paths;
  paths.reserve(target.predicted_paths.size());
  const double start = parameters.check_prepare_phase ?
      0.0 : parameters.prepare_duration_s;
  for (const auto &source : target.predicted_paths) {
    BehaviorSafetyPredictedPath filtered;
    filtered.confidence = source.confidence;
    for (const auto &pose : source.poses) {
      if (pose.time_s < start ||
          (pose.time_s < parameters.prepare_duration_s &&
           pose.velocity_mps <
               parameters.stopped_object_velocity_threshold_mps)) {
        continue;
      }
      filtered.poses.push_back(pose);
    }
    paths.push_back(std::move(filtered));
  }
  return paths;
}

std::optional<BehaviorLaneChangeSafetyResult> allPathsCollide(
    const std::vector<std::vector<BehaviorSafetyPose>> &ego_paths,
    const std::vector<BehaviorLaneChangeTarget> &targets,
    const BehaviorLaneChangeSafetyParameters &parameters,
    bool object_behind_ego) {
  if (ego_paths.empty() || targets.empty()) return std::nullopt;
  BehaviorSafetyRegions collision_regions;
  if (parameters.check_lane_regions) {
    if (parameters.current_lane_region.empty() ||
        parameters.expanded_target_lane_region.empty()) {
      return std::nullopt;
    }
    collision_regions = {parameters.current_lane_region,
                         parameters.expanded_target_lane_region};
  }
  BehaviorLaneChangeSafetyResult last;
  last.safe = false;
  last.object_behind_ego = object_behind_ego;
  for (std::size_t ego_index = 0; ego_index < ego_paths.size(); ++ego_index) {
    bool collided = false;
    for (std::size_t object_index = 0; object_index < targets.size();
         ++object_index) {
      const auto &target = targets[object_index];
      const auto &rss =
          target.initial_velocity_mps <=
                  parameters.stopped_object_velocity_threshold_mps ?
              parameters.parked_object : parameters.moving_object;
      const auto paths = filteredPaths(target, parameters);
      const auto collision = firstBehaviorPathObjectCollisionAcrossPaths(
          ego_paths[ego_index], parameters.ego_vehicle,
          target.object, paths, parameters.use_all_predicted_paths,
          rss, collision_regions);
      if (collision) {
        collided = true;
        last.ego_path_index = ego_index;
        last.object_index = object_index;
        last.collision = collision;
        break;
      }
    }
    if (!collided) return std::nullopt;
  }
  return last;
}

}  // 익명 네임스페이스

BehaviorLaneChangeRtcChannel behaviorLaneChangeRtcChannelFromStatus(
    const BehaviorLaneChangeRtcStatus &status) {
  const bool terminated =
      status.state == BehaviorLaneChangeRtcState::Succeeded ||
      status.state == BehaviorLaneChangeRtcState::Failed;
  const bool approved = !terminated &&
      ((status.auto_mode && !status.requested) ? status.safe :
                                                    status.command_activated);
  const bool force_activated =
      (status.state == BehaviorLaneChangeRtcState::Waiting ||
       status.state == BehaviorLaneChangeRtcState::Running) &&
      status.command_activated && (!status.safe || status.requested);
  const bool force_deactivated =
      status.state == BehaviorLaneChangeRtcState::Running &&
      !status.command_activated && status.safe && !status.auto_mode;
  return {true, approved, terminated, force_activated, force_deactivated};
}

BehaviorLaneChangeRtcSignals assembleBehaviorLaneChangeRtcSignals(
    const std::vector<BehaviorLaneChangeRtcChannel> &channels,
    bool execution_ready, bool rtc_interfaces_configured) {
  BehaviorLaneChangeRtcSignals result;
  result.ready = execution_ready;
  result.activated = !rtc_interfaces_configured;
  for (const auto &channel : channels) {
    result.activated |= channel.registered && channel.approved &&
                        !channel.terminated;
    result.force_activated |= channel.force_activated;
    result.force_deactivated |= channel.force_deactivated;
  }
  return result;
}

std::optional<Trajectory> generateBehaviorLaneChangeAbortPath(
    const BehaviorLaneChangeAbortInput &input) {
  const auto finite_nonnegative = [](double value) {
    return std::isfinite(value) && value >= 0.0;
  };
  if (input.selected_path.size() < 4 ||
      input.current_lane_reference.size() < 2 ||
      input.previous_module_path.empty() ||
      !std::isfinite(input.ego.x) || !std::isfinite(input.ego.y) ||
      !finite_nonnegative(input.ego_velocity_mps) ||
      !finite_nonnegative(input.minimum_lane_change_velocity_mps) ||
      !finite_nonnegative(input.delta_time_s) ||
      !finite_nonnegative(input.return_duration_s) ||
      !finite_nonnegative(input.maximum_lateral_jerk_mps3) ||
      !finite_nonnegative(input.maximum_lateral_acceleration_mps2) ||
      !finite_nonnegative(input.stopping_distance_m) ||
      !finite_nonnegative(input.distance_to_terminal_start_m)) return std::nullopt;
  const auto nearest = [](const Trajectory &path, double x, double y) {
    std::size_t best = 0;
    double distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path.size(); ++i) {
      const double dx = path[i].x - x;
      const double dy = path[i].y - y;
      const double squared = dx * dx + dy * dy;
      if (squared < distance) { distance = squared; best = i; }
    }
    return best;
  };
  const auto &selected = input.selected_path;
  const auto &terminal = input.current_lane_reference.back();
  const std::size_t end = nearest(selected, terminal.x, terminal.y);
  const std::size_t ego = nearest(selected, input.ego.x, input.ego.y);
  if (ego >= end) return std::nullopt;
  const double speed = std::max(input.minimum_lane_change_velocity_mps,
                                input.ego_velocity_mps);
  const auto position_at_time = [&](double duration) {
    double distance = std::hypot(selected[ego].x - input.ego.x,
                                 selected[ego].y - input.ego.y);
    for (std::size_t i = ego + 1; i <= end; ++i) {
      distance += std::hypot(selected[i].x - selected[i - 1].x,
                             selected[i].y - selected[i - 1].y);
      if (distance > speed * duration) return std::pair{i, distance};
    }
    return std::pair{ego, 0.0};
  };
  const auto [start, start_distance] = position_at_time(input.delta_time_s);
  const auto [return_index, return_distance] = position_at_time(
      input.delta_time_s + input.return_duration_s);
  if (return_index <= start + 1 ||
      start_distance + return_distance + input.stopping_distance_m >
          input.distance_to_terminal_start_m) return std::nullopt;
  const auto &point = selected[return_index];
  const std::size_t nearest_lane = nearest(
      input.current_lane_reference, point.x, point.y);
  const std::size_t segment = std::min(
      nearest_lane, input.current_lane_reference.size() - 2);
  const auto &lane_a = input.current_lane_reference[segment];
  const auto &lane_b = input.current_lane_reference[segment + 1];
  const double vx = lane_b.x - lane_a.x;
  const double vy = lane_b.y - lane_a.y;
  const double norm = std::hypot(vx, vy);
  if (norm < 1.0e-6) return std::nullopt;
  const double shift = -((point.x - lane_a.x) * vy -
                         (point.y - lane_a.y) * vx) / norm;
  const double jerk = 0.5 * std::abs(shift) *
      std::pow(4.0 * speed / std::max(start_distance, 1.0e-3), 3);
  if (jerk > input.maximum_lateral_jerk_mps3) return std::nullopt;
  BehaviorPathShiftParameters parameters;
  parameters.velocity_mps = speed;
  parameters.lateral_acceleration_limit_mps2 =
      input.maximum_lateral_acceleration_mps2;
  Trajectory shifted;
  try {
    shifted = shiftBehaviorPath(selected, {{start, return_index, shift}},
                                parameters).path;
  } catch (const std::exception &) {
    return std::nullopt;
  }
  const std::size_t join = nearest(input.previous_module_path,
                                   shifted[return_index].x,
                                   shifted[return_index].y);
  Trajectory result(shifted.begin(), shifted.begin() +
      static_cast<std::ptrdiff_t>(return_index));
  result.insert(result.end(), input.previous_module_path.begin() +
      static_cast<std::ptrdiff_t>(join), input.previous_module_path.end());
  return result.empty() ? std::nullopt :
      std::optional<Trajectory>{std::move(result)};
}

BehaviorLaneChangeObjectGroup classifyBehaviorLaneChangeObject(
    const BehaviorLaneChangeObjectFilterInput &input) {
  const double values[] = {
      input.object_lateral_from_current_center_m,
      input.ego_lateral_from_current_center_m,
      input.vehicle_width_m, input.initial_velocity_mps,
      input.stopped_velocity_threshold_mps};
  for (const auto value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("차로 변경 객체 분류 값이 유효하지 않습니다");
    }
  }
  if (input.vehicle_width_m <= 0.0 ||
      input.stopped_velocity_threshold_mps <= 0.0) {
    throw std::invalid_argument("차로 변경 객체 분류 범위가 유효하지 않습니다");
  }
  const auto object = checkedPolygon(input.initial_polygon);
  const auto current = checkedPolygon(input.current_lane);
  const auto target = checkedPolygon(input.target_lane);
  const auto expanded = checkedPolygon(input.expanded_target_lane);
  const bool lateral_far = std::abs(
      input.object_lateral_from_current_center_m -
      input.ego_lateral_from_current_center_m) >
      input.vehicle_width_m * 0.5;
  const bool stopped = std::abs(input.initial_velocity_mps) <
      input.stopped_velocity_threshold_mps;
  if (lateral_far && input.before_terminal) {
    const bool target_overlap = bg::intersects(object, target) ||
        (!stopped && input.is_vehicle &&
         predictedPathOverlapsLane(input.predicted_paths, target));
    if (target_overlap) {
      if (!input.ahead_of_ego && !stopped) {
        return BehaviorLaneChangeObjectGroup::TargetTrailing;
      }
      if (input.ahead_of_ego) {
        return stopped ?
            BehaviorLaneChangeObjectGroup::TargetLeadingStopped :
            BehaviorLaneChangeObjectGroup::TargetLeadingMoving;
      }
    }
    if (stopped && input.ahead_of_ego &&
        bg::intersects(object, expanded)) {
      return BehaviorLaneChangeObjectGroup::TargetLeadingStoppedAtBoundary;
    }
  }
  if (!stopped) {
    for (const auto &points : input.preceding_target_lanes) {
      if (bg::intersects(object, checkedPolygon(points))) {
        return BehaviorLaneChangeObjectGroup::TargetTrailing;
      }
    }
  }
  if (input.ahead_of_ego && input.before_terminal &&
      bg::intersects(object, current)) {
    return BehaviorLaneChangeObjectGroup::CurrentLeading;
  }
  return BehaviorLaneChangeObjectGroup::Other;
}

BehaviorLaneChangeSelectedTargets selectBehaviorLaneChangeTargets(
    const std::vector<BehaviorLaneChangeSelectableObject> &objects,
    bool check_current_lane, bool check_other_lanes, bool ego_stuck) {
  using Entry = std::pair<double, BehaviorLaneChangeTarget>;
  std::array<std::vector<Entry>,
             static_cast<std::size_t>(BehaviorLaneChangeObjectGroup::Other) + 1>
      groups;
  for (const auto &object : objects) {
    const auto &filter = object.filter;
    if (!std::isfinite(object.signed_distance_from_ego_m) ||
        !std::isfinite(filter.ego_yaw_rad) ||
        !std::isfinite(filter.object_yaw_rad) ||
        !std::isfinite(filter.incoming_yaw_threshold_rad) ||
        filter.incoming_yaw_threshold_rad < 0.0 ||
        !std::isfinite(filter.initial_velocity_mps)) {
      throw std::invalid_argument("차로 변경 객체 선별 값이 유효하지 않습니다");
    }
    if (!filter.type_allowed) continue;
    const double yaw_difference = std::abs(std::remainder(
        filter.ego_yaw_rad - filter.object_yaw_rad,
        2.0 * std::acos(-1.0)));
    if (yaw_difference > filter.incoming_yaw_threshold_rad &&
        std::abs(filter.initial_velocity_mps) >= 0.5) continue;
    const auto group = classifyBehaviorLaneChangeObject(object.filter);
    groups[static_cast<std::size_t>(group)].emplace_back(
        object.signed_distance_from_ego_m, object.target);
  }
  const auto ascending = [](const Entry &first, const Entry &second) {
    return first.first < second.first;
  };
  const auto descending = [](const Entry &first, const Entry &second) {
    return first.first > second.first;
  };
  for (std::size_t index = 0; index < groups.size() - 1; ++index) {
    if (index == static_cast<std::size_t>(
                     BehaviorLaneChangeObjectGroup::TargetTrailing)) {
      std::stable_sort(groups[index].begin(), groups[index].end(), descending);
    } else {
      std::stable_sort(groups[index].begin(), groups[index].end(), ascending);
    }
  }
  BehaviorLaneChangeSelectedTargets selected;
  const auto append = [&](BehaviorLaneChangeObjectGroup group,
                          std::vector<BehaviorLaneChangeTarget> &destination) {
    for (const auto &entry : groups[static_cast<std::size_t>(group)]) {
      destination.push_back(entry.second);
    }
  };
  append(BehaviorLaneChangeObjectGroup::TargetLeadingMoving,
         selected.leading);
  append(BehaviorLaneChangeObjectGroup::TargetLeadingStopped,
         selected.leading);
  append(BehaviorLaneChangeObjectGroup::TargetLeadingStoppedAtBoundary,
         selected.leading);
  if (check_current_lane || ego_stuck) {
    append(BehaviorLaneChangeObjectGroup::CurrentLeading,
           selected.leading);
  }
  if (check_other_lanes) {
    append(BehaviorLaneChangeObjectGroup::Other, selected.leading);
  }
  append(BehaviorLaneChangeObjectGroup::TargetTrailing,
         selected.trailing);
  return selected;
}

BehaviorLaneChangeSafetyResult evaluateBehaviorLaneChangeObjectSafety(
    const std::vector<std::vector<BehaviorSafetyPose>> &ego_paths,
    const std::vector<BehaviorLaneChangeTarget> &trailing,
    const std::vector<BehaviorLaneChangeTarget> &leading,
    const BehaviorLaneChangeSafetyParameters &parameters) {
  if (!std::isfinite(parameters.prepare_duration_s) ||
      parameters.prepare_duration_s < 0.0 ||
      !std::isfinite(parameters.stopped_object_velocity_threshold_mps) ||
      parameters.stopped_object_velocity_threshold_mps < 0.0) {
    throw std::invalid_argument("차로 변경 안전 매개변수가 유효하지 않습니다");
  }
  if (const auto trailing_collision = allPathsCollide(
          ego_paths, trailing, parameters, true)) {
    return *trailing_collision;
  }
  if (const auto leading_collision = allPathsCollide(
          ego_paths, leading, parameters, false)) {
    return *leading_collision;
  }
  return {};
}

BehaviorLaneChangeSafetyResult evaluateBehaviorLaneChangeCandidateSafety(
    const Trajectory &candidate_path,
    const BehaviorLaneChangeCandidateSafetyInput &input) {
  const auto ego_paths = predictBehaviorLaneChangeEgoPaths(
      candidate_path, input.prediction,
      input.global_min_acceleration_mps2,
      input.deceleration_sampling_num);
  if (ego_paths.empty() || ego_paths.front().empty()) return {};
  const auto targets = selectBehaviorLaneChangeTargets(
      input.objects, input.check_current_lane,
      input.check_other_lanes, input.ego_stuck);
  return evaluateBehaviorLaneChangeObjectSafety(
      ego_paths, targets.trailing, targets.leading, input.safety);
}

BehaviorLaneChangeCandidateSearchResult searchBehaviorLaneChangeCandidates(
    const BehaviorLaneChangeCandidateSearchInput &input) {
  LaneChangeCandidateLengthGate length_gate(
      input.prepare_length_threshold_m, input.shift_length_threshold_m);
  BehaviorLaneChangeCandidateSearchResult result;
  for (const auto &prepare : input.prepare_plans) {
    if (prepare.prepare_path.empty() ||
        !length_gate.acceptsPrepare(prepare.phase.length_m)) {
      continue;
    }
    const auto &start = prepare.prepare_path.back();
    for (const auto &shift : prepare.shifts) {
      if (!length_gate.acceptsShift(
              prepare.phase.length_m, shift.phase.length_m)) {
        continue;
      }
      std::optional<LaneChangeShiftCandidate> path;
      try {
        path = generateLaneChangeShiftCandidate(
            prepare.prepare_path, shift.target_reference,
            {start.x, start.y}, shift.change_end,
            shift.lateral_shift_m,
            prepare.phase.terminal_velocity_mps,
            shift.phase.actual_longitudinal_acceleration_mps2,
            shift.phase.lateral_acceleration_mps2,
            shift.phase.terminal_velocity_mps);
      } catch (const std::exception &) {
        continue;
      }
      if (!path) continue;
      length_gate.recordConstructed(
          prepare.phase.length_m, shift.phase.length_m);
      auto safety = input.safety;
      safety.prediction.prepare_duration_s = prepare.phase.duration_s;
      safety.prediction.lane_change_duration_s = shift.phase.duration_s;
      safety.prediction.prepare_acceleration_mps2 =
          prepare.phase.actual_longitudinal_acceleration_mps2;
      safety.prediction.lane_change_acceleration_mps2 =
          shift.phase.actual_longitudinal_acceleration_mps2;
      safety.safety.prepare_duration_s = prepare.phase.duration_s;
      BehaviorLaneChangeSafetyResult verdict;
      try {
        verdict = evaluateBehaviorLaneChangeCandidateSafety(
            path->path, safety);
      } catch (const std::invalid_argument &) {
        // A malformed perception footprint or lane polygon cannot establish
        // safety. Keep the candidate rejected while other modules continue.
        verdict.safe = false;
      }
      result.candidates.push_back({
          std::move(*path), prepare.phase, shift.phase, verdict});
      if (verdict.safe) {
        result.first_safe_index = result.candidates.size() - 1;
        return result;
      }
    }
  }
  return result;
}

BehaviorLaneChangeFailureDecision decideBehaviorLaneChangeFailureTransition(
    const BehaviorLaneChangeFailureInput &input) {
  if (input.force_activated) {
    return {input.current_state, input.current_state, {}, false};
  }
  using State = BehaviorLaneChangeState;
  std::pair<State, std::string_view> reported;
  if (input.current_state == State::Abort) {
    reported = input.abort_finished ?
        std::pair{State::Cancel, std::string_view{"Aborted"}} :
        std::pair{State::Abort, std::string_view{"Aborting"}};
  } else if (input.waiting_approval) {
    reported = input.near_regulatory_element ?
        std::pair{State::Cancel, std::string_view{"CloseToRegElement"}} :
        std::pair{State::Normal, std::string_view{"WaitingForApproval"}};
  } else if (!input.path_valid) {
    reported = {State::Cancel, "InvalidPath"};
  } else if (input.preparing && input.force_deactivated &&
             input.can_return_current_lane) {
    reported = {State::Cancel, "ForceDeactivation"};
  } else if (input.post_process_safe) {
    reported = {State::Normal, "SafeToLaneChange"};
  } else if (!input.cancel_enabled) {
    reported = {State::Warning, "CancelDisabled"};
  } else if (input.preparing && input.can_return_current_lane) {
    reported = {State::Cancel, "SafeToCancel"};
  } else if (input.near_terminal) {
    reported = {State::Warning, "TooNearTerminal"};
  } else if (!input.abort_enabled) {
    reported = {State::Warning, "AbortDisabled"};
  } else if (!input.can_return_current_lane) {
    reported = {State::Warning, "TooLateToAbort"};
  } else if (!input.abort_path_available) {
    reported = {State::Warning, "AbortPathNotFound"};
  } else {
    reported = {State::Abort, "SafeToAbort"};
  }

  if (reported.first == State::Cancel) {
    return {reported.first, State::Cancel, reported.second, true};
  }
  if (reported.first == State::Abort) {
    return {reported.first, State::Abort, reported.second, false};
  }
  const auto module_state = !input.post_process_safe && input.stop_required ?
      State::Stop : State::Normal;
  return {reported.first, module_state, reported.second, false};
}

BehaviorLaneChangeExecutionResult executeBehaviorLaneChange(
    const BehaviorLaneChangeExecutionInput &input) {
  auto failure = input.failure;
  bool activated = input.activated;
  if (input.use_rtc_channels || !input.rtc_statuses.empty()) {
    auto channels = input.rtc_channels;
    for (const auto &status : input.rtc_statuses) {
      channels.push_back(behaviorLaneChangeRtcChannelFromStatus(status));
    }
    const auto rtc = assembleBehaviorLaneChangeRtcSignals(
        channels, true);
    activated = rtc.activated;
    failure.force_activated = rtc.force_activated;
    failure.force_deactivated = rtc.force_deactivated;
  }
  auto abort_path = input.abort_path;
  if (!abort_path && input.abort_generation) {
    abort_path = generateBehaviorLaneChangeAbortPath(*input.abort_generation);
  }
  failure.waiting_approval = !failure.force_activated &&
      (failure.waiting_approval || !activated);
  failure.abort_path_available = abort_path.has_value();
  auto decision = decideBehaviorLaneChangeFailureTransition(failure);
  BehaviorLaneChangeExecutionResult result{
      input.fallback_path, decision, std::nullopt};
  if (decision.module_state == BehaviorLaneChangeState::Abort &&
      abort_path && !abort_path->empty()) {
    result.path = *abort_path;
    return result;
  }
  if (decision.module_state == BehaviorLaneChangeState::Stop &&
      input.stop_path && !input.stop_path->empty()) {
    result.path = *input.stop_path;
    return result;
  }
  if (decision.module_state != BehaviorLaneChangeState::Normal ||
      (!activated && !failure.force_activated) ||
      failure.waiting_approval ||
      decision.reported_state == BehaviorLaneChangeState::Cancel) {
    return result;
  }
  const auto index = failure.force_activated &&
                             !input.search.candidates.empty() ?
      std::optional<std::size_t>{0} : input.search.first_safe_index;
  if (!index || *index >= input.search.candidates.size()) return result;
  result.path = input.search.candidates[*index].path.path;
  result.selected_candidate_index = *index;
  return result;
}

}  // planning 네임스페이스
