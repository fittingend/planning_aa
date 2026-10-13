// Copyright 2020 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 신호등 scene.cpp와 utils.cpp의 계산을 독립 자료형으로 이관했습니다.

#include "planning/traffic_light_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace planning {
namespace {

struct Point2d {
  double x;
  double y;
};

double cross(Point2d a, Point2d b) { return a.x * b.y - a.y * b.x; }

double segmentLength(const TrajectoryPoint &a, const TrajectoryPoint &b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

std::vector<double> arcLengths(const Trajectory &path) {
  std::vector<double> arc(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    arc[i] = arc[i - 1] + segmentLength(path[i - 1], path[i]);
  }
  return arc;
}

double closestArc(const Trajectory &path, const std::vector<double> &arc,
                  double x, double y) {
  double nearest = std::numeric_limits<double>::infinity();
  double selected = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i + 1].x - path[i].x;
    const double dy = path[i + 1].y - path[i].y;
    const double squared = dx * dx + dy * dy;
    if (squared <= 1.0e-12) continue;
    const double ratio = std::clamp(
        ((x - path[i].x) * dx + (y - path[i].y) * dy) / squared, 0.0, 1.0);
    const double px = path[i].x + ratio * dx;
    const double py = path[i].y + ratio * dy;
    const double distance = (x - px) * (x - px) + (y - py) * (y - py);
    if (distance < nearest) {
      nearest = distance;
      selected = arc[i] + ratio * std::sqrt(squared);
    }
  }
  return selected;
}

std::optional<double> crossingArc(const Trajectory &path,
                                  const std::vector<double> &arc,
                                  StopLineSegment line, double extension) {
  const double line_length = std::hypot(line.x2 - line.x1, line.y2 - line.y1);
  if (line_length <= 1.0e-12) return std::nullopt;
  const double ux = (line.x2 - line.x1) / line_length;
  const double uy = (line.y2 - line.y1) / line_length;
  const Point2d a{line.x1 - extension * ux, line.y1 - extension * uy};
  const Point2d b{line.x2 + extension * ux, line.y2 + extension * uy};
  const Point2d direction{b.x - a.x, b.y - a.y};
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Point2d start{path[i].x, path[i].y};
    const Point2d path_direction{
        path[i + 1].x - start.x, path[i + 1].y - start.y};
    const double denominator = cross(path_direction, direction);
    if (std::abs(denominator) <= 1.0e-12) continue;
    const Point2d delta{a.x - start.x, a.y - start.y};
    const double path_ratio = cross(delta, direction) / denominator;
    const double line_ratio = cross(delta, path_direction) / denominator;
    if (path_ratio >= 0.0 && path_ratio <= 1.0 &&
        line_ratio >= 0.0 && line_ratio <= 1.0) {
      return arc[i] + path_ratio * segmentLength(path[i], path[i + 1]);
    }
  }
  return std::nullopt;
}

Trajectory insertStop(const Trajectory &path, const std::vector<double> &arc,
                      double stop_arc) {
  Trajectory result = path;
  const auto upper = std::upper_bound(arc.begin(), arc.end(), stop_arc);
  const std::size_t index = std::clamp<std::size_t>(
      std::distance(arc.begin(), upper), 1, path.size() - 1);
  const double ratio = (stop_arc - arc[index - 1]) /
                       (arc[index] - arc[index - 1]);
  auto point = path[index - 1];
  point.x += (path[index].x - point.x) * ratio;
  point.y += (path[index].y - point.y) * ratio;
  point.longitudinal_velocity_mps = 0.0F;

  constexpr double minimum_distance = 0.001;
  std::size_t first_zero = index;
  for (std::size_t i = index - 1; i <= std::min(index + 1, path.size() - 1); ++i) {
    if (segmentLength(path[i], point) < minimum_distance) {
      first_zero = i;
      result[i].longitudinal_velocity_mps = 0.0F;
      break;
    }
  }
  if (first_zero == index && segmentLength(path[index], point) >= minimum_distance) {
    result.insert(result.begin() + index, point);
  }
  for (std::size_t i = first_zero; i < result.size(); ++i) {
    result[i].longitudinal_velocity_mps =
        std::min(result[i].longitudinal_velocity_mps, 0.0F);
  }
  return result;
}

double judgeDistance(double velocity, double acceleration,
                     const TrafficLightParameters &parameters) {
  if (velocity <= 0.0) return 0.0;
  const double max_acc = parameters.max_stop_acceleration_mps2;
  const double max_jerk = parameters.max_stop_jerk_mps3;
  const double x1 = velocity * parameters.response_delay_s;
  const double v2 = velocity +
      (max_acc * max_acc - acceleration * acceleration) / (2.0 * max_jerk);
  if (v2 <= 0.0) {
    const double t2 = -(max_acc +
        std::sqrt(acceleration * acceleration - 2.0 * max_jerk * velocity)) /
        max_jerk;
    const double x2 = velocity * t2 + acceleration * t2 * t2 / 2.0 +
                      max_jerk * t2 * t2 * t2 / 6.0;
    return std::max(0.0, x1 + x2);
  }
  const double t2 = (max_acc - acceleration) / max_jerk;
  const double x2 = velocity * t2 + acceleration * t2 * t2 / 2.0 +
                    max_jerk * t2 * t2 * t2 / 6.0;
  const double x3 = -v2 * v2 / (2.0 * max_acc);
  return std::max(0.0, x1 + x2 + x3);
}

}  // 익명 네임스페이스

TrafficLightCore::TrafficLightCore(StopLineSegment line,
                                   TrafficLightParameters parameters)
    : line_(line), parameters_(parameters) {
  const double values[] = {
      line.x1, line.y1, line.x2, line.y2, parameters.stop_margin_m,
      parameters.front_offset_m, parameters.line_extension_m,
      parameters.signal_timeout_s, parameters.stop_hysteresis_s,
      parameters.yellow_period_s, parameters.max_stop_acceleration_mps2,
      parameters.max_stop_jerk_mps3, parameters.response_delay_s};
  for (double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("신호등 설정에는 유한한 값이 필요합니다");
    }
  }
  if (parameters.front_offset_m < 0.0 || parameters.line_extension_m < 0.0 ||
      parameters.signal_timeout_s < 0.0 || parameters.stop_hysteresis_s < 0.0 ||
      parameters.yellow_period_s < 0.0 || parameters.response_delay_s < 0.0 ||
      parameters.max_stop_acceleration_mps2 >= 0.0 ||
      parameters.max_stop_jerk_mps3 >= 0.0) {
    throw std::invalid_argument("신호등 설정이 잘못되었습니다");
  }
}

TrafficLightResult TrafficLightCore::apply(const Trajectory &path,
                                           const TrafficLightInput &input) {
  const double values[] = {input.ego_x, input.ego_y, input.ego_velocity_mps,
                           input.ego_acceleration_mps2, input.now_s};
  for (double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("차량 상태에는 유한한 값이 필요합니다");
    }
  }
  if (input.signal_stamp_s && !std::isfinite(*input.signal_stamp_s)) {
    throw std::invalid_argument("신호 시각에는 유한한 값이 필요합니다");
  }
  if (input.signal_stamp_s && input.signal_requires_stop) {
    last_signal_stamp_s_ = input.signal_stamp_s;
    last_signal_requires_stop_ = *input.signal_requires_stop;
  }
  if (parameters_.enable_rtc) activated_ = input.rtc_activated;

  TrafficLightResult result;
  result.trajectory = path;
  result.state = state_;
  result.safe = safe_;
  result.activated = activated_;
  if (path.size() < 2) return result;

  const auto arc = arcLengths(path);
  for (std::size_t i = 1; i < arc.size(); ++i) {
    if (arc[i] <= arc[i - 1]) return result;
  }
  const auto crossing = crossingArc(path, arc, line_, parameters_.line_extension_m);
  if (!crossing) {
    safe_ = true;
    if (!parameters_.enable_rtc) activated_ = true;
    result.safe = safe_;
    result.activated = activated_;
    return result;
  }

  const double stop_arc = *crossing -
      (parameters_.stop_margin_m + parameters_.front_offset_m);
  if (stop_arc < 0.0 || stop_arc > arc.back()) return result;
  result.stop_arc_m = stop_arc;
  result.distance_to_stop_m =
      stop_arc - closestArc(path, arc, input.ego_x, input.ego_y);

  if (state_ == TrafficLightState::GoOut) {
    if (result.distance_to_stop_m > 1.0) state_ = TrafficLightState::Approach;
    stop_signal_since_s_.reset();
    result.state = state_;
    return result;
  }
  if (result.distance_to_stop_m < -2.0) {
    state_ = TrafficLightState::GoOut;
    stop_signal_since_s_.reset();
    result.state = state_;
    return result;
  }

  const bool stop_signal = last_signal_stamp_s_
      ? (input.now_s - *last_signal_stamp_s_ > parameters_.signal_timeout_s ||
         last_signal_requires_stop_)
      : !parameters_.simulation;
  if (stop_signal) {
    if (!stop_signal_since_s_) stop_signal_since_s_ = input.now_s;
  } else {
    stop_signal_since_s_.reset();
  }
  const double elapsed = stop_signal_since_s_
      ? std::max(input.now_s - *stop_signal_since_s_, 0.0) : 0.0;
  const bool to_stop = stop_signal &&
      (previous_stop_ || elapsed > parameters_.stop_hysteresis_s);
  safe_ = !to_stop;
  if (!parameters_.enable_rtc) activated_ = safe_;
  result.safe = safe_;
  result.activated = activated_;
  if (activated_) {
    previous_stop_ = false;
    return result;
  }

  const double judge = judgeDistance(
      input.ego_velocity_mps, input.ego_acceleration_mps2, parameters_);
  const bool stoppable = judge < result.distance_to_stop_m ||
                         input.ego_velocity_mps < 2.0;
  const bool reachable = result.distance_to_stop_m <
                         input.ego_velocity_mps * parameters_.yellow_period_s;
  if (parameters_.enable_pass_judge && !stoppable && !previous_stop_ && reachable) {
    return result;
  }

  result.trajectory = insertStop(path, arc, stop_arc);
  result.applied_stop = true;
  previous_stop_ = true;
  return result;
}

TrafficLightResult TrafficLightCore::apply(
    const Trajectory &path, TrafficLightInput input,
    const TrafficLightSignalObservation &observation) {
  input.signal_stamp_s = observation.stamp_s;
  input.signal_requires_stop = trafficSignalRequiresStop(
      observation.lane_turn, observation.elements);
  return apply(path, input);
}

}  // planning 네임스페이스
