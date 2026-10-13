// SPDX-License-Identifier: Apache-2.0
// 원본 차로 변경의 시간별 속도·이동 거리 공식을 독립 경로에 적용합니다.

#include "planning/behavior_path_prediction.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace planning {
namespace {

double yaw(const TrajectoryPoint &point) {
  return std::atan2(
      2.0 * (point.orientation_w * point.orientation_z +
             point.orientation_x * point.orientation_y),
      1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                   point.orientation_z * point.orientation_z));
}

BehaviorSafetyPose sample(const Trajectory &path,
                          const std::vector<double> &arcs,
                          double time, double arc, double velocity) {
  const double clamped = std::clamp(arc, 0.0, arcs.back());
  auto upper = std::lower_bound(arcs.begin(), arcs.end(), clamped);
  if (upper == arcs.begin()) {
    return {time, path.front().x, path.front().y,
            yaw(path.front()), velocity};
  }
  if (upper == arcs.end()) {
    return {time, path.back().x, path.back().y,
            yaw(path.back()), velocity};
  }
  const std::size_t after = static_cast<std::size_t>(upper - arcs.begin());
  const std::size_t before = after - 1;
  const double length = arcs[after] - arcs[before];
  const double ratio = length > 1.0e-9 ?
      std::clamp((clamped - arcs[before]) / length, 0.0, 1.0) : 0.0;
  const double angle = yaw(path[before]);
  const double turn = std::remainder(yaw(path[after]) - angle,
                                     2.0 * std::acos(-1.0));
  return {time,
          path[before].x + ratio * (path[after].x - path[before].x),
          path[before].y + ratio * (path[after].y - path[before].y),
          angle + ratio * turn, velocity};
}

}  // 익명 네임스페이스

std::vector<BehaviorSafetyPose> predictBehaviorLaneChangeEgoPath(
    const Trajectory &path,
    const BehaviorLaneChangePredictionParameters &parameters) {
  if (path.empty()) return {};
  if (!std::isfinite(parameters.ego_arc_m) ||
      !std::isfinite(parameters.initial_velocity_mps) ||
      parameters.initial_velocity_mps < 0.0 ||
      !std::isfinite(parameters.prepare_duration_s) ||
      parameters.prepare_duration_s < 0.0 ||
      !std::isfinite(parameters.lane_change_duration_s) ||
      parameters.lane_change_duration_s < 0.0 ||
      !std::isfinite(parameters.prepare_acceleration_mps2) ||
      !std::isfinite(parameters.lane_change_acceleration_mps2) ||
      parameters.prepare_velocity_limit_mps < 0.0 ||
      parameters.lane_change_velocity_limit_mps < 0.0 ||
      !std::isfinite(parameters.prepare_velocity_limit_mps) ||
      !std::isfinite(parameters.lane_change_velocity_limit_mps) ||
      !std::isfinite(parameters.resolution_s) ||
      parameters.resolution_s <= 0.0) {
    throw std::invalid_argument("차로 변경 예측 매개변수가 유효하지 않습니다");
  }
  std::vector<double> arcs(path.size(), 0.0);
  for (std::size_t index = 1; index < path.size(); ++index) {
    arcs[index] = arcs[index - 1] +
        std::hypot(path[index].x - path[index - 1].x,
                   path[index].y - path[index - 1].y);
  }
  std::vector<BehaviorSafetyPose> predicted;
  const double preparation = parameters.prepare_duration_s;
  const double duration = preparation + parameters.lane_change_duration_s;
  predicted.reserve(static_cast<std::size_t>(
      std::ceil(duration / parameters.resolution_s)) + 2);
  for (double time = 0.0; time < preparation;
       time += parameters.resolution_s) {
    const double speed = std::clamp(
        parameters.initial_velocity_mps +
            parameters.prepare_acceleration_mps2 * time,
        0.0, parameters.prepare_velocity_limit_mps);
    const double travel = parameters.initial_velocity_mps * time +
        0.5 * parameters.prepare_acceleration_mps2 * time * time;
    predicted.push_back(sample(path, arcs, time,
                               parameters.ego_arc_m + travel, speed));
  }
  const double change_initial_speed = std::clamp(
      parameters.initial_velocity_mps +
          parameters.prepare_acceleration_mps2 * preparation,
      0.0, parameters.prepare_velocity_limit_mps);
  const double preparation_distance =
      parameters.initial_velocity_mps * preparation +
      0.5 * parameters.prepare_acceleration_mps2 *
          preparation * preparation;
  for (double time = preparation; time < duration;
       time += parameters.resolution_s) {
    const double elapsed = time - preparation;
    const double speed = std::clamp(
        change_initial_speed +
            parameters.lane_change_acceleration_mps2 * elapsed,
        0.0, parameters.lane_change_velocity_limit_mps);
    const double travel = preparation_distance +
        change_initial_speed * elapsed +
        0.5 * parameters.lane_change_acceleration_mps2 *
            elapsed * elapsed;
    predicted.push_back(sample(path, arcs, time,
                               parameters.ego_arc_m + travel, speed));
  }
  return predicted;
}

std::vector<std::vector<BehaviorSafetyPose>>
predictBehaviorLaneChangeEgoPaths(
    const Trajectory &path,
    const BehaviorLaneChangePredictionParameters &parameters,
    double global_min_acceleration_mps2,
    std::size_t deceleration_sampling_num) {
  if (!std::isfinite(global_min_acceleration_mps2) ||
      deceleration_sampling_num == 0) {
    throw std::invalid_argument("차로 변경 감속 표본 입력이 유효하지 않습니다");
  }
  const double minimum_acceleration = std::min(
      parameters.lane_change_acceleration_mps2,
      global_min_acceleration_mps2);
  const std::size_t count = std::abs(
      minimum_acceleration - parameters.lane_change_acceleration_mps2) >
      1.0e-3 ? deceleration_sampling_num : 1;
  const double resolution = (
      minimum_acceleration - parameters.lane_change_acceleration_mps2) /
      static_cast<double>(count);
  std::vector<std::vector<BehaviorSafetyPose>> result;
  result.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    auto sample_parameters = parameters;
    sample_parameters.lane_change_acceleration_mps2 +=
        static_cast<double>(index) * resolution;
    result.push_back(predictBehaviorLaneChangeEgoPath(
        path, sample_parameters));
  }
  return result;
}

}  // planning 네임스페이스
