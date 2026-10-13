// Copyright 2022 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 돌발 진입 모듈의 상태 전환, 제동과 시간 유지 규칙을 이관했습니다.

#include "planning/run_out_state.hpp"
#include "planning/trajectory_stop.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace planning {
namespace {

struct BrakingMotion {
  double distance = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
};

BrakingMotion advanceBraking(
    BrakingMotion motion, double jerk, double duration) {
  motion.distance += motion.velocity * duration +
      0.5 * motion.acceleration * duration * duration +
      jerk * duration * duration * duration / 6.0;
  motion.velocity += motion.acceleration * duration +
      0.5 * jerk * duration * duration;
  motion.acceleration += jerk * duration;
  return motion;
}

std::optional<double> runOutDecelerationDistance(
    double velocity, double acceleration, double target,
    double minimum_acceleration, double jerk_acc, double jerk_dec) {
  if (velocity < target) return std::nullopt;
  constexpr double epsilon = 1.0e-3;
  const double initial_jerk = minimum_acceleration < acceleration
      ? jerk_dec : jerk_acc;
  const double first_time =
      (minimum_acceleration - acceleration) / initial_jerk;
  const double last_time = -minimum_acceleration / jerk_acc;
  const double constant_time =
      (target - velocity - acceleration * first_time -
       0.5 * initial_jerk * first_time * first_time -
       minimum_acceleration * last_time -
       0.5 * jerk_acc * last_time * last_time) /
      minimum_acceleration;
  BrakingMotion motion{0.0, velocity, acceleration};
  if (constant_time > epsilon) {
    motion = advanceBraking(
        motion, initial_jerk, first_time > epsilon ? first_time : 0.0);
    motion = advanceBraking(motion, 0.0, constant_time);
    motion = advanceBraking(
        motion, jerk_acc, last_time > epsilon ? last_time : 0.0);
  } else if (0.5 * (-acceleration) / jerk_acc * acceleration >
                 target - velocity || acceleration > epsilon) {
    const double squared =
        (target - velocity - 0.5 * (-acceleration) / jerk_dec *
                                 acceleration) *
        (2.0 * jerk_acc * jerk_dec / (jerk_acc - jerk_dec));
    if (squared < 0.0) return std::nullopt;
    const double minimum = -std::sqrt(squared);
    const double before = (minimum - acceleration) / jerk_dec;
    motion = advanceBraking(
        motion, jerk_dec, before > epsilon ? before : 0.0);
    const double after = -minimum / jerk_acc;
    motion = advanceBraking(
        motion, jerk_acc, after > epsilon ? after : 0.0);
  } else {
    motion = advanceBraking(
        motion, jerk_acc, -acceleration / jerk_acc > epsilon
            ? -acceleration / jerk_acc : 0.0);
  }
  if (!std::isfinite(motion.distance) || motion.distance < 0.0 ||
      std::abs(motion.velocity - target) > 0.3 ||
      std::abs(motion.acceleration) > 0.1) {
    return std::nullopt;
  }
  return motion.distance;
}

}  // 익명 네임스페이스

RunOutStateCore::RunOutStateCore(RunOutStateParameters parameters)
    : parameters_(parameters) {
  if (!std::isfinite(parameters.stop_velocity_mps) ||
      !std::isfinite(parameters.stop_duration_s) ||
      !std::isfinite(parameters.disable_approach_distance_m) ||
      !std::isfinite(parameters.keep_approach_duration_s) ||
      parameters.stop_velocity_mps < 0.0 ||
      parameters.stop_duration_s < 0.0 ||
      parameters.disable_approach_distance_m < 0.0 ||
      parameters.keep_approach_duration_s < 0.0) {
    throw std::invalid_argument("돌발 진입 상태 설정이 잘못되었습니다");
  }
}

RunOutStateResult RunOutStateCore::update(const RunOutStateInput &input) {
  if (!std::isfinite(input.now_s) || !std::isfinite(input.velocity_mps) ||
      !std::isfinite(input.distance_to_collision_m)) {
    throw std::invalid_argument("돌발 진입 상태 입력에는 유한한 값이 필요합니다");
  }
  auto target = input.obstacle_id;
  switch (state_) {
    case RunOutState::Go:
      if (input.velocity_mps < parameters_.stop_velocity_mps) {
        stop_since_s_ = input.now_s;
        state_ = RunOutState::Stop;
      }
      break;
    case RunOutState::Stop:
      if (input.velocity_mps > parameters_.stop_velocity_mps ||
          !input.obstacle_id) {
        state_ = RunOutState::Go;
      } else if (input.now_s - stop_since_s_ > parameters_.stop_duration_s) {
        state_ = RunOutState::Approach;
        last_approach_obstacle_s_ = input.now_s;
        previous_obstacle_id_ = input.obstacle_id;
      }
      break;
    case RunOutState::Approach:
      if (input.distance_to_collision_m >
              parameters_.disable_approach_distance_m ||
          !input.obstacle_id) {
        if (input.now_s - last_approach_obstacle_s_ <
            parameters_.keep_approach_duration_s) {
          target = previous_obstacle_id_;
        } else {
          state_ = RunOutState::Go;
        }
      } else {
        last_approach_obstacle_s_ = input.now_s;
        previous_obstacle_id_ = input.obstacle_id;
      }
      break;
  }
  return {state_, target};
}

RunOutTemporalCore::RunOutTemporalCore(RunOutTemporalParameters parameters)
    : parameters_(parameters) {
  if (!std::isfinite(parameters_.detection_threshold_s) ||
      !std::isfinite(parameters_.keep_stop_point_s) ||
      parameters_.detection_threshold_s < 0.0 ||
      parameters_.keep_stop_point_s < 0.0) {
    throw std::invalid_argument("돌발 진입 시간 설정이 잘못되었습니다");
  }
}

bool RunOutTemporalCore::acceptCollision(double now_s, bool detected) {
  if (!std::isfinite(now_s)) {
    throw std::invalid_argument("돌발 진입 검출 시각에는 유한한 값이 필요합니다");
  }
  if (!detected) {
    first_detected_s_.reset();
    return false;
  }
  if (!parameters_.ignore_momentary_detection) return true;
  if (!first_detected_s_) {
    first_detected_s_ = now_s;
    return false;
  }
  return now_s - *first_detected_s_ >= parameters_.detection_threshold_s;
}

void RunOutTemporalCore::recordStop(
    double now_s, RunOutStopPoint stop_point) {
  if (!std::isfinite(now_s) || !std::isfinite(stop_point.x) ||
      !std::isfinite(stop_point.y)) {
    throw std::invalid_argument("돌발 진입 정지점에는 유한한 값이 필요합니다");
  }
  last_stop_s_ = now_s;
  last_stop_point_ = stop_point;
}

std::optional<RunOutStopPoint> RunOutTemporalCore::maintainedStop(
    double now_s, bool inserted_now) const {
  if (!std::isfinite(now_s)) {
    throw std::invalid_argument("돌발 진입 유지 시각에는 유한한 값이 필요합니다");
  }
  if (parameters_.approaching_enabled || inserted_now ||
      !last_stop_s_ || !last_stop_point_ ||
      now_s - *last_stop_s_ >= parameters_.keep_stop_point_s) {
    return std::nullopt;
  }
  return last_stop_point_;
}

RunOutApproachResult applyRunOutApproachVelocity(
    const Trajectory &input, std::size_t ego_segment_index,
    double collision_arc_m, const RunOutApproachParameters &parameters) {
  if (input.size() < 2 || ego_segment_index >= input.size() - 1 ||
      !std::isfinite(collision_arc_m) || collision_arc_m < 0.0 ||
      !std::isfinite(parameters.velocity_limit_mps) ||
      !std::isfinite(parameters.margin_m) ||
      !std::isfinite(parameters.front_offset_m) ||
      parameters.velocity_limit_mps < 0.0 || parameters.margin_m < 0.0 ||
      parameters.front_offset_m < 0.0) {
    throw std::invalid_argument("돌발 진입 접근 속도 입력이 잘못되었습니다");
  }

  RunOutApproachResult result;
  result.trajectory = input;
  for (std::size_t i = ego_segment_index; i < result.trajectory.size(); ++i) {
    result.trajectory[i].longitudinal_velocity_mps = std::min(
        result.trajectory[i].longitudinal_velocity_mps,
        static_cast<float>(parameters.velocity_limit_mps));
  }

  const double stop_arc = collision_arc_m - parameters.margin_m -
                          parameters.front_offset_m;
  if (stop_arc < 0.0) return result;
  const auto stopped = insertZeroVelocityAtArc(result.trajectory, stop_arc);
  result.trajectory = stopped.trajectory;
  result.stop_arc_m = stopped.stop_arc_m;
  return result;
}

TrajectoryStopResult applyRunOutStopVelocity(
    const Trajectory &input, double collision_arc_m,
    double stop_margin_m, double front_offset_m) {
  if (!std::isfinite(collision_arc_m) ||
      !std::isfinite(stop_margin_m) || !std::isfinite(front_offset_m) ||
      collision_arc_m < 0.0 || stop_margin_m < 0.0 ||
      front_offset_m < 0.0) {
    throw std::invalid_argument("돌발 진입 정지 경로 입력이 잘못되었습니다");
  }
  const double stop_arc = collision_arc_m - stop_margin_m - front_offset_m;
  if (stop_arc < 0.0) return {input, std::nullopt};
  return insertZeroVelocityAtArc(input, stop_arc);
}

RunOutBrakingResult planRunOutBraking(
    const Trajectory &trajectory, const RunOutBrakingInput &input,
    const RunOutBrakingParameters &parameters) {
  const double values[] = {
      input.collision_arc_m, input.ego_arc_m, input.velocity_mps,
      input.acceleration_mps2, parameters.stop_margin_m,
      parameters.front_offset_m, parameters.deceleration_jerk_mps3,
      parameters.normal_min_jerk_mps3,
      parameters.normal_min_acceleration_mps2,
      parameters.limit_min_acceleration_mps2};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("돌발 진입 제동 입력에는 유한한 값이 필요합니다");
    }
  }
  if (trajectory.empty() || input.collision_arc_m < 0.0 ||
      input.velocity_mps < 0.0 || parameters.stop_margin_m < 0.0 ||
      parameters.front_offset_m < 0.0 ||
      parameters.deceleration_jerk_mps3 >= 0.0 ||
      parameters.normal_min_jerk_mps3 >= 0.0 ||
      parameters.normal_min_acceleration_mps2 >= 0.0 ||
      parameters.limit_min_acceleration_mps2 >= 0.0) {
    throw std::invalid_argument("돌발 진입 제동 설정이 잘못되었습니다");
  }
  const double minimum_acceleration =
      parameters.deceleration_jerk_mps3 <
              parameters.normal_min_jerk_mps3
          ? parameters.limit_min_acceleration_mps2
          : parameters.normal_min_acceleration_mps2;
  const auto distance = runOutDecelerationDistance(
      input.velocity_mps, input.acceleration_mps2, 0.0,
      minimum_acceleration, -parameters.deceleration_jerk_mps3,
      parameters.deceleration_jerk_mps3);
  RunOutBrakingResult result;
  result.trajectory = trajectory;
  result.required_stop_distance_m = distance.value_or(0.0);
  result.distance_valid = distance.has_value();
  const double distance_to_collision = input.collision_arc_m -
      input.ego_arc_m - parameters.front_offset_m;
  result.deceleration_required = !distance ||
      result.required_stop_distance_m >
          distance_to_collision - parameters.stop_margin_m;
  if (!result.deceleration_required && input.object_detection) return result;
  if (!result.deceleration_required &&
      !(input.velocity_mps < 2.5 / 3.6 &&
        input.acceleration_mps2 < 0.01)) return result;
  const auto stopped = applyRunOutStopVelocity(
      trajectory, input.collision_arc_m, parameters.stop_margin_m,
      parameters.front_offset_m);
  result.trajectory = stopped.trajectory;
  result.stop_arc_m = stopped.stop_arc_m;
  return result;
}

}  // planning 네임스페이스
