// SPDX-License-Identifier: Apache-2.0
#include "planning/planning_final_pipeline_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace planning {

void applySelectedVelocityLimit(
    PlanningFinalFrame &frame, const VelocityLimit &limit) {
  frame.maximum_velocity_mps = limit.max_velocity;
  frame.external_limit_active = true;
  frame.external_limit_use_constraints = limit.use_constraints;
  frame.external_min_acceleration_mps2 =
      limit.constraints.min_acceleration;
  frame.external_max_jerk_mps3 = limit.constraints.max_jerk;
  frame.external_min_jerk_mps3 = limit.constraints.min_jerk;
}

namespace {

std::size_t nearestPoint(const Trajectory &trajectory,
                         const TrajectoryPoint &ego) {
  std::size_t closest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < trajectory.size(); ++index) {
    const auto &point = trajectory[index];
    const double distance = std::hypot(point.x - ego.x, point.y - ego.y);
    if (distance < best) {
      best = distance;
      closest = index;
    }
  }
  return closest;
}

double distanceToStop(const Trajectory &trajectory, std::size_t closest) {
  double distance = 0.0;
  for (std::size_t index = closest; index < trajectory.size(); ++index) {
    if (std::abs(trajectory[index].longitudinal_velocity_mps) < 1.0e-3)
      return distance;
    if (index + 1 < trajectory.size())
      distance += std::hypot(trajectory[index + 1].x - trajectory[index].x,
                             trajectory[index + 1].y - trajectory[index].y);
  }
  return distance;
}

// VelocitySmootherNode::calcTrajectoryVelocity extracts the local path before
// inserting the external velocity limit and applying the stop approach rule.
Trajectory extractSmootherPath(const Trajectory &input, std::size_t closest,
                              double ahead_length_m, double behind_length_m) {
  std::size_t ahead = input.size() - 1;
  double distance = 0.0;
  for (std::size_t index = closest; index + 1 < input.size(); ++index) {
    distance += std::hypot(input[index + 1].x - input[index].x,
                           input[index + 1].y - input[index].y);
    if (distance > ahead_length_m) {
      ahead = index + 1;
      break;
    }
  }
  std::size_t behind = 0;
  distance = 0.0;
  for (std::size_t index = closest; index > 0; --index) {
    distance += std::hypot(input[index].x - input[index - 1].x,
                           input[index].y - input[index - 1].y);
    if (distance > behind_length_m) {
      behind = index - 1;
      break;
    }
  }
  return Trajectory(input.begin() + behind, input.begin() + ahead + 1);
}

struct ProjectedPathPoint {
  double arc = 0.0;
  double x = 0.0;
  double y = 0.0;
};

ProjectedPathPoint projectOnPath(const Trajectory &path,
                                const TrajectoryPoint &pose) {
  ProjectedPathPoint selected;
  double best = std::numeric_limits<double>::infinity();
  double arc = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double dx = path[i].x - path[i - 1].x;
    const double dy = path[i].y - path[i - 1].y;
    const double length2 = dx * dx + dy * dy;
    if (length2 < 1.0e-12) continue;
    const double ratio = std::clamp(
        ((pose.x - path[i - 1].x) * dx + (pose.y - path[i - 1].y) * dy) /
            length2, 0.0, 1.0);
    const double x = path[i - 1].x + ratio * dx;
    const double y = path[i - 1].y + ratio * dy;
    const double error = std::hypot(pose.x - x, pose.y - y);
    if (error < best) {
      best = error;
      selected = {arc + ratio * std::sqrt(length2), x, y};
    }
    arc += std::sqrt(length2);
  }
  return selected;
}

// VelocitySmootherNode uses trajectory_utils::calcStopDistWithJerkConstraints
// when an external limit changes. Integrate the same trapezoid, triangle, or
// linear jerk profile to locate the first point subject to the new limit.
std::optional<double> externalLimitDistance(
    double velocity, double acceleration, double target, double jerk_up,
    double jerk_down, double minimum_acceleration) {
  if (!(jerk_up > 0.0 && jerk_down < 0.0 && minimum_acceleration < 0.0))
    return std::nullopt;
  const auto time_to = [](double a, double target_a, double jerk) {
    return (target_a - a) / jerk;
  };
  const double decel_change =
      0.5 * time_to(acceleration, 0.0, jerk_down) * acceleration +
      0.5 * time_to(0.0, minimum_acceleration, jerk_down) *
          minimum_acceleration;
  const double recover_change =
      0.5 * time_to(minimum_acceleration, 0.0, jerk_up) *
          minimum_acceleration;
  const double constant_time =
      (target - velocity - decel_change - recover_change) /
      minimum_acceleration;
  struct Segment { double jerk; double duration; };
  std::vector<Segment> profile;
  if (constant_time > 0.0) {
    profile = {{jerk_down, time_to(acceleration, minimum_acceleration,
                                   jerk_down)},
               {0.0, constant_time},
               {jerk_up, time_to(minimum_acceleration, 0.0, jerk_up)}};
  } else {
    const double velocity_error = target - velocity;
    const double decel_to_zero =
        0.5 * acceleration * time_to(acceleration, 0.0, jerk_up);
    if (decel_to_zero > velocity_error || acceleration > 0.0) {
      const double square =
          (velocity_error - 0.5 * acceleration *
              time_to(acceleration, 0.0, jerk_down)) * 2.0 /
          (1.0 / jerk_down - 1.0 / jerk_up);
      if (square < 0.0) return std::nullopt;
      const double peak = -std::sqrt(square);
      profile = {{jerk_down, time_to(acceleration, peak, jerk_down)},
                 {jerk_up, time_to(peak, 0.0, jerk_up)}};
    } else {
      if (std::abs(velocity_error) < 1.0e-9) return 0.0;
      const double jerk =
          -acceleration * acceleration / (2.0 * velocity_error);
      if (std::abs(jerk) < 1.0e-9) return std::nullopt;
      profile = {{jerk, time_to(acceleration, 0.0, jerk)}};
    }
  }
  double distance = 0.0;
  for (const auto segment : profile) {
    const double duration = segment.duration < 1.0e-6 ? 0.0 :
                            segment.duration;
    if (!std::isfinite(duration) || duration < 0.0) return std::nullopt;
    distance += velocity * duration +
                0.5 * acceleration * duration * duration +
                segment.jerk * duration * duration * duration / 6.0;
    velocity += acceleration * duration +
                0.5 * segment.jerk * duration * duration;
    acceleration += segment.jerk * duration;
  }
  if (!std::isfinite(distance) || distance < 0.0 ||
      std::abs(velocity - target) > 0.3 || std::abs(acceleration) > 0.1)
    return std::nullopt;
  return distance;
}

Trajectory limitExternalVelocity(const Trajectory &input,
                                 const TrajectoryPoint &ego,
                                 double deceleration_velocity,
                                 double external_velocity,
                                 double distance_to_limit,
                                 double nearest_distance,
                                 double nearest_yaw) {
  Trajectory output = input;
  for (auto &point : output) point.longitudinal_velocity_mps = std::min(
      point.longitudinal_velocity_mps,
      static_cast<float>(deceleration_velocity));
  if (output.size() < 2) {
    if (!output.empty()) output.back().longitudinal_velocity_mps = std::min(
        output.back().longitudinal_velocity_mps,
        static_cast<float>(external_velocity));
    return output;
  }
  // motion_utils::findFirstNearestSegmentIndexWithSoftConstraints first
  // selects a point, then the adjacent segment. insertTargetPoint measures
  // its distance from that segment's front point, not from the ego projection.
  const auto yaw_of = [](const TrajectoryPoint &point) {
    return std::atan2(
        2.0 * (point.orientation_w * point.orientation_z +
               point.orientation_x * point.orientation_y),
        1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                     point.orientation_z * point.orientation_z));
  };
  constexpr double full_turn = 6.28318530717958647692;
  const double path_yaw = std::atan2(
      output[1].y - output[0].y, output[1].x - output[0].x);
  if (std::abs(std::remainder(yaw_of(output[0]) - path_yaw, full_turn)) >=
      full_turn / 4.0) {
    // insertTargetPoint does not insert into a reverse trajectory.
    output.back().longitudinal_velocity_mps = std::min(
        output.back().longitudinal_velocity_mps,
        static_cast<float>(external_velocity));
    return output;
  }
  const auto nearest_with = [&](bool check_distance, bool check_yaw)
      -> std::optional<std::size_t> {
    double best = std::numeric_limits<double>::max();
    std::optional<std::size_t> nearest;
    for (std::size_t i = 0; i < output.size(); ++i) {
      const double dx = output[i].x - ego.x;
      const double dy = output[i].y - ego.y;
      const double squared = dx * dx + dy * dy;
      const bool rejected =
          (check_distance && squared > nearest_distance * nearest_distance) ||
          (check_yaw && std::abs(std::remainder(
              yaw_of(output[i]) - yaw_of(ego), full_turn)) > nearest_yaw);
      if (rejected) {
        if (nearest) break;
        continue;
      }
      if (squared < best) {
        best = squared;
        nearest = i;
      }
    }
    return nearest;
  };
  auto nearest = nearest_with(true, true);
  if (!nearest) nearest = nearest_with(true, false);
  if (!nearest) nearest = nearest_with(false, false);
  const std::size_t nearest_index = nearest.value_or(0);
  std::size_t segment = nearest_index;
  if (nearest_index == output.size() - 1) {
    segment = nearest_index - 1;
  } else if (nearest_index != 0) {
    const double dx = output[nearest_index + 1].x - output[nearest_index].x;
    const double dy = output[nearest_index + 1].y - output[nearest_index].y;
    if (dx * (ego.x - output[nearest_index].x) +
        dy * (ego.y - output[nearest_index].y) <= 0.0)
      segment = nearest_index - 1;
  }
  double remaining = std::max(0.0, distance_to_limit);
  for (std::size_t i = segment + 1; i < output.size(); ++i) {
    const double length = std::hypot(output[i].x - output[i - 1].x,
                                     output[i].y - output[i - 1].y);
    if (length >= remaining) {
      std::size_t first = i;
      if (remaining > 1.0e-3 &&
          length - remaining > 1.0e-3 && length > 1.0e-9) {
        const double ratio = remaining / length;
        auto inserted = output[i - 1];
        inserted.x += ratio * (output[i].x - inserted.x);
        inserted.y += ratio * (output[i].y - inserted.y);
        inserted.z += ratio * (output[i].z - inserted.z);
        const double yaw = std::atan2(
            output[i].y - inserted.y, output[i].x - inserted.x);
        const double pitch = std::atan2(
            output[i].z - inserted.z,
            std::hypot(output[i].x - inserted.x, output[i].y - inserted.y));
        const double sy = std::sin(yaw / 2.0);
        const double cy = std::cos(yaw / 2.0);
        const double sp = std::sin(pitch / 2.0);
        const double cp = std::cos(pitch / 2.0);
        inserted.orientation_x = -sp * sy;
        inserted.orientation_y = sp * cy;
        inserted.orientation_z = cp * sy;
        inserted.orientation_w = cp * cy;
        output[i - 1].orientation_x = inserted.orientation_x;
        output[i - 1].orientation_y = inserted.orientation_y;
        output[i - 1].orientation_z = inserted.orientation_z;
        output[i - 1].orientation_w = inserted.orientation_w;
        output.insert(output.begin() + i, inserted);
      } else if (remaining <= 1.0e-3) {
        first = i - 1;
      }
      for (std::size_t j = first; j < output.size(); ++j)
        output[j].longitudinal_velocity_mps = std::min(
            output[j].longitudinal_velocity_mps,
            static_cast<float>(external_velocity));
      return output;
    }
    remaining -= length;
  }
  output.back().longitudinal_velocity_mps = std::min(
      output.back().longitudinal_velocity_mps,
      static_cast<float>(external_velocity));
  return output;
}

}  // namespace

PlanningFinalPipelineCore::PlanningFinalPipelineCore(
    ScenarioSelectorParameters scenario_parameters,
    JerkSmootherParameters smoother_parameters,
    JerkNodeOutputParameters node_parameters,
    JerkPathLimitParameters path_limit_parameters,
    PrecheckThresholds validator_thresholds,
    InvalidTrajectoryPolicy invalid_policy)
    : scenario_parameters_(scenario_parameters),
      smoother_parameters_(smoother_parameters),
      node_parameters_(node_parameters),
      path_limit_parameters_(path_limit_parameters),
      validator_thresholds_(validator_thresholds),
      invalid_policy_(invalid_policy),
      scenario_(scenario_parameters),
      validator_(validator_thresholds, invalid_policy) {
  scenario_.onMap(true);
  node_parameters_.post_resampling.max_trajectory_length = 300.0;
  node_parameters_.post_resampling.min_trajectory_length = 30.0;
  node_parameters_.post_resampling.resample_time = 10.0;
  node_parameters_.post_resampling.dense_resample_dt = 0.1;
  node_parameters_.post_resampling.dense_min_interval_distance = 0.1;
  node_parameters_.post_resampling.sparse_resample_dt = 0.1;
  node_parameters_.post_resampling.sparse_min_interval_distance = 1.0;
}

PlanningFinalResult PlanningFinalPipelineCore::onFrame(
    const PlanningFinalFrame &frame) {
  PlanningFinalResult result;
  if (frame.route_id.empty() || frame.lane_driving.size() < 2 ||
      !std::isfinite(frame.now_s) ||
      !std::isfinite(frame.odometry_stamp_s) ||
      !std::isfinite(frame.maximum_velocity_mps) ||
      frame.maximum_velocity_mps < 0.0) return result;
  if (route_id_ != frame.route_id) {
    route_id_ = frame.route_id;
    scenario_.onRoute(route_id_);
    // Only scenario_selector consumes the route UUID. The source velocity
    // smoother and validator are separate nodes: receiving a new route does
    // not clear their previous trajectory or external velocity limit state.
  }
  scenario_.onOdometry(frame.odometry_stamp_s, frame.ego.x, frame.ego.y,
                       frame.ego_speed_mps);
  scenario_.onOperationMode(frame.autonomous, frame.control_enabled);
  scenario_.onParkingCompleted(frame.parking_completed);
  scenario_.onLaneTrajectory({frame.lane_driving, frame.now_s}, frame.now_s);
  if (frame.parking)
    scenario_.onParkingTrajectory({*frame.parking, frame.now_s}, frame.now_s);
  result.scenario = scenario_.tick(frame.now_s, frame.position);
  if (!result.scenario) return result;
  const bool parking = result.scenario->current == Scenario::Parking;
  const auto selected = parking && frame.parking ?
      scenario_.onParkingTrajectory({*frame.parking, frame.now_s}, frame.now_s) :
      scenario_.onLaneTrajectory({frame.lane_driving, frame.now_s}, frame.now_s);
  if (!selected || selected->points.size() < 2) return result;
  result.selected = selected->points;

  const double node_maximum_velocity =
      frame.node_maximum_velocity_mps.value_or(frame.maximum_velocity_mps);
  if (!std::isfinite(node_maximum_velocity) || node_maximum_velocity < 0.0)
    return result;
  if (!external_velocity_mps_) {
    external_velocity_mps_ = node_maximum_velocity;
    maximum_velocity_with_deceleration_mps_ = node_maximum_velocity;
  }
  std::optional<JerkMotion> previous_motion;
  double travel_distance = 0.0;
  if (previous_smoother_internal_ &&
      !previous_smoother_internal_->empty()) {
    const auto projected = projectOnPath(*previous_smoother_internal_,
                                         frame.ego);
    if (previous_projected_ego_)
      travel_distance = std::hypot(
              projected.x - previous_projected_ego_->first,
              projected.y - previous_projected_ego_->second);
    previous_motion = projectJerkPreviousMotion(
        *previous_smoother_internal_, frame.ego,
        node_parameters_.nearest_distance, node_parameters_.nearest_yaw);
  }
  if (frame.external_limit_active) {
    if (previous_motion && std::abs(frame.ego_speed_mps) < 1.0e-4 &&
        *external_velocity_mps_ < 1.0e-4)
      external_limit_distance_m_ = 0.0;
    if (!previous_motion) {
      *external_velocity_mps_ = frame.maximum_velocity_mps;
      maximum_velocity_with_deceleration_mps_ = frame.maximum_velocity_mps;
      external_limit_distance_m_ = 0.0;
    } else if (std::abs(*external_velocity_mps_ -
                        frame.maximum_velocity_mps) > 1.0e-4) {
      const double v0 = previous_motion->velocity;
      const double a0 = previous_motion->acceleration;
      const JerkInitialMotionParameters initial_parameters;
      const double jerk_down = frame.external_limit_use_constraints ?
          frame.external_min_jerk_mps3 : smoother_parameters_.qp.min_jerk;
      const double jerk_up = frame.external_limit_use_constraints ?
          frame.external_max_jerk_mps3 : smoother_parameters_.qp.max_jerk;
      const double minimum_acceleration =
          frame.external_limit_use_constraints ?
          frame.external_min_acceleration_mps2 :
          smoother_parameters_.qp.min_deceleration;
      // VelocitySmootherNode::calcExternalVelocityLimit applies a new limit
      // directly while the vehicle is in its engage-speed range.
      const bool engaging =
          std::abs(frame.ego_speed_mps) <
              initial_parameters.engage_velocity *
                  initial_parameters.engage_exit_ratio &&
          v0 >= initial_parameters.engage_velocity;
      maximum_velocity_with_deceleration_mps_ = engaging ?
          frame.maximum_velocity_mps :
          (a0 > 0.0 ? v0 - 0.5 * a0 * a0 / jerk_down : v0);
      if (engaging) {
        external_limit_distance_m_ = 0.0;
      } else if (frame.maximum_velocity_mps <
          maximum_velocity_with_deceleration_mps_) {
        external_limit_distance_m_ = externalLimitDistance(
            v0, a0, frame.maximum_velocity_mps, jerk_up,
            jerk_down, minimum_acceleration).value_or(0.0) + 0.3;
      } else {
        maximum_velocity_with_deceleration_mps_ =
            frame.maximum_velocity_mps;
        external_limit_distance_m_ = 0.0;
      }
      *external_velocity_mps_ = frame.maximum_velocity_mps;
    }
  }
  external_limit_distance_m_ = std::max(
      0.0, external_limit_distance_m_ - travel_distance);
  auto raw_input = *result.selected;
  // VelocitySmootherNode::onCurrentTrajectory always zeroes its raw terminal
  // point before applying the external limit and extracting the local path.
  raw_input.back().longitudinal_velocity_mps = 0.0F;
  raw_input = extractSmootherPath(
      raw_input, nearestPoint(raw_input, frame.ego),
      node_parameters_.extract_ahead_distance_m,
      node_parameters_.extract_behind_distance_m);
  // The source node calls applyExternalVelocityLimit on every trajectory,
  // including when the current limit equals its configured maximum velocity.
  auto input = limitExternalVelocity(
      raw_input, frame.ego,
      maximum_velocity_with_deceleration_mps_,
      *external_velocity_mps_, external_limit_distance_m_,
      node_parameters_.nearest_distance, node_parameters_.nearest_yaw);
  limitVelocityApproachingStop(input, 2.778, 0.0);
  const bool reverse = isReverseJerkTrajectory(input);
  if (reverse) flipJerkTrajectoryVelocity(input);
  const std::size_t input_closest = nearestPoint(input, frame.ego);
  const auto initial = selectJerkInitialMotion(
      frame.ego_speed_mps, frame.ego_acceleration_mps2,
      input[input_closest].longitudinal_velocity_mps,
      distanceToStop(input, input_closest),
      frame.autonomous && frame.control_enabled, previous_motion, {});
  auto lateral = applyJerkLateralAccelerationLimit(
      input, initial.motion.velocity, initial.motion.acceleration,
      path_limit_parameters_);
  auto steering = applyJerkSteeringRateLimit(lateral,
                                             path_limit_parameters_);
  auto resampled = resampleJerkNodeInput(
      steering, frame.ego, node_parameters_.nearest_distance,
      node_parameters_.nearest_yaw, smoother_parameters_.resampling, 0.1);
  if (resampled.empty()) return result;
  resampled.back().longitudinal_velocity_mps = 0.0F;
  const std::size_t closest = nearestPoint(resampled, frame.ego);
  const Trajectory clipped(resampled.begin() + closest, resampled.end());
  auto solved = runJerkSmoother(
      clipped, initial.motion.velocity, initial.motion.acceleration,
      smoother_parameters_);
  if (!solved.output) return result;
  result.smoother_solved = solved.qp_result.solved;
  auto parameters = node_parameters_;
  parameters.max_velocity = node_maximum_velocity;
  parameters.keep_closest_motion_behind =
      initial.type != JerkInitialType::PreviousOutput;
  parameters.preserve_velocity_restarts = true;
  Trajectory internal;
  auto output = previous_smoother_internal_ ?
      finalizeNextJerkNodeTrajectory(
          resampled, closest, *solved.output, *previous_smoother_internal_,
          frame.ego, frame.ego_speed_mps, parameters, &internal) :
      finalizeFirstJerkNodeTrajectory(
          resampled, closest, *solved.output, frame.ego,
          frame.ego_speed_mps, parameters, &internal);
  previous_smoother_internal_ = std::move(internal);
  const auto projected = projectOnPath(*previous_smoother_internal_,
                                       frame.ego);
  previous_projected_ego_ = std::make_pair(projected.x, projected.y);
  if (reverse) flipJerkTrajectoryVelocity(output);
  result.smoothed = std::move(output);
  result.validation = validator_.onTrajectory(
      *result.smoothed,
      EgoState{frame.ego.x, frame.ego.y, frame.ego_speed_mps});
  return result;
}

void PlanningFinalPipelineCore::reset() {
  scenario_ = ScenarioSelectorCore(scenario_parameters_);
  scenario_.onMap(true);
  validator_ = PlanningValidatorCore(validator_thresholds_, invalid_policy_);
  previous_smoother_internal_.reset();
  previous_projected_ego_.reset();
  external_velocity_mps_.reset();
  external_limit_distance_m_ = 0.0;
  maximum_velocity_with_deceleration_mps_ = 0.0;
  route_id_.clear();
}

}  // namespace planning
