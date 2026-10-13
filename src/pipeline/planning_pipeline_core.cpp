// SPDX-License-Identifier: Apache-2.0
// 행동 속도·평활화·경로 최적화·동작 속도·정지·감속·검증의 호출 순서입니다.

#include "planning/planning_pipeline_core.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>

namespace planning {
namespace {

double egoArcOnPath(const Trajectory &path, const TrajectoryPoint &ego) {
  double accumulated = 0.0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  double nearest_arc = 0.0;
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const auto &a = path[index];
    const auto &b = path[index + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared < 1.0e-12) continue;
    const double ratio = std::clamp(
        ((ego.x - a.x) * dx + (ego.y - a.y) * dy) /
            length_squared,
        0.0, 1.0);
    const double error = std::hypot(
        ego.x - a.x - ratio * dx,
        ego.y - a.y - ratio * dy);
    const double length = std::sqrt(length_squared);
    if (error < nearest_distance) {
      nearest_distance = error;
      nearest_arc = accumulated + ratio * length;
    }
    accumulated += length;
  }
  return nearest_arc;
}

}  // 익명 네임스페이스

PlanningPipelineCore::PlanningPipelineCore(
    BehaviorVelocityStage behavior_velocity_stage,
    PathOptimizerCoreParameters optimizer_parameters,
    PrecheckThresholds validator_thresholds,
    InvalidTrajectoryPolicy invalid_policy,
    ObstacleCruiseStopParameters cruise_parameters,
    MotionVelocityStage motion_velocity_stage,
    CruiseStopCandidateStage cruise_candidate_stage,
    CruiseSlowdownStage cruise_slowdown_stage)
    : behavior_velocity_stage_(std::move(behavior_velocity_stage)),
      motion_velocity_stage_(std::move(motion_velocity_stage)),
      cruise_candidate_stage_(std::move(cruise_candidate_stage)),
      cruise_slowdown_stage_(std::move(cruise_slowdown_stage)),
      optimizer_(optimizer_parameters),
      cruise_stop_(cruise_parameters),
      validator_(validator_thresholds, invalid_policy),
      validator_thresholds_(validator_thresholds),
      invalid_policy_(invalid_policy) {
  if (!behavior_velocity_stage_) {
    throw std::invalid_argument("행동 속도 계획 단계가 필요합니다");
  }
}

PlanningPipelineResult PlanningPipelineCore::onFrame(
    const PlanningPipelineFrame &frame) {
  PlanningPipelineResult result;
  if (!std::isfinite(frame.timestamp_s) ||
      !std::isfinite(frame.ego.x) || !std::isfinite(frame.ego.y) ||
      !std::isfinite(frame.ego_speed_mps) ||
      frame.behavior_path.points.size() < 2 ||
      (!frame.behavior_path.lane_ids.empty() &&
       frame.behavior_path.lane_ids.size() !=
           frame.behavior_path.points.size()) ||
      frame.left_bound.size() < 2 || frame.right_bound.size() < 2) {
    return result;
  }
  result.behavior_velocity = behavior_velocity_stage_(
      frame.behavior_path, frame.ego, frame.timestamp_s);
  if (result.behavior_velocity.size() < 2) return result;
  result.elastic_band = elastic_band_.run(
      result.behavior_velocity, frame.ego,
      frame.ego_speed_mps, frame.timestamp_s).trajectory;
  PathOptimizerCoreInput optimizer_input;
  optimizer_input.path = result.elastic_band;
  optimizer_input.left_bound = frame.left_bound;
  optimizer_input.right_bound = frame.right_bound;
  optimizer_input.ego = frame.ego;
  optimizer_input.ego_speed = frame.ego_speed_mps;
  optimizer_input.goal_contained = frame.goal_contained;
  result.optimizer_state_reset = requiresPathOptimizerReset(
      optimizer_input, previous_optimizer_input_);
  if (result.optimizer_state_reset) optimizer_.reset();
  previous_optimizer_input_ = optimizer_input;
  const auto optimized = optimizer_.optimize(optimizer_input);
  if (!optimized) {
    result.status = PlanningPipelineStatus::OptimizerFailed;
    return result;
  }
  result.optimized = optimized->output;
  auto motion_plans = frame.motion_velocity_plans;
  if (motion_velocity_stage_) {
    auto generated = motion_velocity_stage_(
        result.optimized, frame.ego, frame.timestamp_s);
    motion_plans.insert(motion_plans.end(),
                        std::make_move_iterator(generated.begin()),
                        std::make_move_iterator(generated.end()));
  }
  result.motion_velocity = applyMotionVelocityPlans(
      result.optimized, motion_plans);
  const auto cruise_ego = frame.cruise_callback ?
      frame.cruise_callback->ego : frame.ego;
  const double cruise_speed = frame.cruise_callback ?
      frame.cruise_callback->ego_speed_mps : frame.ego_speed_mps;
  const double cruise_time = frame.cruise_callback ?
      frame.cruise_callback->timestamp_s : frame.timestamp_s;
  auto cruise_candidates = frame.cruise_stop_candidates;
  if (cruise_candidate_stage_) {
    auto generated = cruise_candidate_stage_(
        result.motion_velocity, cruise_ego, cruise_speed, cruise_time);
    cruise_candidates.insert(cruise_candidates.end(),
                             std::make_move_iterator(generated.begin()),
                             std::make_move_iterator(generated.end()));
  }
  result.cruise_candidate_count = cruise_candidates.size();
  // ObstacleCruisePlannerNode::onTrajectory calls generateStopTrajectory,
  // generateCruiseTrajectory, then generateSlowDownTrajectory in this order.
  const auto stop_result = cruise_stop_.plan(
      result.motion_velocity,
      egoArcOnPath(result.motion_velocity, cruise_ego),
      cruise_speed, cruise_candidates);
  result.cruise_stop_applied = stop_result.trajectory;
  result.cruise_stop_distance_from_start_m =
      stop_result.stop_distance_from_start;
  result.cruise_slowed = cruise_slowdown_stage_ ?
      cruise_slowdown_stage_(result.motion_velocity,
                             result.cruise_stop_applied, cruise_ego,
                             cruise_speed, cruise_time) :
      result.cruise_stop_applied;
  if (result.cruise_slowed.size() < 2) return result;
  result.cruise_stopped = result.cruise_slowed;
  result.validation = validator_.onTrajectory(
      result.cruise_stopped,
      EgoState{frame.ego.x, frame.ego.y, frame.ego_speed_mps});
  result.status = PlanningPipelineStatus::Complete;
  return result;
}

void PlanningPipelineCore::reset() {
  elastic_band_.reset();
  optimizer_.reset();
  previous_optimizer_input_.reset();
  cruise_stop_.reset();
  validator_ = PlanningValidatorCore(
      validator_thresholds_, invalid_policy_);
}

}  // planning 네임스페이스
