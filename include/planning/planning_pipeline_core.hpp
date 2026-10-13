// SPDX-License-Identifier: Apache-2.0
// 같은 프레임의 행동 경로에서 최종 검증까지 독립 계산 단계를 연결합니다.

#pragma once

#include "planning/behavior_goal_refinement.hpp"
#include "planning/elastic_band_core.hpp"
#include "planning/motion_velocity_core.hpp"
#include "planning/obstacle_cruise_stop_core.hpp"
#include "planning/path_optimizer_core.hpp"
#include "planning/planning_validator_core.hpp"

#include <functional>
#include <optional>

namespace planning {

using BehaviorVelocityStage = std::function<Trajectory(
    const BehaviorGoalPath &, const TrajectoryPoint &, double)>;
using MotionVelocityStage = std::function<std::vector<MotionVelocityPlan>(
    const Trajectory &, const TrajectoryPoint &, double)>;
using CruiseStopCandidateStage =
    std::function<std::vector<ObstacleCruiseStopCandidate>(
        const Trajectory &, const TrajectoryPoint &, double, double)>;
using CruiseSlowdownStage = std::function<Trajectory(
    const Trajectory &, const Trajectory &, const TrajectoryPoint &,
    double, double)>;

struct PlanningPipelineFrame {
  double timestamp_s = 0.0;
  BehaviorGoalPath behavior_path;
  TrajectoryPoint ego;
  double ego_speed_mps = 0.0;
  std::vector<PathOptimizerPoint2D> left_bound;
  std::vector<PathOptimizerPoint2D> right_bound;
  bool goal_contained = false;
  std::vector<MotionVelocityPlan> motion_velocity_plans;
  std::vector<ObstacleCruiseStopCandidate> cruise_stop_candidates;
  // The obstacle cruise node snapshots these inputs at its own trajectory callback.
  struct CruiseCallback {
    double timestamp_s = 0.0;
    TrajectoryPoint ego;
    double ego_speed_mps = 0.0;
    std::optional<double> ego_acceleration_mps2;
  };
  std::optional<CruiseCallback> cruise_callback;
};

enum class PlanningPipelineStatus {
  Complete,
  InvalidInput,
  OptimizerFailed,
};

struct PlanningPipelineResult {
  PlanningPipelineStatus status = PlanningPipelineStatus::InvalidInput;
  Trajectory behavior_velocity;
  Trajectory elastic_band;
  Trajectory optimized;
  Trajectory motion_velocity;
  Trajectory cruise_stop_applied;
  Trajectory cruise_slowed;
  // Final obstacle-cruise output after stop, cruise-limit, and slow-down stages.
  Trajectory cruise_stopped;
  bool optimizer_state_reset = false;
  std::size_t cruise_candidate_count = 0;
  std::optional<double> cruise_stop_distance_from_start_m;
  std::optional<ValidationOutcome> validation;
};

class PlanningPipelineCore {
 public:
  PlanningPipelineCore(
      BehaviorVelocityStage behavior_velocity_stage,
      PathOptimizerCoreParameters optimizer_parameters = {},
      PrecheckThresholds validator_thresholds = {},
      InvalidTrajectoryPolicy invalid_policy =
          InvalidTrajectoryPolicy::UsePreviousResult,
      ObstacleCruiseStopParameters cruise_parameters = {},
      MotionVelocityStage motion_velocity_stage = {},
      CruiseStopCandidateStage cruise_candidate_stage = {},
      CruiseSlowdownStage cruise_slowdown_stage = {});

  // 각 계산기는 이전 콜백 상태를 유지하며 프레임 순서대로 실행됩니다.
  PlanningPipelineResult onFrame(const PlanningPipelineFrame &frame);
  void reset();

 private:
  BehaviorVelocityStage behavior_velocity_stage_;
  MotionVelocityStage motion_velocity_stage_;
  CruiseStopCandidateStage cruise_candidate_stage_;
  CruiseSlowdownStage cruise_slowdown_stage_;
  ElasticBandPlanner elastic_band_;
  PathOptimizerCore optimizer_;
  std::optional<PathOptimizerCoreInput> previous_optimizer_input_;
  ObstacleCruiseStopCore cruise_stop_;
  PlanningValidatorCore validator_;
  PrecheckThresholds validator_thresholds_;
  InvalidTrajectoryPolicy invalid_policy_;
};

}  // planning 네임스페이스
