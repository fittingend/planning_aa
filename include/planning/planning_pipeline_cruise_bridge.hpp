// SPDX-License-Identifier: Apache-2.0
// 같은 인식 묶음의 객체를 종단 경로의 순항 정지 후보로 변환합니다.

#pragma once

#include "planning/behavior_velocity_perception_bridge.hpp"
#include "planning/obstacle_cruise_candidate.hpp"
#include "planning/obstacle_cruise_stop_core.hpp"
#include "planning/obstacle_cruise_pid_core.hpp"

#include <unordered_map>
#include <unordered_set>

namespace planning {

struct PlanningCruisePerceptionFrame {
  std::vector<PlanningPerceivedObject> objects;
  double stamp_s = 0.0;
  bool slowdown_enabled = false;
};

struct CruiseVehicleGeometry {
  // 기본 planning_simulator의 sample_vehicle 차량 외형.
  double front_m = 3.79;
  double rear_m = 1.10;
  double width_m = 1.896;
};

struct CruiseCandidateDiagnostic {
  std::string id;
  double lateral_distance_m = 0.0;
  double collision_overlap_s = 0.0;
  double first_collision_arc_m = 0.0;
  double projected_velocity_mps = 0.0;
  double approach_velocity_mps = 0.0;
  std::size_t prediction_rank = 0;
};

// 원본의 slow_down_condition_counter_와 직전 활성 객체 상태를 보존합니다.
class CruiseSlowdownCandidateState {
 public:
  void beginFrame();
  bool shouldSlowDown(const std::string &id, double precise_lateral_distance_m);
  void accepted(const std::string &id);
  void endFrame();
  void reset();

 private:
  std::unordered_map<std::string, int> counters_;
  std::unordered_set<std::string> previous_active_;
  std::unordered_set<std::string> current_active_;
  std::unordered_set<std::string> current_seen_;
};

class PlanningCruiseStopCandidateCore {
 public:
  explicit PlanningCruiseStopCandidateCore(
      CruiseCandidateParameters cruise_parameters = {},
      CruiseVehicleGeometry vehicle = {});
  std::vector<ObstacleCruiseStopCandidate> plan(
      const Trajectory &trajectory, const TrajectoryPoint &ego,
      double ego_speed_mps, double now_s,
      const PlanningCruisePerceptionFrame &perception);
  const std::vector<std::string> &claimedObjectIds() const;
  const std::vector<CruisePidObstacle> &pidObstacles() const;
  const std::vector<CruiseCandidateDiagnostic> &cruiseDiagnostics() const;
  void reset();

 private:
  CruiseCandidateParameters cruise_parameters_;
  CruiseVehicleGeometry vehicle_;
  CruiseStopConsistency consistency_;
  CruiseInsideHysteresis cruise_history_;
  std::vector<std::string> claimed_object_ids_;
  std::vector<CruisePidObstacle> pid_obstacles_;
  std::vector<CruiseCandidateDiagnostic> cruise_diagnostics_;
};

}  // planning 네임스페이스
