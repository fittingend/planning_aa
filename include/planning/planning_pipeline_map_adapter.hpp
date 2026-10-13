// SPDX-License-Identifier: Apache-2.0
// 이진 지도 행동 속도 플러그인을 독립 계획 파이프라인에 연결합니다.

#pragma once

#include "planning/behavior_velocity_map_planner.hpp"
#include "planning/planning_pipeline_core.hpp"
#include "planning/planning_pipeline_cruise_bridge.hpp"
#include "planning/obstacle_cruise_pointcloud.hpp"
#include "planning/obstacle_cruise_slowdown_core.hpp"
#include "planning/obstacle_velocity_limiter_core.hpp"
#include "planning/behavior_candidate_callback.hpp"
#include "planning/behavior_route_action.hpp"
#include "planning/costmap_generator_core.hpp"
#include "planning/behavior_dynamic_drivable_area.hpp"
#include "planning/obstacle_cruise_pid_core.hpp"
#include "planning/velocity_limit_selector.hpp"

#include <memory>
#include <limits>
#include <string>
#include <string_view>
#include <unordered_map>

namespace planning {

struct PlanningRouteControlState {
  bool controlled_by_autoware_autonomously = true;
  bool has_possible_reroute_approved_module = false;
};

struct PlanningPipelineMapResult {
  PlanningPipelineResult pipeline;
  std::optional<BehaviorVelocityMapResult> behavior_velocity;
  std::optional<BehaviorCandidateCallbackResult> behavior_candidates;
  std::optional<BehaviorRouteActionResult> behavior_action;
  std::size_t avoidance_perception_input_count = 0;
  // Continuously observed objects before the final avoidance decision.
  std::vector<BehaviorDynamicAvoidanceObservation>
      dynamic_avoidance_targets;
  // Objects accepted by the final avoidance decision, before polygon creation.
  std::vector<BehaviorDynamicAvoidanceObservation>
      dynamic_avoidance_final_targets;
  // Objects whose exclusion polygons were actually produced.
  std::vector<BehaviorDynamicAvoidanceObservation>
      dynamic_avoidance_selected_targets;
  std::vector<BehaviorSafetyPolygon>
      dynamic_avoidance_candidate_polygons;
  BehaviorDynamicDrivableArea dynamic_drivable_area;
  std::size_t cruise_slowdown_candidate_count = 0;
  std::optional<CrosswalkOccupancyGrid> generated_parking_grid;
  BehaviorGoalPath route_behavior_path;
  // Route centerline after input resampling/cropping, before goal refinement.
  BehaviorGoalPath route_reference_path;
  // Route path after fixed-goal refinement, before output resampling.
  BehaviorGoalPath route_goal_refined_path;
  // A new selector event is emitted while cruise is active and once on clear.
  std::optional<VelocityLimit> selected_velocity_limit;
  std::size_t cruise_pid_obstacle_count = 0;
  std::optional<CruisePidVelocityLimit> cruise_pid_limit;
  std::vector<CruiseCandidateDiagnostic> cruise_diagnostics;
  std::vector<MotionVelocitySlowdown> cruise_slowdown_intervals;
};

struct PlanningPipelinePointcloudInput {
  std::vector<MotionVelocityPoint2D> velocity_limiter_points;
  std::vector<double> velocity_limiter_time_from_start_s;
  VelocityLimiterObstacleMasks velocity_limiter_masks;
  ObstacleVelocityLimiterParameters velocity_limiter_parameters;
  bool velocity_limiter_enabled = false;
  std::vector<CruisePoint3D> cruise_points;
  CruisePointcloudParameters cruise_parameters;
  bool cruise_stop_enabled = false;
  bool cruise_slowdown_enabled = false;
};

// MotionVelocityPlannerNode's out-of-lane plugin consumes its own callback
// snapshot. Supply either its minimum braking arc or the odometry,
// acceleration and smoother limits needed to calculate that arc. An
// empty arrival-time array uses the source node's 0.5 m resampling and
// calculate_time_from_start policy on the independently calculated path.
struct PlanningOutOfLaneBrakingInput {
  double ego_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double ego_acceleration_mps2 = std::numeric_limits<double>::quiet_NaN();
  double minimum_acceleration_mps2 = std::numeric_limits<double>::quiet_NaN();
  double minimum_jerk_mps3 = std::numeric_limits<double>::quiet_NaN();
};

struct PlanningOutOfLaneInput {
  std::vector<double> time_from_start_s;
  std::vector<OutOfLaneTrackedObject> objects;
  std::vector<std::int64_t> red_traffic_light_ids;
  std::optional<double> minimum_stop_arc_m;
  // When an arc is not supplied, calculate it from the callback's odometry,
  // acceleration and smoother limits using the original PlannerData formula.
  std::optional<PlanningOutOfLaneBrakingInput> braking;
  double stop_line_search_distance_m = 100.0;
};

enum class MotionVelocityModule {
  OutOfLane,
  ObstacleVelocityLimiter,
  DynamicObstacleStop,
};

class PlanningPipelineMapAdapter {
 public:
  PlanningPipelineMapAdapter(
      std::vector<std::uint8_t> map_data,
      std::vector<BehaviorVelocityModule> modules,
      PathOptimizerCoreParameters optimizer_parameters = {},
      PrecheckThresholds validator_thresholds = {},
      InvalidTrajectoryPolicy invalid_policy =
          InvalidTrajectoryPolicy::UsePreviousResult,
      ObstacleCruiseStopParameters cruise_parameters = {},
      CruiseCandidateParameters cruise_candidate_parameters = {},
      CruiseVehicleGeometry cruise_vehicle = {},
      // 원시 중심선 선행 절단은 원본 입력 짝이 확인될 때까지 선택적으로 검증합니다.
      bool experimental_route_sequence_clip = false,
      // 기본 launch의 backward_path_length. 발행 경로 절단은 여기에 2m를 더합니다.
      double route_backward_path_length_m = 5.0,
      VelocityLimitSelectorConfig selector_config = {11.1},
      CruisePidParameters cruise_pid_parameters = {});

  PlanningPipelineMapAdapter(const PlanningPipelineMapAdapter &) = delete;
  PlanningPipelineMapAdapter &operator=(const PlanningPipelineMapAdapter &) = delete;

  // 콜백 시각의 객체·신호·차량 입력을 경로와 합쳐 플러그인부터 검증까지 실행합니다.
  PlanningPipelineMapResult onFrame(
      const PlanningPipelineFrame &frame,
      const BehaviorVelocityMapInput &velocity_context,
      const PlanningCruisePerceptionFrame &cruise_perception = {},
      const PlanningPipelinePointcloudInput &pointcloud = {},
      const std::optional<PlanningCruisePerceptionFrame> &cruise_callback_perception =
          std::nullopt,
      const std::optional<PlanningOutOfLaneInput> &out_of_lane_input =
          std::nullopt);
  PlanningPipelineMapResult onRouteFrame(
      PlanningPipelineFrame frame, const MissionRoute &route,
      const BehaviorVelocityMapInput &velocity_context,
      const PlanningCruisePerceptionFrame &cruise_perception = {},
      const PlanningPipelinePointcloudInput &pointcloud = {},
      std::optional<BehaviorGoalPose> goal = std::nullopt,
      std::optional<BehaviorCandidateCallbackInput> candidates = std::nullopt,
      // 0이면 원본 RouteHandler처럼 차로별 Lanelet2 제한속도를 사용합니다.
      double route_speed_mps = 0.0,
      std::optional<BehaviorRouteActionInput> action = std::nullopt,
      bool route_lane_change_activated = false,
      const std::optional<PlanningCruisePerceptionFrame> &cruise_callback_perception =
          std::nullopt,
      const std::optional<PlanningOutOfLaneInput> &out_of_lane_input =
          std::nullopt,
      PlanningRouteControlState route_control = {});
  MotionVelocityPlan planOutOfLane(
      const Trajectory &path, const TrajectoryPoint &ego,
      double now_s, const PlanningOutOfLaneInput &input);
  bool loadBehaviorVelocityModule(BehaviorVelocityModule module);
  bool unloadBehaviorVelocityModule(BehaviorVelocityModule module);
  bool loadBehaviorVelocityPlugin(std::string_view class_name);
  bool unloadBehaviorVelocityPlugin(std::string_view module_name);
  bool loadMotionVelocityPlugin(std::string_view class_name);
  bool unloadMotionVelocityPlugin(std::string_view class_name);
  const std::vector<MotionVelocityModule> &motionVelocityModules() const noexcept {
    return motion_modules_;
  }
  void reset();

 private:
  std::vector<std::uint8_t> map_data_;
  std::vector<BehaviorVelocityModule> modules_;
  std::unique_ptr<BehaviorVelocityMapPlanner> map_planner_;
  LaneletBinAdapter route_map_;
  bool experimental_route_sequence_clip_ = false;
  double route_backward_path_length_m_ = 5.0;
  std::vector<std::int64_t> route_signature_;
  std::optional<std::int64_t> current_route_lane_id_;
  CostmapGeneratorCore costmap_;
  BehaviorVelocityMapInput velocity_context_;
  std::optional<BehaviorVelocityMapResult> last_velocity_result_;
  DynamicObstacleStopCore dynamic_stop_;
  std::vector<MotionVelocityModule> motion_modules_{
      MotionVelocityModule::OutOfLane,
      MotionVelocityModule::ObstacleVelocityLimiter,
      MotionVelocityModule::DynamicObstacleStop};
  OutOfLaneDecisionCore out_of_lane_;
  std::optional<PlanningOutOfLaneInput> out_of_lane_input_;
  CruiseVehicleGeometry cruise_vehicle_;
  PlanningCruiseStopCandidateCore cruise_candidates_;
  ObstacleCruisePidCore cruise_pid_;
  VelocityLimitSelectorConfig selector_config_;
  VelocityLimitSelector velocity_limit_selector_;
  bool cruise_limit_active_ = false;
  PlanningCruisePerceptionFrame cruise_perception_;
  double cruise_acceleration_mps2_ = 0.0;
  PlanningPipelinePointcloudInput pointcloud_;
  ObstacleCruiseSlowdownCore cruise_slowdown_;
  CruiseSlowdownCandidateState cruise_slowdown_candidates_;
  std::vector<CruisePointcloudObstacle> pointcloud_obstacles_;
  std::size_t last_slowdown_candidate_count_ = 0;
  std::vector<MotionVelocitySlowdown> last_slowdown_intervals_;
  PlanningPipelineCore pipeline_;
  std::vector<std::int64_t> active_route_ids_;
  std::vector<StaticAvoidanceAnchoredShiftArc> registered_avoidance_;
  BehaviorPullOverPhaseState pull_over_phase_;
  std::optional<BehaviorRouteActionResult> active_pull_over_action_;
  std::optional<BehaviorRoutePullOverRequest> active_pull_over_request_;
  BehaviorRoutePhaseState start_phase_;
  std::optional<BehaviorRouteActionResult> active_start_action_;
  std::optional<BehaviorRouteStartRequest> active_start_request_;
  std::optional<BehaviorRouteActionResult> active_side_shift_action_;
  std::optional<BehaviorRouteSideShiftRequest> active_side_shift_request_;
  std::optional<BehaviorRouteActionResult> active_lane_change_action_;
  std::vector<std::int64_t> active_lane_change_target_ids_;
  std::vector<std::int64_t> active_lane_change_route_signature_;
  Trajectory active_lane_change_target_reference_;
  Trajectory active_sampling_path_;
  std::optional<BehaviorDynamicAvoidanceTracker>
      dynamic_avoidance_tracker_;
  std::optional<BehaviorDynamicAvoidanceParameters>
      dynamic_avoidance_parameters_;
  std::unordered_map<std::string, PlanningPerceivedObject>
      dynamic_avoidance_last_objects_;
  std::unordered_map<std::string, BehaviorSafetyPose>
      dynamic_avoidance_feasibility_origin_;
  struct DynamicAvoidancePreviousBoundary {
    double near_m = 0.0;
    Trajectory reference;
  };
  std::unordered_map<std::string, DynamicAvoidancePreviousBoundary>
      dynamic_avoidance_previous_boundary_;
  std::unordered_map<std::string, double> dynamic_inside_last_time_s_;
  std::optional<double> last_side_shift_request_time_s_;
  std::optional<double> ego_stopped_since_s_;
  struct StationaryObservationState {
    double since_s = 0.0;
    double last_seen_s = 0.0;
    double x = 0.0;
    double y = 0.0;
  };
  std::unordered_map<std::string, StationaryObservationState>
      stationary_since_s_;
};

}  // planning 네임스페이스
