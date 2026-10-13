// 이진 지도의 규제 요소를 경로 차로에 연결해 행동 속도 코어를 순서대로 실행합니다.

#pragma once

#include "planning/lanelet_bin_adapter.hpp"
#include "planning/crosswalk_object_state.hpp"
#include "planning/crosswalk_occlusion.hpp"
#include "planning/occlusion_spot_core.hpp"
#include "planning/traffic_light_core.hpp"
#include "planning/walkway_core.hpp"
#include "planning/intersection_stuck_core.hpp"
#include "planning/merge_from_private_core.hpp"
#include "planning/intersection_collision_core.hpp"
#include "planning/intersection_occlusion_core.hpp"
#include "planning/detection_area_core.hpp"
#include "planning/dynamic_obstacle_stop_core.hpp"
#include "planning/speed_bump_core.hpp"
#include "planning/no_drivable_lane_core.hpp"
#include "planning/virtual_traffic_light_core.hpp"
#include "planning/blind_spot_core.hpp"
#include "planning/run_out_state.hpp"
#include "planning/run_out_collision.hpp"
#include "planning/run_out_obstacle_creator.hpp"

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace planning {

enum class BehaviorVelocityModule {
  StopSign, TrafficLight, CrosswalkDefault,
  Crosswalk = CrosswalkDefault, Walkway, IntersectionStuck,
  IntersectionCollision, DetectionArea, SpeedBump, NoDrivableLane,
  NoStoppingArea, MergeFromPrivate, VirtualTrafficLight, RunOut, BlindSpot,
  OcclusionSpot
};

// The source load service accepts pluginlib class names, while unload accepts
// getModuleName() values. Composite plugins are left unmapped until their
// shared state and execution order are represented as one unit by the
// pipeline adapter. These one-to-one converters deliberately return empty
// for IntersectionModulePlugin / "intersection".
std::optional<BehaviorVelocityModule> behaviorVelocityLoadClass(
    std::string_view class_name);
std::optional<BehaviorVelocityModule> behaviorVelocityUnloadName(
    std::string_view module_name);

struct BehaviorCrosswalkObject {
  std::string id;
  CrosswalkPoint position;
  std::uint8_t classification = 7;
  CrosswalkPredictedObject predicted;
};

struct NoStoppingVehicle {
  CrosswalkPoint position;
  double yaw_rad = 0.0;
  double length_m = 3.0;
  double width_m = 3.0;
  double speed_mps = 0.0;
  std::uint8_t classification = 1;
  bool target_vehicle = true;
};

struct IntersectionOcclusionStopLines {
  double default_arc_m = 0.0;
  double peeking_arc_m = 0.0;
};

struct BehaviorVelocityMapInput {
  Trajectory trajectory;
  std::vector<std::vector<std::int64_t>> lane_ids;
  double ego_x = 0.0;
  double ego_y = 0.0;
  double ego_velocity_mps = 0.0;
  double ego_acceleration_mps2 = 0.0;
  double now_s = 0.0;
  std::optional<double> signal_stamp_s;
  std::optional<bool> signal_requires_stop;
  bool is_simulation = false;
  std::map<std::int64_t, TrafficLightSignalObservation>
      traffic_signal_observations;
  std::vector<BehaviorCrosswalkObject> crosswalk_objects;
  std::vector<DynamicObject> dynamic_stop_objects;
  std::map<std::int64_t, CrosswalkPedestrianSignal> pedestrian_signals;
  // Legacy name: true selects the activated/go branch for clear crossings.
  // In the source node with enable_rtc=false, setSafe() selects this branch.
  bool crosswalk_enable_rtc = false;
  bool crosswalk_auto_activate_when_safe = false;
  bool crosswalk_occlusion_enabled = false;
  std::optional<CrosswalkOccupancyGrid> crosswalk_occupancy_grid;
  std::vector<CrosswalkOcclusionMaskObject> crosswalk_occlusion_mask_objects;
  CrosswalkOcclusionParameters crosswalk_occlusion_parameters;
  std::optional<CrosswalkOccupancyGrid> occlusion_spot_occupancy_grid;
  std::vector<CrosswalkOcclusionMaskObject> occlusion_spot_vehicles;
  OcclusionSpotParameters occlusion_spot_parameters;
  std::vector<CrosswalkStuckVehicle> crosswalk_stuck_vehicles;
  std::vector<IntersectionStuckObject> intersection_stuck_vehicles;
  std::vector<IntersectionPredictedVehicle> intersection_predicted_vehicles;
  std::vector<CrosswalkPoint> detection_area_obstacle_points;
  std::vector<NoStoppingVehicle> no_stopping_vehicles;
  std::map<std::int64_t, TrafficLampColor> intersection_signal_colors;
  bool intersection_occlusion_enabled = false;
  std::optional<CrosswalkOccupancyGrid> intersection_occupancy_grid;
  // 호출자가 두 정지선의 경로 호길이를 제공하면 정적 가림 시간 해제를 적용합니다.
  std::map<std::int64_t, IntersectionOcclusionStopLines>
      intersection_occlusion_stoplines;
  CrosswalkStuckParameters crosswalk_stuck_parameters;
  std::map<std::int64_t, VirtualTrafficLightMessage>
      virtual_traffic_light_messages;
  VirtualTrafficLightParameters virtual_traffic_light_parameters{3.0, true};
  double virtual_traffic_light_front_offset_m = 3.79;
  double virtual_traffic_light_dead_line_margin_m = 1.0;
  double virtual_traffic_light_near_line_distance_m = 1.0;
  double virtual_traffic_light_hold_stop_margin_m = 0.0;
  std::map<std::int64_t, BlindSpotInput> blind_spot_inputs;
  std::vector<BlindSpotMapObject> blind_spot_map_objects;
  bool blind_spot_auto_enabled = false;
  BlindSpotParameters blind_spot_parameters;
  // 이미 계산한 충돌 호길이가 있으면 이를 우선 사용합니다.
  std::optional<double> run_out_collision_arc_m;
  std::vector<RunOutCollisionObject> run_out_objects;
  RunOutDetectionMode run_out_detection_mode = RunOutDetectionMode::Object;
  std::vector<RunOutPoint> run_out_points;
  std::vector<std::vector<CruisePredictedPose>> run_out_detection_polygons;
  RunOutCreatorParameters run_out_creator_parameters;
  RunOutCollisionParameters run_out_collision_parameters;
  RunOutObjectFilterParameters run_out_object_filter_parameters;
  bool run_out_use_map_partitions = true;
  bool run_out_object_detection = true;
  RunOutBrakingParameters run_out_braking_parameters;
};

struct BehaviorVelocityMapResult {
  Trajectory trajectory;
  std::vector<std::int64_t> stop_sign_regulatory_ids;
  std::vector<std::int64_t> traffic_light_regulatory_ids;
  std::vector<std::int64_t> crosswalk_lanelet_ids;
  std::vector<std::int64_t> walkway_lanelet_ids;
  std::vector<std::int64_t> intersection_stuck_lanelet_ids;
  std::vector<std::int64_t> intersection_collision_lanelet_ids;
  std::vector<std::int64_t> intersection_occlusion_lanelet_ids;
  std::map<std::int64_t, IntersectionOcclusionStatus>
      intersection_occlusion_statuses;
  std::vector<std::int64_t> detection_area_regulatory_ids;
  std::vector<std::int64_t> speed_bump_regulatory_ids;
  std::vector<std::int64_t> no_drivable_lanelet_ids;
  std::vector<std::int64_t> no_stopping_area_regulatory_ids;
  std::vector<std::int64_t> merge_from_private_lanelet_ids;
  std::map<std::int64_t, VirtualTrafficLightResult>
      virtual_traffic_light_states;
  std::map<std::int64_t, VirtualTrafficLightCommand>
      virtual_traffic_light_commands;
  std::map<std::int64_t, BlindSpotDecision> blind_spot_decisions;
  std::optional<double> run_out_stop_arc_m;
  std::optional<RunOutCollision> run_out_collision;
  std::vector<double> occlusion_spot_collision_arcs_m;
  bool run_out_deceleration_required = false;
  std::map<std::int64_t, std::map<std::string, CrosswalkCollisionState>>
      crosswalk_object_states;
};

class BehaviorVelocityMapPlanner {
 public:
  explicit BehaviorVelocityMapPlanner(
      const std::vector<std::uint8_t> &map_data,
      std::vector<BehaviorVelocityModule> modules = {
          BehaviorVelocityModule::Crosswalk,
          BehaviorVelocityModule::Walkway,
          BehaviorVelocityModule::TrafficLight,
          BehaviorVelocityModule::StopSign},
      RunOutTemporalParameters run_out_temporal_parameters = {});

  BehaviorVelocityMapResult apply(const BehaviorVelocityMapInput &input);
  // 원본 planner_manager의 실행 중 plugin 적재·해제 의미를 코어에서 제공합니다.
  // 이미 적재된 모듈 또는 없는 모듈에 대한 요청은 false를 반환합니다.
  bool loadModule(BehaviorVelocityModule module);
  bool unloadModule(BehaviorVelocityModule module);
  const std::vector<BehaviorVelocityModule> &modules() const noexcept {
    return modules_;
  }

 private:
  const std::vector<CrosswalkBounds> &conflictsFor(std::int64_t lanelet_id);
  struct CrosswalkMapFeature {
    std::int64_t lanelet_id = 0;
    CrosswalkBounds bounds;
    std::vector<CrosswalkPoint> polygon;
    std::vector<std::int64_t> traffic_light_ids;
  };

  LaneletBinAdapter map_;
  std::vector<BehaviorVelocityModule> modules_;
  std::vector<CrosswalkMapFeature> crosswalks_;
  std::map<std::int64_t, WalkwayCore> walkways_;
  std::map<std::int64_t, std::vector<CrosswalkBounds>> intersection_conflicts_;
  std::map<std::int64_t, IntersectionCollisionState> intersection_collision_states_;
  std::map<std::int64_t, IntersectionCollisionState> intersection_occlusion_states_;
  std::map<std::int64_t, IntersectionOcclusionReleaseState>
      intersection_occlusion_release_states_;
  std::optional<double> intersection_ego_stopped_since_s_;
  std::map<std::int64_t, DetectionAreaCore> detection_areas_;
  std::map<std::int64_t, DetectionAreaCore> no_stopping_areas_;
  std::set<std::int64_t> no_stopping_unstoppable_;
  std::map<std::int64_t, MergeFromPrivateCore> merge_from_private_;
  std::map<std::int64_t, NoDrivableLaneCore> no_drivable_lanes_;
  std::map<std::pair<std::int64_t, std::size_t>, StopLineCore> stop_signs_;
  std::map<std::pair<std::int64_t, std::size_t>, TrafficLightCore> traffic_lights_;
  std::map<std::int64_t, CrosswalkObjectStateTracker> crosswalk_object_states_;
  std::map<std::int64_t, CrosswalkOcclusionPlanner> crosswalk_occlusion_states_;
  std::map<std::int64_t, VirtualTrafficLightCore> virtual_traffic_lights_;
  std::map<std::int64_t, BlindSpotCore> blind_spots_;
  RunOutTemporalCore run_out_temporal_;
  RunOutTemporalParameters run_out_temporal_parameters_;
  RunOutObjectFilterCore run_out_object_filter_;
  std::vector<RunOutObjectFilterParameters::PartitionSegment>
      run_out_map_partitions_;
  std::vector<VelocityLimiterSegment> occlusion_spot_partitions_;
  std::map<std::int64_t, double> crosswalk_ego_stopped_since_s_;
};

}  // planning 네임스페이스
