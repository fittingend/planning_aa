// ROS 메시지에서 분리한 Lanelet2 이진 지도 자료를 읽는 선택적 어댑터입니다.

#pragma once

#include "planning/lanelet_osm_adapter.hpp"
#include "planning/blind_spot_core.hpp"
#include "planning/behavior_goal_refinement.hpp"
#include "planning/behavior_lane_change_route.hpp"
#include "planning/behavior_path_lane_safety.hpp"
#include "planning/obstacle_velocity_limiter_core.hpp"
#include "planning/out_of_lane_core.hpp"
#include "planning/scenario_selector.hpp"
#include "planning/speed_bump_core.hpp"
#include "planning/virtual_traffic_light_core.hpp"

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace planning {

struct LaneCenterlinePoint3D {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct TrafficLightStopLine {
  std::int64_t regulatory_id = 0;
  StopLineSegment segment;
};

struct IntersectionTrafficControlDistances {
  std::int64_t conflict_lanelet_id = 0;
  double distance_to_stopline_m = 0.0;
  double stopline_to_ego_path_m = 0.0;
};

struct DetectionAreaMapFeature {
  std::int64_t regulatory_id = 0;
  StopLineSegment stop_line;
  std::vector<std::vector<CrosswalkPoint>> areas;
};

struct NoStoppingAreaMapFeature {
  std::int64_t regulatory_id = 0;
  std::optional<StopLineSegment> stop_line;
  std::vector<std::vector<CrosswalkPoint>> areas;
};

struct VirtualTrafficLightMapFeature {
  std::int64_t regulatory_id = 0;
  VirtualTrafficLightCommandMetadata command_metadata;
  std::vector<StopLineSegment> start_lines;
  std::vector<StopLineSegment> stop_lines;
  std::vector<StopLineSegment> end_lines;
};

struct BlindSpotMapRegion {
  std::int64_t lanelet_id = 0;
  std::vector<CrosswalkPoint> polygon;
  std::vector<CrosswalkPoint> centerline;
};

struct BlindSpotMapObject {
  std::string id;
  int classification = 0;
  CrosswalkPoint position;
  double longitudinal_velocity_mps = 0.0;
};

struct BlindSpotMappedObjects {
  double ego_arc_m = 0.0;
  double stop_arc_m = 0.0;
  std::vector<BlindSpotObject> objects;
};

struct BlindSpotStopArcs {
  double default_stop_arc_m = 0.0;
  double critical_stop_arc_m = 0.0;
};

struct RouteCheckpoint {
  double x = 0.0;
  double y = 0.0;
  double yaw_rad = 0.0;
  double z = 0.0;
  double orientation_x = 0.0;
  double orientation_y = 0.0;
  double orientation_z = 0.0;
  double orientation_w = 1.0;
  bool has_orientation = false;
};

struct MissionRouteSection {
  std::int64_t preferred_lanelet_id = 0;
  std::vector<std::int64_t> primitive_lanelet_ids;
};

struct MissionRoute {
  std::vector<MissionRouteSection> sections;
  double goal_height_m = 0.0;
};

struct CostmapAreaPolygons {
  std::vector<BehaviorSafetyPolygon> road;
  std::vector<BehaviorSafetyPolygon> parking;
};

class LaneletBinAdapter {
 public:
  explicit LaneletBinAdapter(const std::vector<std::uint8_t> &map_data);
  ~LaneletBinAdapter();
  LaneletBinAdapter(LaneletBinAdapter &&) noexcept;
  LaneletBinAdapter &operator=(LaneletBinAdapter &&) noexcept;
  LaneletBinAdapter(const LaneletBinAdapter &) = delete;
  LaneletBinAdapter &operator=(const LaneletBinAdapter &) = delete;

  std::vector<StopSignStopLine> stopSignStopLinesForLanelet(
      std::int64_t lanelet_id) const;
  std::vector<StopLineSegment> trafficLightStopLines(
      std::int64_t regulatory_id) const;
  std::vector<TrafficLightStopLine> trafficLightStopLinesForLanelet(
      std::int64_t lanelet_id) const;
  LaneTurnDirection laneTurnDirection(std::int64_t lanelet_id) const;
  CrosswalkBounds crosswalkBounds(std::int64_t lanelet_id) const;
  std::vector<std::int64_t> crosswalkLaneletIds() const;
  std::vector<std::int64_t> crosswalkTrafficLightIds(
      std::int64_t lanelet_id) const;
  std::vector<std::int64_t> walkwayLaneletIds() const;
  CrosswalkBounds walkwayBounds(std::int64_t lanelet_id) const;
  std::vector<StopLineSegment> walkwayStopLines(
      std::int64_t lanelet_id) const;
  std::vector<std::int64_t> intersectionLaneletIds() const;
  std::vector<std::int64_t> intersectionConflictingLaneletIds(
      std::int64_t lanelet_id) const;
  std::vector<std::int64_t> intersectionOcclusionLaneletIds(
      std::int64_t lanelet_id, double preceding_length_m = 70.0) const;
  std::vector<std::vector<CrosswalkPoint>> intersectionOcclusionDivisions(
      std::int64_t lanelet_id, double resolution_m,
      double preceding_length_m = 70.0) const;
  CrosswalkBounds laneBounds(std::int64_t lanelet_id) const;
  std::vector<CrosswalkPoint> laneCenterline(
      std::int64_t lanelet_id) const;
  std::vector<LaneCenterlinePoint3D> laneCenterline3d(
      std::int64_t lanelet_id) const;
  // RouteHandler가 중심선 경로에 기록하는 차로별 교통규칙 속도(m/s).
  double laneSpeedLimit(std::int64_t lanelet_id) const;
  // 지정한 목표 차로 순서의 지도 중심선을 실제 차로 변경 기준 경로로 추출합니다.
  std::optional<LaneChangeMappedReference> laneChangeTargetReference(
      const std::vector<std::int64_t> &target_lanelet_ids,
      BehaviorPathShiftAnchor change_start, double change_length_m,
      double change_velocity_mps, double forward_path_length_m,
      double next_change_buffer_m,
      std::optional<BehaviorPathShiftAnchor> goal = std::nullopt) const;
  // 현재 경로 앞의 교차로·횡단보도·신호등 정지선 거리를 구합니다.
  LaneChangeRegulatoryDistances laneChangeRegulatoryDistances(
      const Trajectory &reference,
      const std::vector<std::int64_t> &current_lanelet_ids,
      double ego_x, double ego_y) const;
  // 세계 좌표에서 지정한 거리 안의 가장 가까운 도로 차로를 찾습니다.
  std::optional<std::int64_t> nearestRoadLaneletId(
      double x, double y, double maximum_distance_m = 10.0) const;
  // RouteHandler의 route 차로 집합 검색: 다각형 거리 우선, 동률은 방향으로 고릅니다.
  std::optional<std::int64_t> closestRouteRoadLaneletId(
      double x, double y, double yaw_rad,
      const std::vector<std::int64_t> &candidate_ids,
      double maximum_distance_m = std::numeric_limits<double>::infinity(),
      double maximum_yaw_rad = 3.141592653589793) const;
  // 같은 방향의 인접 도로 차로인지 지도 연결 관계로 확인합니다.
  bool areAdjacentRoadLanelets(std::int64_t current_lanelet_id,
                               std::int64_t target_lanelet_id) const;
  // 선호 차로가 여러 칸 떨어졌을 때 한 번에 이동할 이웃 차로를 고릅니다.
  std::optional<std::int64_t> nextRoadLaneletTowardPreferred(
      std::int64_t current_lanelet_id,
      std::int64_t preferred_lanelet_id,
      const std::vector<std::int64_t> &allowed_ids) const;
  // 객체 중심이 횡단보도 또는 교차로 지도 영역 안에 있는지 확인합니다.
  bool pointInCrosswalk(double x, double y) const;
  bool pointInIntersection(double x, double y) const;
  bool pointInRoadLanelet(std::int64_t lanelet_id,
                          double x, double y) const;
  bool hasAdjacentRoadLanelet(std::int64_t lanelet_id,
                              bool left_side) const;
  // 요청 방향의 같은 방향 인접 도로 차로 ID를 반환합니다.
  std::optional<std::int64_t> adjacentRoadLaneletId(
      std::int64_t lanelet_id, bool left_side) const;
  // 같은 경계를 역방향으로 공유하는 반대편 도로 차로를 찾습니다.
  bool hasOppositeRoadLanelet(std::int64_t lanelet_id,
                             bool left_side) const;
  // 다음 경로 구간 후보에서 지도 연결 관계가 있는 도로 차로를 고릅니다.
  std::optional<std::int64_t> followingRoadLaneletId(
      std::int64_t lanelet_id,
      const std::vector<std::int64_t> &candidate_ids,
      std::int64_t preferred_lanelet_id = 0) const;
  // 지정한 차로의 좌우 경계로 출발 경로 이탈 검사 다각형을 만듭니다.
  std::vector<std::vector<BehaviorPathPoint2d>> lanePolygons(
      const std::vector<std::int64_t> &lanelet_ids) const;
  // Costmap의 허용 영역: 모든 road 차로, parking_lot, 폭이 있는 parking_space.
  CostmapAreaPolygons costmapAreaPolygons() const;
  // 경로의 차량 외곽 볼록 껍질과 겹치는 지도 차로 다각형을 모읍니다.
  std::vector<std::vector<BehaviorPathPoint2d>> lanePolygonsNearPath(
      const Trajectory &path,
      const BehaviorPathVehicleFootprint &vehicle = {}) const;
  // 목표 차로 밖의 목표를 중심선으로 보정하고 경로의 차로 포함 여부를 확인합니다.
  BehaviorGoalPath refineFixedGoalPath(
      const BehaviorGoalPath &input, BehaviorGoalPose goal,
      std::int64_t goal_lanelet_id,
      double initial_search_radius_m = 7.5) const;
  // 회전 차로의 바깥쪽 절반과 같은 방향 인접 차로의 감지 띠를 만듭니다.
  std::optional<BlindSpotMapRegion> blindSpotRegion(
      std::int64_t lanelet_id, double ignore_width_from_center_m = 0.7,
      double adjacent_extend_width_m = 1.5,
      LaneTurnDirection direction = LaneTurnDirection::Else) const;
  // 경로에서 회전 차로 이전의 차로를 따라 객체의 감지 구역과 호길이를 계산합니다.
  BlindSpotMappedObjects mapBlindSpotObjects(
      const std::vector<std::int64_t> &preceding_lane_ids,
      LaneTurnDirection direction, CrosswalkPoint ego_position,
      const std::vector<BlindSpotMapObject> &objects,
      double backward_detection_length_m = 100.0,
      std::optional<CrosswalkPoint> stop_position = std::nullopt) const;
  // 형제 직진 차로 경계가 있으면 차량 외형의 첫 교차점, 없으면 회전 차로 진입점을 씁니다.
  // 다중 차로 ID 입력점은 원본 경로 재표본화처럼 진입점 탐색용 경로에 보존합니다.
  std::optional<BlindSpotStopArcs> blindSpotStopArcs(
      std::int64_t lanelet_id, const Trajectory &path,
      double front_offset_m = 3.79, double rear_offset_m = 1.1,
      double half_width_m = 0.948, double stop_line_margin_m = 1.0,
      const std::vector<CrosswalkPoint> &multi_lane_points = {}) const;
  std::vector<VelocityLimiterSegment> staticObstacleSegments(
      double minimum_x, double minimum_y,
      double maximum_x, double maximum_y,
      const std::vector<std::string> &tags = {"guard_rail"}) const;
  // 돌발 진입 필터가 쓰는 가드레일·울타리·벽 구획 선분을 읽습니다.
  std::vector<VelocityLimiterSegment> partitionSegments() const;
  MotionVelocityPlan planStaticObstacleVelocityLimit(
      const Trajectory &path, const std::vector<double> &time_from_start,
      std::size_t ego_index, double current_ego_velocity,
      const ObstacleVelocityLimiterParameters &parameters = {},
      const std::vector<std::string> &tags = {"guard_rail"},
      const BicycleVelocityProjectionParameters &bicycle_parameters = {}) const;
  // 경로와 겹치는 도로 차로와 직접 선행 차로를 제외한 주변 도로 차로를 찾습니다.
  std::vector<std::int64_t> outOfLaneRoadLaneletIds(
      const std::vector<CrosswalkPoint> &path,
      double maximum_footprint_offset_m = 5.24,
      double front_offset_m = 3.74) const;
  std::vector<OutOfLanePolygon> outOfLaneRoadPolygons(
      const Trajectory &path,
      double maximum_footprint_offset_m = 5.24,
      double front_offset_m = 3.74) const;
  std::optional<MotionVelocityPoint2D> findOutOfLaneMapCandidate(
      const Trajectory &path,
      const std::vector<double> &time_from_start,
      const std::vector<OutOfLanePredictedPath> &objects,
      double minimum_stop_arc,
      const OutOfLaneParameters &decision_parameters = {},
      const OutOfLaneGeometryParameters &geometry_parameters = {}) const;
  std::vector<OutOfLaneStopLine> outOfLaneRedStopLines(
      MotionVelocityPoint2D ego_position, double search_distance,
      const std::vector<std::int64_t> &red_traffic_light_ids) const;
  // 지도 차로·적색 정지선·객체 필터를 한 번의 차로 밖 주행 후보 계산에 연결합니다.
  std::optional<MotionVelocityPoint2D>
  findOutOfLaneMapCandidateFromTrackedObjects(
      const Trajectory &path,
      const std::vector<double> &time_from_start,
      MotionVelocityPoint2D ego_position,
      const std::vector<OutOfLaneTrackedObject> &objects,
      const std::vector<std::int64_t> &red_traffic_light_ids,
      double search_distance, double minimum_stop_arc,
      const OutOfLaneParameters &decision_parameters = {},
      const OutOfLaneGeometryParameters &geometry_parameters = {},
      const OutOfLaneObjectFilterParameters &filter_parameters = {}) const;
  bool isVehicleOnIntersectionConflictLane(
      std::int64_t lanelet_id, double x, double y,
      double yaw_rad, double distance_margin_m = 0.75,
      double heading_margin_rad = 0.785) const;
  bool isVehicleOnIntersectionAttentionLane(
      std::int64_t lanelet_id, double x, double y,
      double yaw_rad, double distance_margin_m = 0.75,
      double heading_margin_rad = 0.785) const;
  std::vector<std::int64_t> intersectionObservedTrafficLightIds(
      std::int64_t lanelet_id) const;
  bool intersectionHasTrafficLight(std::int64_t lanelet_id) const;
  std::vector<DetectionAreaMapFeature> detectionAreasForLanelet(
      std::int64_t lanelet_id) const;
  std::vector<NoStoppingAreaMapFeature> noStoppingAreasForLanelet(
      std::int64_t lanelet_id) const;
  std::vector<VirtualTrafficLightMapFeature> virtualTrafficLightsForLanelet(
      std::int64_t lanelet_id) const;
  std::vector<SpeedBumpFeature> speedBumpsForLanelet(
      std::int64_t lanelet_id) const;
  bool isNoDrivableLane(std::int64_t lanelet_id) const;
  bool isPrivateLane(std::int64_t lanelet_id) const;
  bool isUrbanLane(std::int64_t lanelet_id) const;
  std::optional<MissionRoute> routeBetweenCheckpoints(
      RouteCheckpoint start, RouteCheckpoint goal,
      bool consider_no_drivable_lanes = false,
      std::string *failure_reason = nullptr) const;
  std::optional<MissionRoute> routeThroughCheckpoints(
      const std::vector<RouteCheckpoint> &checkpoints,
      bool consider_no_drivable_lanes = false,
      std::string *failure_reason = nullptr) const;
  std::optional<IntersectionTrafficControlDistances>
  intersectionTrafficControlDistances(
      std::int64_t lanelet_id, double x, double y,
      double yaw_rad) const;
  ScenarioPosition scenarioPosition(
      double ego_x, double ego_y, double ego_yaw_rad,
      double goal_x, double goal_y,
      const std::vector<std::int64_t> &route_lanelet_ids) const;

 private:
  std::optional<MissionRoute> routeBetweenCheckpointsImpl(
      RouteCheckpoint start, RouteCheckpoint goal,
      bool consider_no_drivable_lanes, bool validate_goal,
      std::vector<std::int64_t> *path_lanelet_ids = nullptr,
      std::string *failure_reason = nullptr) const;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // planning 네임스페이스
