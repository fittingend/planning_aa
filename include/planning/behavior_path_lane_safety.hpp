// SPDX-License-Identifier: Apache-2.0
// 차로 다각형과 차량 외형으로 행동 경로의 이탈 후보를 찾습니다.

#pragma once

#include "planning/behavior_path_shift.hpp"
#include "planning/behavior_path_object_safety.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace planning {

struct BehaviorPathPoint2d {
  double x = 0.0;
  double y = 0.0;
};

struct BehaviorPathVehicleFootprint {
  double front_m = 3.79;
  double back_m = 1.10;
  double half_width_m = 0.948;
  double sample_spacing_m = 0.5;
};

struct BehaviorPathLaneDeparture {
  std::size_t segment_index = 0;
  double segment_fraction = 0.0;
};

// 차로 변경 모듈의 isValidPath처럼 경로점의 위치만 차로 안에 있는지 검사합니다.
bool behaviorPathPointsInsideLanes(
    const Trajectory &path,
    const std::vector<std::vector<BehaviorPathPoint2d>> &lane_polygons);

// 경로 점과 점 사이도 간격별로 검사하며 첫 이탈 위치를 반환합니다.
std::optional<BehaviorPathLaneDeparture> firstBehaviorPathLaneDeparture(
    const Trajectory &path,
    const std::vector<std::vector<BehaviorPathPoint2d>> &lane_polygons,
    const BehaviorPathVehicleFootprint &vehicle = {});

// 이미 만들어진 출발 후보에서 차로 이탈 후보만 제거합니다.
std::vector<StartPullOutCandidate> selectLaneSafeStartPullOutCandidates(
    const std::vector<StartPullOutCandidate> &candidates,
    const std::vector<std::vector<BehaviorPathPoint2d>> &lane_polygons,
    const BehaviorPathVehicleFootprint &vehicle = {});

struct StartPullOutPoseSample {
  BehaviorSafetyPose pose;
  double backward_distance_m = 0.0;
  double lane_arc_m = 0.0;
};

// 지도 중심선을 출발 자세의 측방 거리만큼 옮긴 뒤 지정한 후진 거리에서 보간합니다.
std::vector<std::optional<BehaviorSafetyPose>> sampleStartPullOutBackwardPoses(
    const std::vector<BehaviorPathPoint2d> &centerline,
    const BehaviorSafetyPose &start_pose,
    const std::vector<double> &backward_distances_m);

struct StartPullOutSearchInput {
  std::vector<StartPullOutPoseSample> samples;
  double start_lane_arc_m = 0.0;
  double lane_length_m = 0.0;
  double base_link_to_rear_m = 0.0;
  double maximum_backward_distance_m = 0.0;
  double ignore_distance_from_lane_end_m = 0.0;
  double front_collision_margin_m = 0.0;
  double backward_collision_margin_m = 0.0;
  BehaviorSafetyFootprint vehicle;
  std::vector<BehaviorSafetyPolygon> front_stop_objects;
  std::vector<BehaviorSafetyPolygon> all_stop_objects;
};

struct StartPullOutStopObject {
  BehaviorSafetyPose pose;
  BehaviorSafetyObject shape;
  double velocity_x_mps = 0.0;
  double velocity_y_mps = 0.0;
  bool allowed_class = true;
};

// 정지 속도·차로 외형과 방향·경로 전후방 거리·종류 순서로 객체를 고릅니다.
std::vector<BehaviorSafetyPolygon> filterStartPullOutStopObjects(
    const std::vector<StartPullOutStopObject> &objects,
    const std::vector<std::vector<BehaviorPathPoint2d>> &lane_polygons,
    const std::vector<BehaviorPathPoint2d> &reference_path,
    double current_x, double current_y,
    double maximum_velocity_mps,
    double forward_distance_m, double backward_distance_m);

struct StartPullOutCandidatePlanInput {
  std::vector<BehaviorPathPoint2d> centerline;
  std::vector<std::vector<BehaviorPathPoint2d>> lane_polygons;
  BehaviorSafetyPose original_start_pose;
  std::vector<StartPullOutStopObject> objects;
  double start_lane_arc_m = 0.0;
  double lane_length_m = 0.0;
  double base_link_to_rear_m = 1.10;
  double maximum_backward_distance_m = 20.0;
  double backward_search_resolution_m = 2.0;
  double backward_path_length_m = 5.0;
  double ignore_distance_from_lane_end_m = 0.0;
  double moving_object_threshold_mps = 1.0;
  double first_collision_margin_m = 5.0;
  double second_collision_margin_m = 0.1;
  BehaviorSafetyFootprint vehicle;
};

// 지도 중심선 표본·정지 객체 선별·후방 후보 제외를 한 번에 계산합니다.
std::vector<BehaviorSafetyPose> planStartPullOutPoseCandidates(
    const StartPullOutCandidatePlanInput &input);

// 지도에서 표본화한 후방 자세에 원본 출발 후보의 제외·중단 순서를 적용합니다.
std::vector<BehaviorSafetyPose> searchStartPullOutPoseCandidates(
    const StartPullOutSearchInput &input);

}  // planning 네임스페이스
