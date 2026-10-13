// SPDX-License-Identifier: Apache-2.0
// 동적 장애물 회피 대상의 종류와 연속 관측 상태를 ROS 없이 관리합니다.

#pragma once

#include "planning/behavior_path_object_safety.hpp"
#include "planning/trajectory.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>
#include <optional>

namespace planning {

enum class BehaviorDynamicObjectType {
  OutOfScope,
  Regulated,
  Unregulated,
};

// 원본 모듈은 첫 두 경로 자세가 나타내는 진행 방향이 전진일 때만 실행한다.
bool behaviorDynamicPathForward(const Trajectory &path);

struct BehaviorDynamicAvoidanceParameters {
  std::array<bool, 8> avoid_types{
      false, true, true, true, true, true, true, true};
  double minimum_speed_mps = 0.0;
  double maximum_speed_mps = 100.0;
  int successive_entry_count = 5;
  int successive_exit_count = 1;
  double maximum_object_lateral_distance_m = 1.0;
  double maximum_overtaking_crossing_angle_rad = 1.05;
  double maximum_oncoming_crossing_angle_rad = 0.523;
  double minimum_overtaking_crossing_speed_mps = 1.0;
  double minimum_oncoming_crossing_speed_mps = 1.0;
  double maximum_pedestrian_lateral_speed_mps = 0.8;
  double maximum_front_object_angle_rad = 0.785;
  double minimum_front_object_speed_mps = -0.5;
  double maximum_moving_object_collision_time_s = 40.0;
  double minimum_cut_in_object_speed_mps = 0.5;
  double minimum_cut_in_start_time_s = 1.0;
  double minimum_cut_in_longitudinal_distance_m = 0.0;
  double minimum_cut_out_object_speed_mps = 0.5;
  double minimum_cut_out_lateral_speed_mps = 0.3;
  double maximum_cut_out_since_inside_s = 2.0;
};

struct BehaviorDynamicRoughAdmissionInput {
  std::uint8_t classification = 0;
  double tangent_speed_mps = 0.0;
  double normal_speed_mps = 0.0;
  double speed_norm_mps = 0.0;
  double yaw_difference_rad = 0.0;
  double center_distance_to_path_m = 0.0;
  double object_maximum_radius_m = 0.0;
  double ego_half_width_m = 0.948;
};

// 원본의 종류·속도·가로지름·경로 거리 1차 선별을 계산합니다.
bool behaviorDynamicRoughAdmission(
    const BehaviorDynamicRoughAdmissionInput &input,
    const BehaviorDynamicAvoidanceParameters &parameters);

struct BehaviorDynamicAvoidanceObservation {
  std::string id;
  std::uint8_t classification = 0;
  double speed_mps = 0.0;
  double x = 0.0;
  double y = 0.0;
};

BehaviorDynamicObjectType behaviorDynamicObjectType(
    std::uint8_t classification,
    const BehaviorDynamicAvoidanceParameters &parameters);

struct BehaviorDynamicAvoidanceCorridorInput {
  Trajectory reference;
  double object_x = 0.0;
  double object_y = 0.0;
  double longitudinal_start_m = 0.0;
  double longitudinal_end_m = 0.0;
  double inner_lateral_m = 0.0;
  double outer_lateral_m = 0.0;
};

struct BehaviorDynamicCollisionTimingInput {
  double ego_speed_mps = 0.0;
  double object_path_speed_mps = 0.0;
  double ego_to_object_path_m = 0.0;
  double object_longitudinal_min_m = 0.0;
  double object_longitudinal_max_m = 0.0;
  double ego_front_m = 0.0;
  double ego_rear_m = 0.0;
};

struct BehaviorDynamicCollisionTiming {
  double start_s = 0.0;
  double end_s = 0.0;
};

// 자차와 객체의 경로 방향 상대 속도에서 충돌 시작·종료 시각을 구합니다.
BehaviorDynamicCollisionTiming behaviorDynamicCollisionTiming(
    const BehaviorDynamicCollisionTimingInput &input);

struct BehaviorDynamicCutInInput {
  bool predicted_path_enters_ego_corridor = false;
  double object_path_speed_mps = 0.0;
  double ego_speed_mps = 0.0;
  double ego_to_object_back_m = 0.0;
  double minimum_object_speed_mps = 0.5;
  double minimum_longitudinal_distance_m = 0.0;
  double minimum_start_time_s = 1.0;
};

struct BehaviorDynamicCutOutInput {
  bool object_on_left = false;
  double object_path_speed_mps = 0.0;
  double object_lateral_speed_mps = 0.0;
  std::optional<double> time_since_inside_ego_path_s;
  double minimum_object_speed_mps = 0.5;
  double minimum_lateral_speed_mps = 0.3;
  double maximum_time_since_inside_s = 2.0;
};

struct BehaviorDynamicPredictedPath {
  double confidence = 0.0;
  double time_step_s = 0.0;
  std::vector<BehaviorSafetyPose> poses;
};

struct BehaviorDynamicPredictedPolygonInput {
  std::vector<BehaviorDynamicPredictedPath> paths;
  double object_length_m = 0.0;
  double object_width_m = 0.0;
  double end_time_s = 2.0;
  double minimum_confidence = 0.0;
  double expansion_margin_m = 2.0;
  BehaviorSafetyPolygon ego_reserve_polygon;
};

// 예측 자세의 차량 외곽을 볼록 껍질과 여유로 합치고 자차 통로를 제외합니다.
std::optional<BehaviorSafetyPolygon> behaviorDynamicPredictedPolygon(
    const BehaviorDynamicPredictedPolygonInput &input);

// 보행자 예측 외곽에서 보존할 자차 통로를 객체의 좌우 위치에 맞춰 만듭니다.
std::optional<BehaviorSafetyPolygon> behaviorDynamicEgoReservePolygon(
    const Trajectory &reference, bool object_on_left,
    double vehicle_half_width_m = 0.948,
    double maximum_lateral_avoidance_m = 0.5);

// 규제 객체의 예측 경로를 차량 외형과 전후·측방 여유로 둘러쌉니다.
std::optional<BehaviorSafetyPolygon> behaviorDynamicObjectPathPolygon(
    const std::vector<BehaviorSafetyPose> &predicted_poses,
    double object_length_m, double object_width_m,
    double minimum_longitudinal_margin_m = 3.0,
    double lateral_margin_m = 1.0);

struct BehaviorDynamicLateralBounds {
  double near_m = 0.0;
  double far_m = 0.0;
};

struct BehaviorDynamicUnregulatedLateralInput {
  double object_min_lateral_m = 0.0;
  double object_max_lateral_m = 0.0;
  double object_lateral_speed_mps = 0.0;
  double prediction_time_s = 3.0;
  double obstacle_margin_m = 1.0;
  double vehicle_half_width_m = 0.948;
  double maximum_lateral_avoidance_m = 0.5;
  std::optional<double> previous_near_m;
  bool use_lowpass = true;
  double lowpass_gain = 0.9;
};

// 보행자·자전거의 현재·예상 측방 점유를 자차 회피 경계로 바꿉니다.
std::optional<BehaviorDynamicLateralBounds>
behaviorDynamicUnregulatedLateralBounds(
    const BehaviorDynamicUnregulatedLateralInput &input);

struct BehaviorDynamicRegulatedLateralInput {
  double object_min_lateral_m = 0.0;
  double object_max_lateral_m = 0.0;
  double object_path_speed_mps = 0.0;
  double object_lateral_speed_mps = 0.0;
  bool collision_on_left = false;
  double minimum_front_object_speed_mps = -0.5;
  double maximum_ego_path_cover_ratio = 0.3;
  double obstacle_margin_m = 1.0;
  double lateral_prediction_time_s = 0.0;
  double vehicle_half_width_m = 0.948;
  double maximum_lateral_avoidance_m = 0.5;
  std::optional<double> previous_near_m;
  bool use_lowpass = true;
  double lowpass_gain = 0.9;
};

// 차량류 객체의 통로 겹침과 측방 속도에서 회피 경계를 계산합니다.
std::optional<BehaviorDynamicLateralBounds>
behaviorDynamicRegulatedLateralBounds(
    const BehaviorDynamicRegulatedLateralInput &input);

struct BehaviorDynamicLongitudinalBoundsInput {
  double object_min_longitudinal_m = 0.0;
  double object_max_longitudinal_m = 0.0;
  double object_path_speed_mps = 0.0;
  double ego_speed_mps = 0.0;
  BehaviorDynamicCollisionTiming collision_time;
  double valid_predicted_path_length_m = 0.0;
  double maximum_stopped_object_speed_mps = 0.5;
  double overtaking_start_duration_s = 1.0;
  double overtaking_end_duration_s = 1.0;
  double oncoming_start_duration_s = 1.0;
  double oncoming_end_duration_s = 0.0;
};

// 동적 객체의 현재 외형과 상대 속도에서 경로형 제외 구간의 앞뒤 길이를 계산합니다.
std::optional<std::pair<double, double>>
behaviorDynamicLongitudinalBounds(
    const BehaviorDynamicLongitudinalBoundsInput &input);

struct BehaviorDynamicPathOffsets {
  double object_arc_m = 0.0;
  double minimum_lateral_m = 0.0;
  double maximum_lateral_m = 0.0;
  double minimum_longitudinal_m = 0.0;
  double maximum_longitudinal_m = 0.0;
};

// 객체 외곽 각 점의 경로 측방 거리와 객체 선분 기준 전후 거리를 계산합니다.
std::optional<BehaviorDynamicPathOffsets> behaviorDynamicPathOffsets(
    const Trajectory &reference, double object_x, double object_y,
    const BehaviorSafetyPolygon &object_polygon);

struct BehaviorDynamicRegulatedPlanInput {
  Trajectory reference;
  double ego_x = 0.0;
  double ego_y = 0.0;
  double ego_yaw_rad = 0.0;
  double ego_speed_mps = 0.0;
  double ego_front_m = 3.79;
  double ego_rear_m = 1.10;
  double vehicle_half_width_m = 0.948;
  double maximum_ego_lateral_acceleration_mps2 = 0.3;
  double maximum_ego_lateral_jerk_mps3 = 0.3;
  double ego_shift_delay_s = 1.0;
  // Source manager freezes the reachable lateral paths when an object enters.
  std::optional<BehaviorSafetyPose> feasibility_origin;
  double object_x = 0.0;
  double object_y = 0.0;
  double object_path_speed_mps = 0.0;
  double object_lateral_speed_mps = 0.0;
  BehaviorSafetyPolygon object_polygon;
  bool collision_on_left = false;
  double valid_predicted_path_length_m = 1000.0;
  std::optional<double> previous_near_m;
  bool use_lateral_lowpass = true;
};

struct BehaviorDynamicRegulatedPlan {
  BehaviorSafetyPolygon exclusion_polygon;
  BehaviorDynamicCollisionTiming collision_time;
  BehaviorDynamicLateralBounds lateral_bounds;
  std::pair<double, double> longitudinal_bounds;
};

// 규제 차량의 외곽 투영부터 경로형 제외 다각형까지 순서대로 계산합니다.
std::optional<BehaviorDynamicRegulatedPlan>
planBehaviorDynamicRegulatedObject(
    const BehaviorDynamicRegulatedPlanInput &input);

bool behaviorDynamicObjectWillCutIn(
    const BehaviorDynamicCutInInput &input);
bool behaviorDynamicObjectWillCutOut(
    const BehaviorDynamicCutOutInput &input);

// 객체를 기준으로 한 전후·좌우 구간을 경로형 제외 다각형으로 만듭니다.
std::optional<BehaviorSafetyPolygon> behaviorDynamicAvoidanceCorridor(
    const BehaviorDynamicAvoidanceCorridorInput &input);

class BehaviorDynamicAvoidanceTracker {
 public:
  explicit BehaviorDynamicAvoidanceTracker(
      BehaviorDynamicAvoidanceParameters parameters = {});

  // 매 콜백에서 관측된 객체만 전달하고 유효 대상의 마지막 관측을 반환합니다.
  std::vector<BehaviorDynamicAvoidanceObservation> advance(
      const std::vector<BehaviorDynamicAvoidanceObservation> &observed);
  void reset();

 private:
  BehaviorDynamicAvoidanceParameters parameters_;
  std::unordered_map<std::string, int> counters_;
  std::unordered_map<std::string, BehaviorDynamicAvoidanceObservation>
      objects_;
  std::vector<std::string> valid_ids_;
};

}  // planning 네임스페이스
