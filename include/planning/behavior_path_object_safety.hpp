// SPDX-License-Identifier: Apache-2.0
// 행동 경로 후보의 예측 객체 외곽과 RSS 확장 외곽을 검사합니다.

#pragma once

#include <cstddef>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace planning {

struct BehaviorSafetyPose {
  double time_s = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw_rad = 0.0;
  double velocity_mps = 0.0;
};

struct BehaviorSafetyFootprint {
  double front_m = 3.79;
  double rear_m = 1.10;
  double half_width_m = 0.948;
};

struct BehaviorSafetyObject {
  // 원본 객체 외형 종류: 0 직사각형, 1 원기둥, 2 다각형입니다.
  int shape_type = 0;
  // 다각형 종류는 기록된 길이·폭이 0이어도 외곽 점을 사용합니다.
  double length_m = 4.0;
  double width_m = 1.8;
  // 다각형 종류에 한해서 객체 기준 좌표의 외곽 점을 사용합니다.
  std::vector<std::pair<double, double>> footprint;
  std::vector<BehaviorSafetyPose> predicted;
};

struct BehaviorSafetyPredictedPath {
  double confidence = 0.0;
  std::vector<BehaviorSafetyPose> poses;
};

using BehaviorSafetyPolygon = std::vector<std::pair<double, double>>;
using BehaviorSafetyRegions = std::vector<BehaviorSafetyPolygon>;

// 원본 객체 외형 종류와 자세로 닫힌 세계 좌표 다각형을 만듭니다.
BehaviorSafetyPolygon behaviorSafetyWorldPolygon(
    const BehaviorSafetyPose &pose, const BehaviorSafetyObject &object);

// 출발·목표 후보에서 차량 외곽과 정지 객체 외곽의 최소 거리를 여유와 비교합니다.
bool behaviorFootprintCollidesWithObjects(
    const BehaviorSafetyPose &ego_pose,
    const BehaviorSafetyFootprint &ego_vehicle,
    const std::vector<BehaviorSafetyPolygon> &object_polygons,
    double margin_m);

enum class BehaviorSafetyExtensionPolicy { Rectangle, AlongPath };

struct BehaviorSafetyRssParameters {
  double front_deceleration_mps2 = -1.0;
  double rear_deceleration_mps2 = -1.0;
  double reaction_time_s = 2.0;
  double safety_time_margin_s = 1.0;
  double lateral_margin_m = 2.0;
  double minimum_longitudinal_m = 3.0;
  double longitudinal_velocity_delta_time_s = 0.0;
  double hysteresis_factor = 1.0;
  double yaw_difference_threshold_rad = 3.1416;
  double ego_velocity_limit_mps =
      std::numeric_limits<double>::infinity();
  BehaviorSafetyExtensionPolicy extension_policy =
      BehaviorSafetyExtensionPolicy::Rectangle;
};

enum class BehaviorSafetyCollisionKind {
  DirectOverlap,
  ExtendedOverlap
};

struct BehaviorSafetyCollision {
  BehaviorSafetyCollisionKind kind =
      BehaviorSafetyCollisionKind::DirectOverlap;
  std::size_t object_sample_index = 0;
  double rss_distance_m = 0.0;
  double longitudinal_extension_m = 0.0;
  double lateral_extension_m = 0.0;
  bool object_in_front = false;
  std::size_t predicted_path_index = 0;
};

// 객체 표본 시각에 자차 자세를 보간하고 첫 겹침을 반환합니다.
std::optional<BehaviorSafetyCollision> firstBehaviorPathObjectCollision(
    const std::vector<BehaviorSafetyPose> &ego_path,
    const BehaviorSafetyFootprint &ego_vehicle,
    const BehaviorSafetyObject &object,
    const BehaviorSafetyRssParameters &parameters = {},
    const BehaviorSafetyRegions &collision_regions = {},
    const std::vector<BehaviorSafetyPose> &planned_path = {});

// 원본처럼 최고 신뢰도 한 경로 또는 모든 경로를 대상으로 검사합니다.
std::optional<BehaviorSafetyCollision> firstBehaviorPathObjectCollisionAcrossPaths(
    const std::vector<BehaviorSafetyPose> &ego_path,
    const BehaviorSafetyFootprint &ego_vehicle,
    const BehaviorSafetyObject &object,
    const std::vector<BehaviorSafetyPredictedPath> &predicted_paths,
    bool use_all_predicted_paths,
    const BehaviorSafetyRssParameters &parameters = {},
    const BehaviorSafetyRegions &collision_regions = {},
    const std::vector<BehaviorSafetyPose> &planned_path = {});

}  // planning 네임스페이스
