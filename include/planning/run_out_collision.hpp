#pragma once

// 돌발 진입 모듈의 예측 객체와 차량 주행 외형 교차를 ROS 없이 계산합니다.

#include "planning/obstacle_cruise_candidate.hpp"
#include "planning/trajectory.hpp"

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace planning {

struct RunOutCollisionObject {
  std::string id;
  CruisePredictedPose pose;
  std::vector<CruisePredictedPath> predicted_paths;
  // 분류 확률이 가장 높은 원본 객체 종류를 전달합니다.
  int classification = 7;
  // 0은 직육면체, 1은 원기둥입니다. 원본의 다각형 외형은 충돌 대상에서 제외됩니다.
  int shape_type = 0;
  double length_m = 0.0;
  double width_m = 0.0;
  double minimum_velocity_mps = 0.0;
  double maximum_velocity_mps = 0.0;
};

struct RunOutCollisionParameters {
  double minimum_ego_velocity_mps = 1.0;
  double detection_span_m = 1.0;
  double detection_distance_m = 45.0;
  double maximum_prediction_time_s = 7.0;
  std::vector<int> target_classifications{5, 6, 7};
  double vehicle_front_m = 3.79;
  double vehicle_rear_m = 1.10;
  double vehicle_left_m = 0.948;
  double vehicle_right_m = 0.948;
  double passing_margin_m = 1.1;
  // 충돌 교점이 이 다각형 안에 있으면 횡단보도 억제 설정에 따라 제외합니다.
  std::vector<std::vector<CruisePredictedPose>> suppressed_polygons;
};

struct RunOutObjectFilterParameters {
  struct PartitionSegment {
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
  };
  std::vector<PartitionSegment> partition_segments;
  double partition_search_distance_m = 45.0;
  bool exclude_crossing_ego_cut_line = true;
  double ego_cut_line_length_m = 3.0;
  bool exclude_objects_already_in_path = true;
  double vehicle_width_m = 1.896;
  double path_footprint_extra_margin_m = 0.5;
  double keep_on_path_s = 1.0;
};

class RunOutObjectFilterCore {
 public:
  std::vector<RunOutCollisionObject> filter(
      const Trajectory &path, const CruisePredictedPose &ego_pose,
      double now_s, const std::vector<RunOutCollisionObject> &objects,
      const RunOutObjectFilterParameters &parameters = {});
  void reset();

 private:
  std::unordered_map<std::string, double> first_inside_path_s_;
};

struct RunOutVelocityRange {
  double minimum_mps = 0.0;
  double maximum_mps = 0.0;
};

// 원본 객체 변환기의 XY 속도 공분산에서 속도 범위를 계산합니다.
RunOutVelocityRange runOutVelocityRange(
    double velocity_x_mps, double velocity_y_mps,
    double variance_x, double variance_y,
    double standard_deviation_multiplier = 1.96);

// 원본처럼 경로 끝을 차량 전면 길이만큼 연장하고 자차 앞 45m를 남깁니다.
Trajectory prepareRunOutCollisionPath(
    const Trajectory &path, double ego_x, double ego_y,
    double extension_m = 3.79, double detection_distance_m = 45.0);

// JerkFiltered를 가정한 비교용 경로 평활화입니다.
// QP가 풀리지 않으면 빈 결과를 반환합니다.
std::optional<Trajectory> smoothRunOutCollisionPath(
    const Trajectory &path, const CruisePredictedPose &ego_pose,
    double ego_velocity_mps, double ego_acceleration_mps2,
    std::optional<double> external_maximum_velocity_mps = std::nullopt);

// 실차 설정의 Analytical 스무더를 돌발 진입 경로에 적용합니다.
// 전방 분기만 시험할 때는 역방향 감속을 끌 수 있습니다.
Trajectory smoothRunOutCollisionPathAnalytical(
    const Trajectory &path, const CruisePredictedPose &ego_pose,
    double ego_velocity_mps, double ego_acceleration_mps2,
    std::optional<double> external_maximum_velocity_mps = std::nullopt,
    bool include_deceleration = true);

struct RunOutCollision {
  std::string id;
  double collision_arc_m = 0.0;
  double travel_time_s = 0.0;
  double collision_x = 0.0;
  double collision_y = 0.0;
  double distance_from_vehicle_front_m = 0.0;
};

// 가장 높은 신뢰도의 예측 경로를 객체 속도와 주행 시간으로 보간합니다.
std::optional<CruisePredictedPose> predictRunOutObjectPose(
    const std::vector<CruisePredictedPath> &paths, double travel_time_s,
    double velocity_mps);

// 차량이 경로의 각 검사점에 도달할 때 최소·최대 속도 객체 영역의 첫 교점을 찾습니다.
std::optional<RunOutCollision> findRunOutCollision(
    const Trajectory &path, const std::vector<RunOutCollisionObject> &objects,
    const RunOutCollisionParameters &parameters = {});

}  // planning 네임스페이스
