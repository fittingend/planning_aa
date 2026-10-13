#pragma once

// 장애물 순항 후보의 객체별 제외 조건입니다. 충돌점 계산 결과는 호출자가 제공합니다.

#include "planning/trajectory.hpp"

#include <cstddef>
#include <array>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace planning {

struct CruiseCandidateParameters {
  // ObstacleCruisePlanner LongitudinalInfo::safe_distance_margin is shared by
  // cruise and obstacle-stop planning; the latter must not fix it at 4 m.
  double stop_safe_distance_margin_m = 4.0;
  double maximum_lateral_margin_m = 1.0;
  double maximum_lateral_time_s = 5.0;
  double crossing_angle_threshold_rad = 0.523599;
  double minimum_outside_speed_mps = 1.5;
  double minimum_outside_overlap_s = 0.2;
  double minimum_ego_speed_for_outside_mps = 0.1;
  // behavior_determination.cruise.outside_obstacle.num_of_predicted_paths
  std::size_t outside_predicted_path_count = 3;
  // behavior_determination.obstacle_velocity_threshold_from_stop_to_cruise
  // and obstacle_velocity_threshold_from_cruise_to_stop, respectively.
  double inside_cruise_entry_speed_mps = 1.5;
  double inside_cruise_keep_speed_mps = 1.0;
  // behavior_determination.stop.crossing_obstacle.collision_time_margin
  double stop_collision_time_margin_s = 1.0;
  // behavior_determination.stop.outside_obstacle.* (Obigo launch defaults).
  double outside_stop_lateral_time_s = 4.5;
  // Obigo decision launch에서는 모든 차로 밖 정지 종류가 false입니다.
  // 패키지 기본 YAML을 시험할 때에는 필요한 종류를 명시적으로 켭니다.
  std::array<bool, 8> outside_stop_classes{};
  std::size_t outside_stop_predicted_path_count = 1;
  double pedestrian_deceleration_mps2 = 0.5;
  double bicycle_deceleration_mps2 = 0.5;
};

struct CruiseObjectKinematics {
  double x = 0.0;
  double y = 0.0;
  double yaw_rad = 0.0;
  double local_velocity_x = 0.0;
  double local_velocity_y = 0.0;
};

struct CruisePredictedPose {
  double x = 0.0;
  double y = 0.0;
  double yaw_rad = 0.0;
};

struct CruisePredictedPath {
  std::vector<CruisePredictedPose> poses;
  double time_step_s = 0.0;
  double confidence = 0.0;
};

// 가장 높은 신뢰도의 예측 경로로 현재 자세를 보간하고, 불가능하면 최초 자세를 돌려줍니다.
CruisePredictedPose currentCruiseObjectPose(
    const CruisePredictedPose &initial_pose,
    const std::vector<CruisePredictedPath> &predicted_paths,
    double elapsed_s);

struct CruiseObjectProjection {
  double distance_from_ego_m = 0.0;
  double signed_lateral_offset_m = 0.0;
  double heading_difference_rad = 0.0;
  double longitudinal_velocity_mps = 0.0;
  double approach_velocity_mps = 0.0;
};

// 원본 객체 변환기의 최근접 경로점과 경로 상대 속도 계산입니다.
// 객체 자세는 호출자가 현재 계획 시각으로 보정해 전달합니다.
std::optional<CruiseObjectProjection> projectCruiseObjectToTrajectory(
    const Trajectory &trajectory, std::size_t ego_index,
    const CruiseObjectKinematics &object);

// 원본 객체 변환기의 넓은 횡거리·접근 시간 필터입니다.
bool passesObstacleCruiseRoughLateralFilter(
    const CruiseObjectProjection &projection,
    double object_max_length_m, double vehicle_width_m,
    double maximum_lateral_margin_m = 1.1,
    double maximum_lateral_time_s = 5.0);

struct CruiseCandidateInput {
  bool supported_class = false;
  bool outside_class = false;
  bool driving_forward = true;
  double ego_speed_mps = 0.0;
  double longitudinal_speed_mps = 0.0;
  double approach_speed_mps = 0.0;
  double lateral_distance_m = 0.0;
  double heading_difference_rad = 0.0;
  double object_speed_mps = 0.0;
  bool has_collision_points = false;
  double collision_overlap_s = 0.0;
  bool first_collision_in_front = true;
};

enum class CruiseCandidateRejection {
  None,
  UnsupportedClass,
  ReverseMotion,
  OppositeDirection,
  TooFarLaterally,
  CrossingTrajectory,
  OutsideWhileEgoStopped,
  OutsideUnsupportedClass,
  OutsideTooSlow,
  NoCollision,
  OutsideShortOverlap,
  OutsideBehindEgo,
};

CruiseCandidateRejection classifyCruiseCandidate(
    const CruiseCandidateInput &input,
    const CruiseCandidateParameters &parameters = {});

// 원본 순항 판단의 이전 주기 객체 ID별 진입·유지 속도 임계값입니다.
class CruiseInsideHysteresis {
 public:
  CruiseInsideHysteresis(double entry_speed_mps = 1.5,
                        double keep_speed_mps = 1.0);

  bool permits(const std::string &id, double longitudinal_speed_mps) const;
  void replacePreviousCruiseIds(const std::vector<std::string> &ids);
  void reset();

 private:
  double entry_speed_mps_;
  double keep_speed_mps_;
  std::unordered_set<std::string> previous_ids_;
};

struct CruiseYieldObject {
  std::string id;
  bool outside_cruise_class = false;
  bool crossing_trajectory = false;
  double x = 0.0;
  double y = 0.0;
  double local_velocity_x_mps = 0.0;
  double local_velocity_y_mps = 0.0;
  double longitudinal_velocity_mps = 0.0;
  double lateral_distance_m = 0.0;
  double distance_from_ego_m = 0.0;
  double heading_difference_rad = 0.0;
};

struct CruiseYieldParameters {
  double stopped_velocity_threshold_mps = 0.5;
  double lateral_distance_threshold_m = 5.0;
  double maximum_lateral_gap_m = 2.5;
  double maximum_collision_time_s = 10.0;
  double outside_minimum_speed_mps = 1.5;
  double vehicle_length_m = 4.77;
};

struct CruiseYieldPair {
  std::size_t moving_index = 0;
  std::size_t stopped_index = 0;
};

// 인접 차로 이동 객체가 앞 정차 객체에 접근할 때 양보 순항 쌍을 고릅니다.
std::vector<CruiseYieldPair> findCruiseYieldPairs(
    const std::vector<CruiseYieldObject> &objects,
    const CruiseYieldParameters &parameters = {});

}  // planning 네임스페이스
