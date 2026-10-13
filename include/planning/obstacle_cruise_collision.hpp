#pragma once

// 순항 장애물의 예측 외형과 자차 한 단계 주행 영역의 충돌점 계산입니다.

#include "planning/obstacle_cruise_candidate.hpp"
#include "planning/trajectory.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace planning {

struct CruiseObjectShape {
  // 0은 직육면체, 1은 원기둥, 2는 외곽 다각형입니다.
  int type = 0;
  double length_m = 0.0;
  double width_m = 0.0;
  std::vector<CruisePredictedPose> footprint;
};

struct CruiseTimedPose {
  CruisePredictedPose pose;
  double time_from_object_stamp_s = 0.0;
};

// 원본 예측 경로와 같이 XY 자연 3차 스플라인·방향 구면 보간으로 재표본화합니다.
std::vector<CruiseTimedPose> resampleCruisePredictedPath(
    const CruisePredictedPath &path,
    double interval_s = 0.1, double horizon_s = 10.0);

struct CruiseCollisionPoint {
  double x = 0.0;
  double y = 0.0;
  double time_from_object_stamp_s = 0.0;
  std::size_t trajectory_index = 0;
};

struct CruiseStopCollisionPoint {
  double x = 0.0;
  double y = 0.0;
  double distance_from_start_m = 0.0;
  std::size_t trajectory_index = 0;
};

struct CruiseSlowdownCollisionInterval {
  double front_arc_m = 0.0;
  double back_arc_m = 0.0;
  CruisePredictedPose front_point;
  CruisePredictedPose back_point;
};

// 자차 위치 뒤를 자른 경로를 Akima 방식으로 일정 간격 보간하고 끝을 연장합니다.
Trajectory resampleCruiseCollisionTrajectory(
    const Trajectory &trajectory, std::size_t start_index,
    double interval_m = 2.0, double extension_m = 6.0);

// 곡선에서 객체까지의 직선 진입 가능 거리를 이용해 정지 여유를 줄입니다.
double cruiseCurveStopMargin(
    const Trajectory &trajectory,
    const CruisePredictedPose &collision_point,
    const CruisePredictedPose &object_pose,
    const CruiseObjectShape &shape,
    double vehicle_width_m, double vehicle_front_m,
    double safe_margin_m, double minimum_margin_m,
    double additional_margin_m);

// 현재 장애물 외형에서 자차 주행 다각형까지의 최소 간격입니다.
double cruiseObjectLateralDistance(
    const Trajectory &decimated_trajectory,
    const CruisePredictedPose &object_pose,
    const CruiseObjectShape &shape,
    double vehicle_front_m, double vehicle_rear_m, double vehicle_width_m,
    std::optional<CruisePredictedPose> current_ego_pose = std::nullopt,
    double interval_m = 2.0, double time_to_convergence_s = 1.5);

// 현재 객체와 처음 겹치는 외형 교점에서 차량 앞부분 기준 정지 거리를 구합니다.
std::optional<CruiseStopCollisionPoint> findCruiseCurrentStopCollision(
    const Trajectory &decimated_trajectory,
    const CruisePredictedPose &object_pose,
    const CruiseObjectShape &shape,
    double vehicle_front_m, double vehicle_rear_m, double vehicle_width_m,
    double lateral_margin_m, bool driving_forward = true,
    std::optional<CruisePredictedPose> current_ego_pose = std::nullopt,
    double interval_m = 2.0, double time_to_convergence_s = 1.5);

// 확대된 차량 주행 영역과 현재 객체의 첫·마지막 교차 호길이를 구합니다.
std::optional<CruiseSlowdownCollisionInterval>
findCruiseSlowdownCollisionInterval(
    const Trajectory &decimated_trajectory,
    const CruisePredictedPose &object_pose,
    const CruiseObjectShape &shape,
    double vehicle_front_m, double vehicle_rear_m, double vehicle_width_m,
    double lateral_margin_m,
    std::optional<CruisePredictedPose> current_ego_pose = std::nullopt,
    double interval_m = 2.0, double time_to_convergence_s = 1.5);

// 입력 경로는 원본처럼 충돌 검사 간격으로 축소한 경로를 받습니다.
// 원본의 현재 자차 자세 오차 추가는 호출자가 경로 자세에 반영해야 합니다.
std::vector<CruiseCollisionPoint> findCruiseCollisionPoints(
    const Trajectory &decimated_trajectory,
    const std::vector<CruiseTimedPose> &prediction,
    const CruiseObjectShape &shape,
    double vehicle_front_m, double vehicle_rear_m, double vehicle_width_m,
    double current_elapsed_s, double maximum_prediction_s,
    double maximum_distance_m, bool driving_forward = true,
    std::optional<CruisePredictedPose> current_ego_pose = std::nullopt,
    double interval_m = 2.0, double time_to_convergence_s = 1.5);

}  // planning 네임스페이스
