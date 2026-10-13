#pragma once

// 돌발 진입의 예측 경로 없는 객체와 점군 입력을 동적 객체로 변환합니다.

#include "planning/run_out_collision.hpp"

#include <string>
#include <vector>

namespace planning {

enum class RunOutDetectionMode { Object, ObjectWithoutPath, Points };

struct RunOutPoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct RunOutCreatorParameters {
  double minimum_velocity_mps = 0.0;
  double maximum_velocity_mps = 5.0 / 3.6;
  double prediction_step_s = 0.5;
  double prediction_horizon_s = 7.0;
  double point_diameter_m = 0.1;
  double point_path_interval_m = 0.1;
  double voxel_size_m = 0.05;
};

// 객체의 방향을 가까운 경로 쪽으로 바꾸고 일정 속도 직선 예측을 만듭니다.
std::vector<RunOutCollisionObject> createRunOutObjectsWithoutPath(
    const Trajectory &path,
    const std::vector<RunOutCollisionObject> &objects,
    const RunOutCreatorParameters &parameters = {});

// 점군은 지도 좌표계 변환을 마친 점을 받으며 검출 다각형 안의 점만 사용합니다.
std::vector<RunOutCollisionObject> createRunOutObjectsFromPoints(
    const Trajectory &path, const std::vector<RunOutPoint> &points,
    const std::vector<std::vector<CruisePredictedPose>> &detection_polygons,
    const RunOutCreatorParameters &parameters = {},
    const std::string &id_prefix = "point");

}  // planning 네임스페이스
