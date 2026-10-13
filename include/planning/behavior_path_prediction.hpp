// SPDX-License-Identifier: Apache-2.0
// 행동 경로의 차로 변경 준비·변경 구간 자차 예측 경로를 생성합니다.

#pragma once

#include "planning/behavior_path_object_safety.hpp"
#include "planning/trajectory.hpp"

#include <cstddef>
#include <vector>

namespace planning {

struct BehaviorLaneChangePredictionParameters {
  double ego_arc_m = 0.0;
  double initial_velocity_mps = 0.0;
  double prepare_duration_s = 0.0;
  double lane_change_duration_s = 0.0;
  double prepare_acceleration_mps2 = 0.0;
  double lane_change_acceleration_mps2 = 0.0;
  double prepare_velocity_limit_mps = 0.0;
  double lane_change_velocity_limit_mps = 0.0;
  double resolution_s = 0.5;
};

// 원본 차로 변경의 두 시간 구간을 경로 호길이로 보간합니다.
std::vector<BehaviorSafetyPose> predictBehaviorLaneChangeEgoPath(
    const Trajectory &path,
    const BehaviorLaneChangePredictionParameters &parameters);

// 원본의 감속 표본 개수·간격으로 안전 검사에 쓸 여러 자차 경로를 만듭니다.
std::vector<std::vector<BehaviorSafetyPose>>
predictBehaviorLaneChangeEgoPaths(
    const Trajectory &path,
    const BehaviorLaneChangePredictionParameters &parameters,
    double global_min_acceleration_mps2,
    std::size_t deceleration_sampling_num);

}  // planning 네임스페이스
