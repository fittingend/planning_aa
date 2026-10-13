// SPDX-License-Identifier: Apache-2.0
// 행동 경로 표본 계획기의 Frenet 후보를 ROS 없이 생성합니다.

#pragma once

#include "planning/trajectory.hpp"

#include <optional>
#include <utility>
#include <vector>

namespace planning {

struct BehaviorFrenetState {
  double s_m = 0.0;
  double lateral_m = 0.0;
  double lateral_derivative = 0.0;
  double lateral_second_derivative = 0.0;
};

struct BehaviorFrenetSamplingInput {
  Trajectory reference;
  BehaviorFrenetState initial;
  std::vector<double> target_lengths_m{20.0, 40.0};
  std::vector<double> target_lateral_positions_m{-4.5, -2.5, 0.0, 2.5, 4.5};
  std::vector<double> target_lateral_derivatives{-0.5, 0.0, 0.5};
  std::vector<double> target_lateral_second_derivatives{0.0};
  double resolution_m = 0.5;
};

struct BehaviorFrenetCandidate {
  Trajectory path;
  std::vector<double> arc_lengths_m;
  std::vector<double> curvatures_per_m;
  double target_s_m = 0.0;
  double target_lateral_m = 0.0;
};

// 기준 스플라인의 최근접 호길이와 자세 차이를 Frenet 초기 상태로 바꿉니다.
std::optional<BehaviorFrenetState> projectBehaviorFrenetState(
    const Trajectory &reference, const TrajectoryPoint &pose);

// 원본 표본 계획기의 목표 길이·측방 위치·도함수 조합을 순서대로 생성합니다.
std::vector<BehaviorFrenetCandidate> generateBehaviorFrenetCandidates(
    const BehaviorFrenetSamplingInput &input);

}  // planning 네임스페이스
