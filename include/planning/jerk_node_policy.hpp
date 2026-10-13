// Copyright 2021 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 속도 스무더 노드의 정지점 후처리입니다.

#pragma once

#include "planning/trajectory.hpp"
#include "planning/jerk_resample.hpp"

#include <cstddef>
#include <optional>

namespace planning {

struct StopPolicyResult {
  bool stop_found = false;
  bool stop_output_found = false;
  float velocity_before_enforcement = 0.0F;
};

// 입력의 첫 정지점에 가장 가까운 출력점부터 속도를 0으로 제한합니다.
StopPolicyResult enforceJerkNodeStop(
    const Trajectory &input, Trajectory &output, double maximum_distance,
    double maximum_yaw);

// 정지점 앞의 지정 거리에서 목표 속도를 제한합니다.
void limitVelocityApproachingStop(
    Trajectory &trajectory, double stopping_velocity,
    double stopping_distance);

// 외부 제한 요청이 없을 때의 기본 최고 속도와 정지 접근 속도를 적용합니다.
Trajectory limitJerkNodeInputVelocity(
    const Trajectory &input, double maximum_velocity,
    double stopping_velocity, double stopping_distance);

struct JerkNodeOutputParameters {
  double max_velocity = 4.17;
  double nearest_distance = 3.0;
  double nearest_yaw = 1.046;
  double extract_ahead_distance_m = 200.0;
  double extract_behind_distance_m = 5.0;
  bool keep_closest_motion_behind = false;
  // 기록된 실행 파일의 일시 정지 후 재출발 보간을 재현할 때만 켭니다.
  bool preserve_velocity_restarts = false;
  JerkResampleParameters post_resampling;
};

// 첫 입력에서 최적화 결과에 정지점, 후방 속도, 최고 속도와 출력 재표본화를 적용합니다.
// internal_output에는 원본 노드가 다음 입력을 위해 저장하는 재표본화 전 궤적을 돌려줍니다.
Trajectory finalizeFirstJerkNodeTrajectory(
    const Trajectory &node_resampled_input, std::size_t closest_index,
    const Trajectory &optimized, const TrajectoryPoint &ego_pose,
    double ego_velocity, const JerkNodeOutputParameters &parameters,
    Trajectory *internal_output = nullptr);

// 차량 뒤쪽을 이전 내부 궤적으로 보간하거나 재계획 시 현재 최근접 상태로 채웁니다.
// previous_output에는 발행 궤적이 아닌 이전 프레임의 재표본화 전 궤적을 넣습니다.
Trajectory finalizeNextJerkNodeTrajectory(
    const Trajectory &node_resampled_input, std::size_t closest_index,
    const Trajectory &optimized, const Trajectory &previous_output,
    const TrajectoryPoint &ego_pose, double ego_velocity,
    const JerkNodeOutputParameters &parameters,
    Trajectory *internal_output = nullptr);

struct JerkMotion {
  double velocity = 0.0;
  double acceleration = 0.0;
};

// 이전 내부 궤적의 선분에 차량 위치를 투영해 보간한 운동 상태입니다.
JerkMotion projectJerkPreviousMotion(
    const Trajectory &previous, const TrajectoryPoint &ego_pose,
    double nearest_distance, double nearest_yaw);

enum class JerkInitialType {
  EgoVelocity,
  LargeDeviationReplan,
  Engaging,
  PreviousOutput
};

struct JerkInitialMotionParameters {
  double replan_velocity_deviation = 5.53;
  double engage_velocity = 0.25;
  double engage_acceleration = 0.5;
  double engage_exit_ratio = 0.5;
  double stop_distance_to_prohibit_engage = 0.5;
  double min_deceleration = -1.0;
  double max_acceleration = 1.0;
  bool plan_from_ego_on_manual_mode = true;
};

struct JerkInitialMotionResult {
  JerkMotion motion;
  JerkInitialType type = JerkInitialType::EgoVelocity;
};

// 이전 출력 상태와 운전 모드에 따라 원본 노드의 초기 운동 상태를 선택합니다.
JerkInitialMotionResult selectJerkInitialMotion(
    double vehicle_velocity, double vehicle_acceleration,
    double target_velocity, double distance_to_stop,
    bool autonomous_control_enabled,
    const std::optional<JerkMotion> &previous_desired,
    const JerkInitialMotionParameters &parameters,
    std::optional<double> external_engage_acceleration = std::nullopt);

// 원본 노드처럼 음수 속도가 하나라도 있으면 역방향 궤적으로 취급합니다.
bool isReverseJerkTrajectory(const Trajectory &trajectory);
void flipJerkTrajectoryVelocity(Trajectory &trajectory);

}  // planning 네임스페이스
