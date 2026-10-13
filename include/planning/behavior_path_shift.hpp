// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 행동 경로 모듈의 공통 횡이동 곡선과 점별 이동량을 ROS 없이 계산합니다.

#pragma once

#include "planning/trajectory.hpp"

#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace planning {

struct BehaviorPathShiftLine {
  std::size_t start_index = 0;
  std::size_t end_index = 0;
  double end_shift_m = 0.0;
};

struct BehaviorPathShiftAnchor {
  double x = 0.0;
  double y = 0.0;
};

BehaviorPathShiftLine behaviorShiftLineFromPositions(
    const Trajectory &reference, BehaviorPathShiftAnchor start,
    BehaviorPathShiftAnchor end, double end_shift_m);

enum class BehaviorPathShiftType { Linear, Spline };

struct BehaviorPathShiftParameters {
  BehaviorPathShiftType type = BehaviorPathShiftType::Spline;
  bool offset_back = true;
  double base_offset_m = 0.0;
  double velocity_mps = 0.0;
  double longitudinal_acceleration_mps2 = 0.0;
  double lateral_acceleration_limit_mps2 = 1.0;
};

struct BehaviorPathShiftResult {
  Trajectory path;
  std::vector<double> shift_lengths_m;
};

// 원본 PathShifter의 횡이동 곡선 생성 단계입니다. 점 정리·방향 재계산은 별도 단계입니다.
BehaviorPathShiftResult shiftBehaviorPath(
    const Trajectory &reference,
    const std::vector<BehaviorPathShiftLine> &lines,
    const BehaviorPathShiftParameters &parameters = {});

// 이동 뒤 중복점과 역방향 첫 점을 지우고 진행 방향 자세를 다시 계산합니다.
Trajectory finalizeBehaviorPathShift(const Trajectory &shifted_path);

Trajectory generateBehaviorShiftedPath(
    const Trajectory &reference,
    const std::vector<BehaviorPathShiftLine> &lines,
    const BehaviorPathShiftParameters &parameters = {});

struct BehaviorSideShiftParameters {
  double minimum_start_distance_m = 5.0;
  double start_delay_s = 1.0;
  double lateral_jerk_mps3 = 0.2;
  double minimum_shift_distance_m = 5.0;
  double minimum_shift_speed_mps = 5.56;
  double request_time_limit_s = 1.0;
};

struct BehaviorSideShiftResult {
  Trajectory path;
  BehaviorPathShiftLine line;
};

// 원본 SideShift의 자차 속도·저크 기반 이동 구간을 경로 인덱스로 계산합니다.
std::optional<BehaviorSideShiftResult> generateBehaviorSideShiftPath(
    const Trajectory &reference, std::size_t ego_index,
    double ego_speed_mps, double current_shift_m,
    double requested_shift_m,
    const BehaviorSideShiftParameters &parameters = {});

// 자차를 지난 이동 구간을 지우고 원본 비교자로 고른 이동량을 기본 오프셋으로 넘깁니다.
void trimBehaviorPathShiftLines(
    std::vector<BehaviorPathShiftLine> &lines, std::size_t nearest_index,
    double &base_offset_m);

// 출발 모듈의 종가속도·곡률 제한에서 요구하는 횡이동 거리입니다.
double requiredStartPullOutDistance(
    double longitudinal_acceleration_mps2, double shift_time_s,
    double shift_length_m, double maximum_curvature,
    double minimum_distance_m);

// 원본의 저크·횡가속도 제한에서 횡이동 시간을 계산합니다.
double startPullOutShiftTime(
    double lateral_shift_m, double lateral_jerk_mps3,
    double lateral_acceleration_mps2);

// 차로 중심선의 곡률을 반영해 횡이동 후 목표 호길이를 이동 전 호길이로 바꿉니다.
double startPullOutDistanceBeforeShift(
    const Trajectory &centerline, double after_shift_arc_m,
    double lateral_shift_m);

struct StartPullOutParameters {
  double lateral_jerk_mps3 = 0.5;
  double minimum_lateral_acceleration_mps2 = 0.15;
  double maximum_lateral_acceleration_mps2 = 0.5;
  int lateral_acceleration_samples = 3;
  double maximum_curvature = 0.07;
  double end_curvature_threshold = 0.1;
  double minimum_distance_m = 0.0;
  double centerline_interval_m = 1.0;
};

struct StartPullOutCandidate {
  Trajectory path;
  std::size_t shift_end_index = 0;
  double terminal_velocity_mps = 0.0;
  double longitudinal_acceleration_mps2 = 0.0;
  double lateral_acceleration_mps2 = 0.0;
  bool shifted = false;
};

// 지도에서 이미 추출·재표본화한 중심선의 출발 횡이동 후보를 계산합니다.
// 차로 이탈·장애물 충돌과 경로 뒤쪽 자르기는 호출자가 판단합니다.
std::vector<StartPullOutCandidate> generateStartPullOutCandidates(
    const Trajectory &centerline, BehaviorPathShiftAnchor start,
    double start_lateral_offset_m, bool path_terminal_is_goal,
    const StartPullOutParameters &parameters = {});

struct LaneChangeShiftCandidate {
  Trajectory path;
  Trajectory shifted_target;
  std::size_t shift_end_index = 0;
};

// 제공된 준비 경로와 목표 차로 경로를 횡이동으로 잇습니다.
std::optional<LaneChangeShiftCandidate> generateLaneChangeShiftCandidate(
    const Trajectory &prepare_path, const Trajectory &target_reference,
    BehaviorPathShiftAnchor change_start,
    BehaviorPathShiftAnchor change_end, double lateral_shift_m,
    double initial_velocity_mps, double longitudinal_acceleration_mps2,
    double lateral_acceleration_mps2, double terminal_velocity_mps);

struct LaneChangePhaseMetrics {
  double duration_s = 0.0;
  double length_m = 0.0;
  double terminal_velocity_mps = 0.0;
  double sampled_longitudinal_acceleration_mps2 = 0.0;
  double actual_longitudinal_acceleration_mps2 = 0.0;
  double lateral_acceleration_mps2 = 0.0;
};

struct LaneChangeTargetReferenceArcRange {
  double start_m = 0.0;
  double end_m = 0.0;
};

// 목표 차로의 끝 또는 목표점 앞 여유를 적용해 중심선 추출 구간을 정합니다.
// 목표점이 현재 목표 차로에 없으면 goal_arc_m을 비웁니다.
std::optional<LaneChangeTargetReferenceArcRange>
laneChangeTargetReferenceArcRange(
    double change_start_arc_m, double change_length_m,
    double forward_path_length_m, double target_lane_length_m,
    double next_change_buffer_m, std::optional<double> goal_arc_m);

// 원본 목표 중심선의 최소 30점·0.2초 주행 간격 중 큰 값을 사용합니다.
double laneChangeTargetReferenceInterval(
    double change_length_m, double change_velocity_mps);

// 원본 차로 변경기의 0 포함·내림차순 종가속도 표본을 계산합니다.
std::vector<double> sampleLaneChangeAccelerations(
    double minimum_mps2, double maximum_mps2, int intervals);

// 최소 변경 속도와 방향 지시등 활성 시간을 고려한 실제 준비 시간입니다.
double laneChangeActualPrepareDuration(
    double current_velocity_mps, double minimum_change_velocity_mps,
    double maximum_longitudinal_acceleration_mps2,
    double minimum_prepare_duration_s, double maximum_prepare_duration_s,
    double active_signal_duration_s);

// 차로 종단 가까이에서는 준비 시간을 0.5초 간격으로 다시 표본 추출합니다.
std::vector<double> laneChangePrepareDurations(
    double distance_to_terminal_start_m,
    double front_offset_m, double signal_activation_length_m,
    double nominal_prepare_duration_s, double maximum_prepare_duration_s);

bool laneChangeNeedsAccelerationSampling(
    double maximum_acceleration_mps2, bool ego_stuck,
    double maximum_current_buffer_m, double distance_to_terminal_end_m,
    double distance_to_target_end_m);

// 준비 구간의 속도 상한, 가속도 표본과 주행 거리를 계산합니다.
std::vector<LaneChangePhaseMetrics> generateLaneChangePreparePhases(
    double current_velocity_mps, double maximum_path_velocity_mps,
    double maximum_vehicle_velocity_mps, double minimum_change_velocity_mps,
    double prepare_duration_s, double minimum_acceleration_mps2,
    double maximum_acceleration_mps2, int acceleration_intervals,
    bool sample_acceleration, double minimum_length_m,
    double maximum_length_m);

// 횡이동 구간의 저크 시간, 종가속도와 거리 후보를 계산합니다.
std::vector<LaneChangePhaseMetrics> generateLaneChangeShiftPhases(
    double lateral_shift_m, double initial_velocity_mps,
    double maximum_path_velocity_mps, double maximum_vehicle_velocity_mps,
    double minimum_change_velocity_mps,
    double prepare_acceleration_mps2, double minimum_lateral_acceleration_mps2,
    double maximum_lateral_acceleration_mps2, int lateral_intervals,
    double lateral_jerk_mps3, double maximum_length_m,
    bool near_current_terminal_start, double deceleration_factor);

// 마지막으로 실제 생성된 후보의 두 구간 길이에 대해 원본의 엄격한 중복 제거를 적용합니다.
class LaneChangeCandidateLengthGate {
 public:
  LaneChangeCandidateLengthGate(double prepare_threshold_m,
                                double shift_threshold_m);

  bool acceptsPrepare(double prepare_length_m) const;
  bool acceptsShift(double prepare_length_m, double shift_length_m) const;
  void recordConstructed(double prepare_length_m, double shift_length_m);

 private:
  double prepare_threshold_m_ = 0.0;
  double shift_threshold_m_ = 0.0;
  std::optional<std::pair<double, double>> last_constructed_lengths_m_;
};

}  // planning 네임스페이스
