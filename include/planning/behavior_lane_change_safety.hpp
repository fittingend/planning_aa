// SPDX-License-Identifier: Apache-2.0
// 차로 변경 후보의 후방·전방 객체와 자차 감속 경로별 충돌을 결합합니다.

#pragma once

#include "planning/behavior_path_object_safety.hpp"
#include "planning/behavior_path_prediction.hpp"
#include "planning/behavior_path_shift.hpp"

#include <cstddef>
#include <optional>
#include <string_view>
#include <vector>

namespace planning {

struct BehaviorLaneChangeTarget {
  BehaviorSafetyObject object;
  std::vector<BehaviorSafetyPredictedPath> predicted_paths;
  double initial_velocity_mps = 0.0;
};

enum class BehaviorLaneChangeObjectGroup {
  TargetLeadingMoving,
  TargetLeadingStopped,
  TargetLeadingStoppedAtBoundary,
  TargetTrailing,
  CurrentLeading,
  Other,
};

struct BehaviorLaneChangeObjectFilterInput {
  BehaviorSafetyPolygon initial_polygon;
  std::vector<BehaviorSafetyPredictedPath> predicted_paths;
  BehaviorSafetyPolygon current_lane;
  BehaviorSafetyPolygon target_lane;
  BehaviorSafetyPolygon expanded_target_lane;
  BehaviorSafetyRegions preceding_target_lanes;
  double object_lateral_from_current_center_m = 0.0;
  double ego_lateral_from_current_center_m = 0.0;
  double vehicle_width_m = 0.0;
  // 객체 초기 Twist의 x·y 성분으로 계산한 속력 크기입니다.
  double initial_velocity_mps = 0.0;
  double stopped_velocity_threshold_mps = 1.0;
  double ego_yaw_rad = 0.0;
  double object_yaw_rad = 0.0;
  double incoming_yaw_threshold_rad = 2.3562;
  bool type_allowed = true;
  bool is_vehicle = false;
  bool ahead_of_ego = false;
  bool before_terminal = false;
};

// 차로 경계와 종·횡방향 위치가 준비된 객체를 원본의 검사 집합으로 분류합니다.
BehaviorLaneChangeObjectGroup classifyBehaviorLaneChangeObject(
    const BehaviorLaneChangeObjectFilterInput &input);

struct BehaviorLaneChangeSelectableObject {
  BehaviorLaneChangeTarget target;
  BehaviorLaneChangeObjectFilterInput filter;
  double signed_distance_from_ego_m = 0.0;
};

struct BehaviorLaneChangeSelectedTargets {
  std::vector<BehaviorLaneChangeTarget> leading;
  std::vector<BehaviorLaneChangeTarget> trailing;
};

// 분류 집합 안에서는 전방 거리순, 후방은 역순으로 안전 검사 입력을 만듭니다.
BehaviorLaneChangeSelectedTargets selectBehaviorLaneChangeTargets(
    const std::vector<BehaviorLaneChangeSelectableObject> &objects,
    bool check_current_lane, bool check_other_lanes, bool ego_stuck);

struct BehaviorLaneChangeSafetyParameters {
  double prepare_duration_s = 0.0;
  bool check_prepare_phase = false;
  double stopped_object_velocity_threshold_mps = 1.0;
  bool use_all_predicted_paths = false;
  BehaviorSafetyFootprint ego_vehicle;
  BehaviorSafetyRssParameters moving_object;
  // 지정 시 두 차로 영역 모두 있어야 객체 충돌을 유효하게 취급합니다.
  bool check_lane_regions = false;
  BehaviorSafetyPolygon current_lane_region;
  BehaviorSafetyPolygon expanded_target_lane_region;
  BehaviorSafetyRssParameters parked_object = [] {
    BehaviorSafetyRssParameters value;
    value.rear_deceleration_mps2 = -2.0;
    value.reaction_time_s = 1.0;
    value.safety_time_margin_s = 0.8;
    value.lateral_margin_m = 1.0;
    return value;
  }();
};

struct BehaviorLaneChangeSafetyResult {
  bool safe = true;
  bool object_behind_ego = false;
  std::size_t ego_path_index = 0;
  std::size_t object_index = 0;
  std::optional<BehaviorSafetyCollision> collision;
};

// 모든 감속 예측 경로가 충돌하면 후방, 다음으로 전방 객체를 검사합니다.
BehaviorLaneChangeSafetyResult evaluateBehaviorLaneChangeObjectSafety(
    const std::vector<std::vector<BehaviorSafetyPose>> &ego_paths,
    const std::vector<BehaviorLaneChangeTarget> &trailing,
    const std::vector<BehaviorLaneChangeTarget> &leading,
    const BehaviorLaneChangeSafetyParameters &parameters = {});

struct BehaviorLaneChangeCandidateSafetyInput {
  BehaviorLaneChangePredictionParameters prediction;
  double global_min_acceleration_mps2 = 0.0;
  std::size_t deceleration_sampling_num = 1;
  std::vector<BehaviorLaneChangeSelectableObject> objects;
  bool check_current_lane = false;
  bool check_other_lanes = false;
  bool ego_stuck = false;
  BehaviorLaneChangeSafetyParameters safety;
};

// 한 후보 경로에서 감속 예측·객체 선별·경로별 안전 판정을 이어 실행합니다.
BehaviorLaneChangeSafetyResult evaluateBehaviorLaneChangeCandidateSafety(
    const Trajectory &candidate_path,
    const BehaviorLaneChangeCandidateSafetyInput &input);

struct BehaviorLaneChangeShiftPlan {
  LaneChangePhaseMetrics phase;
  Trajectory target_reference;
  BehaviorPathShiftAnchor change_end;
  double lateral_shift_m = 0.0;
};

struct BehaviorLaneChangePreparePlan {
  LaneChangePhaseMetrics phase;
  Trajectory prepare_path;
  std::vector<BehaviorLaneChangeShiftPlan> shifts;
};

struct BehaviorLaneChangeCandidateSearchInput {
  std::vector<BehaviorLaneChangePreparePlan> prepare_plans;
  double prepare_length_threshold_m = 0.0;
  double shift_length_threshold_m = 0.0;
  BehaviorLaneChangeCandidateSafetyInput safety;
};

struct BehaviorLaneChangeConstructedCandidate {
  LaneChangeShiftCandidate path;
  LaneChangePhaseMetrics prepare_phase;
  LaneChangePhaseMetrics shift_phase;
  BehaviorLaneChangeSafetyResult safety;
};

struct BehaviorLaneChangeCandidateSearchResult {
  std::vector<BehaviorLaneChangeConstructedCandidate> candidates;
  std::optional<std::size_t> first_safe_index;
};

// 준비·횡이동 표본을 원본 순서로 구성하고 안전한 첫 후보를 찾습니다.
// 지도에서 자른 준비·목표 경로와 객체 위치 관계는 호출자가 제공합니다.
BehaviorLaneChangeCandidateSearchResult searchBehaviorLaneChangeCandidates(
    const BehaviorLaneChangeCandidateSearchInput &input);

enum class BehaviorLaneChangeState {
  Normal,
  Cancel,
  Abort,
  Stop,
  Warning,
};

struct BehaviorLaneChangeFailureInput {
  BehaviorLaneChangeState current_state = BehaviorLaneChangeState::Normal;
  bool force_activated = false;
  bool abort_finished = false;
  bool waiting_approval = false;
  bool near_regulatory_element = false;
  bool path_valid = true;
  bool preparing = false;
  bool can_return_current_lane = false;
  bool force_deactivated = false;
  bool post_process_safe = true;
  bool cancel_enabled = true;
  bool near_terminal = false;
  bool abort_enabled = true;
  bool abort_path_available = false;
  bool stop_required = false;
};

struct BehaviorLaneChangeFailureDecision {
  BehaviorLaneChangeState reported_state = BehaviorLaneChangeState::Normal;
  BehaviorLaneChangeState module_state = BehaviorLaneChangeState::Normal;
  std::string_view reason;
  bool transit_failure = false;
};

struct BehaviorLaneChangeRtcChannel {
  bool registered = false;
  bool approved = false;
  bool terminated = false;
  bool force_activated = false;
  bool force_deactivated = false;
};

enum class BehaviorLaneChangeRtcState {
  Waiting, Running, Aborting, Succeeded, Failed
};

struct BehaviorLaneChangeRtcStatus {
  bool safe = false;
  bool requested = false;
  bool command_activated = false;
  bool auto_mode = false;
  BehaviorLaneChangeRtcState state = BehaviorLaneChangeRtcState::Waiting;
};

// 기록된 RTC 상태 필드를 인터페이스의 활성·강제 명령으로 바꿉니다.
BehaviorLaneChangeRtcChannel behaviorLaneChangeRtcChannelFromStatus(
    const BehaviorLaneChangeRtcStatus &status);

struct BehaviorLaneChangeRtcSignals {
  bool activated = false;
  bool force_activated = false;
  bool force_deactivated = false;
  bool ready = false;
};

// 모든 RTC 인터페이스의 등록·강제 명령을 원본의 any_of 규칙으로 합칩니다.
BehaviorLaneChangeRtcSignals assembleBehaviorLaneChangeRtcSignals(
    const std::vector<BehaviorLaneChangeRtcChannel> &channels,
    bool execution_ready, bool rtc_interfaces_configured = true);

struct BehaviorLaneChangeAbortInput {
  Trajectory selected_path;
  Trajectory current_lane_reference;
  Trajectory previous_module_path;
  BehaviorPathShiftAnchor ego;
  double ego_velocity_mps = 0.0;
  double minimum_lane_change_velocity_mps = 0.0;
  double delta_time_s = 0.0;
  double return_duration_s = 0.0;
  double maximum_lateral_jerk_mps3 = 0.0;
  double maximum_lateral_acceleration_mps2 = 0.0;
  double stopping_distance_m = 0.0;
  double distance_to_terminal_start_m = 0.0;
};

// 현재 차로 중심선으로 돌아가는 횡이동을 생성하고 이전 모듈 경로에 잇습니다.
std::optional<Trajectory> generateBehaviorLaneChangeAbortPath(
    const BehaviorLaneChangeAbortInput &input);

// 승인 대기, 강제 활성·해제, 안전 악화에 따른 취소·중단·정지 우선순위입니다.
BehaviorLaneChangeFailureDecision decideBehaviorLaneChangeFailureTransition(
    const BehaviorLaneChangeFailureInput &input);

struct BehaviorLaneChangeExecutionInput {
  BehaviorLaneChangeCandidateSearchResult search;
  BehaviorLaneChangeFailureInput failure;
  bool activated = false;
  Trajectory fallback_path;
  std::optional<Trajectory> abort_path;
  std::optional<BehaviorLaneChangeAbortInput> abort_generation;
  std::optional<Trajectory> stop_path;
  std::vector<BehaviorLaneChangeRtcChannel> rtc_channels;
  std::vector<BehaviorLaneChangeRtcStatus> rtc_statuses;
  bool use_rtc_channels = false;
};

struct BehaviorLaneChangeExecutionResult {
  Trajectory path;
  BehaviorLaneChangeFailureDecision decision;
  std::optional<std::size_t> selected_candidate_index;
};

// 후보 안전성·승인·중단 상태를 적용해 실제로 내보낼 경로를 고릅니다.
BehaviorLaneChangeExecutionResult executeBehaviorLaneChange(
    const BehaviorLaneChangeExecutionInput &input);

}  // planning 네임스페이스
