#include "planning/behavior_lane_change_safety.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

void check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

}  // 익명 네임스페이스

int main() {
  using Group = planning::BehaviorLaneChangeObjectGroup;
  planning::BehaviorLaneChangeObjectFilterInput filter;
  filter.initial_polygon =
      {{4.0, 2.0}, {6.0, 2.0}, {6.0, 3.0}, {4.0, 3.0}};
  filter.current_lane =
      {{0.0, -2.0}, {20.0, -2.0}, {20.0, 1.0}, {0.0, 1.0}};
  filter.target_lane =
      {{0.0, 1.5}, {20.0, 1.5}, {20.0, 4.0}, {0.0, 4.0}};
  filter.expanded_target_lane =
      {{0.0, 1.0}, {20.0, 1.0}, {20.0, 5.0}, {0.0, 5.0}};
  filter.vehicle_width_m = 2.0;
  filter.object_lateral_from_current_center_m = 2.5;
  filter.ahead_of_ego = true;
  filter.before_terminal = true;
  filter.initial_velocity_mps = 3.0;
  check(planning::classifyBehaviorLaneChangeObject(filter) ==
            Group::TargetLeadingMoving,
        "목표 차로 전방의 이동 객체를 선별합니다");
  auto folded_expansion = filter;
  folded_expansion.expanded_target_lane =
      {{0.0, 1.0}, {20.0, 5.0}, {0.0, 5.0}, {20.0, 1.0}};
  check(planning::classifyBehaviorLaneChangeObject(folded_expansion) ==
            Group::TargetLeadingMoving,
        "곡선 차로 확장 경계가 접히더라도 원본처럼 목표 차로 객체를 판정합니다");
  filter.initial_velocity_mps = 0.0;
  check(planning::classifyBehaviorLaneChangeObject(filter) ==
            Group::TargetLeadingStopped,
        "목표 차로 전방의 정지 객체를 따로 선별합니다");
  filter.ahead_of_ego = false;
  filter.initial_velocity_mps = 3.0;
  check(planning::classifyBehaviorLaneChangeObject(filter) ==
            Group::TargetTrailing,
        "목표 차로 후방의 이동 객체를 먼저 선별합니다");
  filter.initial_velocity_mps = 0.0;
  check(planning::classifyBehaviorLaneChangeObject(filter) ==
            Group::Other,
        "목표 차로 후방의 정지 객체는 후방 검사에 넣지 않습니다");
  filter.ahead_of_ego = true;
  filter.initial_polygon =
      {{4.0, 4.3}, {6.0, 4.3}, {6.0, 4.8}, {4.0, 4.8}};
  check(planning::classifyBehaviorLaneChangeObject(filter) ==
            Group::TargetLeadingStoppedAtBoundary,
        "확장 경계에만 있는 정지 객체를 선별합니다");
  filter.initial_velocity_mps = 3.0;
  filter.initial_polygon =
      {{4.0, -3.0}, {6.0, -3.0}, {6.0, -2.5}, {4.0, -2.5}};
  filter.is_vehicle = true;
  filter.predicted_paths = {
      {1.0, {{0.0, 5.0, -2.75, 0.0, 3.0},
             {1.0, 5.0, 2.5, 0.0, 3.0}}}};
  check(planning::classifyBehaviorLaneChangeObject(filter) ==
            Group::TargetLeadingMoving,
        "이동 차량의 예측 선분이 목표 차로를 가로지르면 포함합니다");
  filter.is_vehicle = false;
  check(planning::classifyBehaviorLaneChangeObject(filter) == Group::Other,
        "차량이 아닌 객체는 예측 선분만으로 목표 차로에 넣지 않습니다");
  filter.ahead_of_ego = false;
  filter.before_terminal = false;
  filter.preceding_target_lanes = {
      {{3.0, -4.0}, {7.0, -4.0}, {7.0, -2.0}, {3.0, -2.0}}};
  check(planning::classifyBehaviorLaneChangeObject(filter) ==
            Group::TargetTrailing,
        "목표 차로 후방 영역의 이동 객체를 선별합니다");
  filter.initial_polygon =
      {{4.0, -0.5}, {6.0, -0.5}, {6.0, 0.5}, {4.0, 0.5}};
  filter.ahead_of_ego = true;
  filter.before_terminal = true;
  check(planning::classifyBehaviorLaneChangeObject(filter) ==
            Group::CurrentLeading,
        "현재 차로 전방 객체를 별도 집합에 둡니다");
  const auto current_filter = filter;
  auto target_filter = filter;
  target_filter.initial_polygon =
      {{4.0, 2.0}, {6.0, 2.0}, {6.0, 3.0}, {4.0, 3.0}};
  target_filter.object_lateral_from_current_center_m = 2.5;
  auto trailing_filter = target_filter;
  trailing_filter.ahead_of_ego = false;
  auto other_filter = filter;
  other_filter.initial_polygon =
      {{4.0, -5.0}, {6.0, -5.0}, {6.0, -4.0}, {4.0, -4.0}};
  other_filter.preceding_target_lanes.clear();
  const auto selectable = [](
      const planning::BehaviorLaneChangeObjectFilterInput &conditions,
      double distance_m, double identifier) {
    planning::BehaviorLaneChangeSelectableObject object;
    object.filter = conditions;
    object.signed_distance_from_ego_m = distance_m;
    object.target.object.length_m = identifier;
    return object;
  };
  const auto selected = planning::selectBehaviorLaneChangeTargets(
      {selectable(target_filter, 9.0, 9.0),
       selectable(trailing_filter, -12.0, 12.0),
       selectable(current_filter, 2.0, 2.0),
       selectable(target_filter, 5.0, 5.0),
       selectable(other_filter, 1.0, 1.0),
       selectable(trailing_filter, -3.0, 3.0)},
      false, false, false);
  check(selected.leading.size() == 2 &&
            selected.leading[0].object.length_m == 5.0 &&
            selected.leading[1].object.length_m == 9.0 &&
            selected.trailing.size() == 2 &&
            selected.trailing[0].object.length_m == 3.0 &&
            selected.trailing[1].object.length_m == 12.0,
        "목표 객체는 집합별 원본 순서와 후방 거리 역순으로 전달합니다");
  const auto selected_all = planning::selectBehaviorLaneChangeTargets(
      {selectable(target_filter, 9.0, 9.0),
       selectable(current_filter, 2.0, 2.0),
       selectable(other_filter, 1.0, 1.0)},
      false, true, true);
  check(selected_all.leading.size() == 3 &&
            selected_all.leading[0].object.length_m == 9.0 &&
            selected_all.leading[1].object.length_m == 2.0 &&
            selected_all.leading[2].object.length_m == 1.0,
        "자차 정체·기타 차로 설정에서 추가 객체를 뒤에 연결합니다");
  auto oncoming_filter = target_filter;
  oncoming_filter.object_yaw_rad = 3.141592653589793;
  auto stationary_filter = oncoming_filter;
  stationary_filter.initial_velocity_mps = 0.0;
  auto disallowed_filter = target_filter;
  disallowed_filter.type_allowed = false;
  const auto prefiltered = planning::selectBehaviorLaneChangeTargets(
      {selectable(oncoming_filter, 8.0, 8.0),
       selectable(stationary_filter, 5.0, 5.0),
       selectable(disallowed_filter, 3.0, 3.0)},
      true, true, false);
  check(prefiltered.leading.size() == 1 &&
            prefiltered.leading[0].object.length_m == 5.0,
        "이동 역방향·허용되지 않은 종류를 제외하고 정지 역방향은 유지합니다");

  const std::vector<planning::BehaviorSafetyPose> near{
      {0.0, 0.0, 0.0, 0.0, 0.0},
      {2.0, 0.0, 0.0, 0.0, 0.0}};
  const std::vector<planning::BehaviorSafetyPose> far{
      {0.0, 0.0, 20.0, 0.0, 0.0},
      {2.0, 0.0, 20.0, 0.0, 0.0}};
  planning::BehaviorLaneChangeTarget target;
  target.predicted_paths = {
      {1.0, {{1.0, 0.0, 0.0, 0.0, 2.0}}}};
  target.initial_velocity_mps = 2.0;
  const auto one_safe = planning::evaluateBehaviorLaneChangeObjectSafety(
      {near, far}, {target}, {});
  check(one_safe.safe,
        "감속 예측 경로 하나가 안전하면 차로 변경 후보를 거부하지 않습니다");

  const auto trailing = planning::evaluateBehaviorLaneChangeObjectSafety(
      {near, near}, {target}, {target});
  check(!trailing.safe && trailing.object_behind_ego &&
            trailing.ego_path_index == 1 &&
            trailing.collision.has_value(),
        "모든 경로 충돌 시 후방 객체 판정을 먼저 반환합니다");

  const auto leading = planning::evaluateBehaviorLaneChangeObjectSafety(
      {near}, {}, {target});
  check(!leading.safe && !leading.object_behind_ego,
        "후방이 안전할 때 전방 객체 충돌을 반환합니다");

  planning::BehaviorLaneChangeSafetyParameters parameters;
  parameters.prepare_duration_s = 2.0;
  const auto after_prepare = planning::evaluateBehaviorLaneChangeObjectSafety(
      {near}, {}, {target}, parameters);
  check(after_prepare.safe,
        "준비 구간을 검사하지 않으면 이전 객체 표본을 제외합니다");
  parameters.check_prepare_phase = true;
  const auto moving_prepare =
      planning::evaluateBehaviorLaneChangeObjectSafety(
          {near}, {}, {target}, parameters);
  check(!moving_prepare.safe,
        "준비 구간 검사에서는 움직이는 객체 표본을 포함합니다");
  target.initial_velocity_mps = 0.0;
  target.predicted_paths[0].poses[0].velocity_mps = 0.0;
  const auto stopped_prepare =
      planning::evaluateBehaviorLaneChangeObjectSafety(
          {near}, {}, {target}, parameters);
  check(stopped_prepare.safe,
        "준비 구간의 정지 객체 표본은 제외합니다");

  const std::vector<planning::BehaviorSafetyPose> fast{
      {0.0, 0.0, 0.0, 0.0, 5.0},
      {2.0, 0.0, 0.0, 0.0, 5.0}};
  planning::BehaviorLaneChangeTarget marginal;
  marginal.predicted_paths = {
      {1.0, {{1.0, 18.0, 0.0, 0.0, 0.0}}}};
  marginal.initial_velocity_mps = 2.0;
  const auto moving_rss =
      planning::evaluateBehaviorLaneChangeObjectSafety(
          {fast}, {}, {marginal});
  marginal.initial_velocity_mps = 0.0;
  const auto parked_rss =
      planning::evaluateBehaviorLaneChangeObjectSafety(
          {fast}, {}, {marginal});
  check(!moving_rss.safe && parked_rss.safe,
        "초기 객체 속도에 따라 주행·주차 RSS 여유를 선택합니다");
  parameters.prepare_duration_s = 0.0;
  parameters.check_lane_regions = true;
  parameters.current_lane_region =
      {{20.0, -3.0}, {25.0, -3.0}, {25.0, 3.0}, {20.0, 3.0}};
  parameters.expanded_target_lane_region =
      {{26.0, -3.0}, {31.0, -3.0}, {31.0, 3.0}, {26.0, 3.0}};
  planning::BehaviorLaneChangeTarget outside;
  outside.predicted_paths = {
      {1.0, {{1.0, 0.0, 0.0, 0.0, 2.0}}}};
  outside.initial_velocity_mps = 2.0;
  const auto outside_lane =
      planning::evaluateBehaviorLaneChangeObjectSafety(
          {near}, {}, {outside}, parameters);
  check(outside_lane.safe,
        "차로 영역 밖 객체와의 충돌은 차로 변경 거부에서 제외합니다");
  parameters.current_lane_region =
      {{-3.0, -3.0}, {3.0, -3.0}, {3.0, 3.0}, {-3.0, 3.0}};
  const auto inside_lane =
      planning::evaluateBehaviorLaneChangeObjectSafety(
          {near}, {}, {outside}, parameters);
  check(!inside_lane.safe,
        "현재 차로 안의 충돌 객체는 차로 변경을 거부합니다");
  parameters.expanded_target_lane_region.clear();
  const auto missing_lane =
      planning::evaluateBehaviorLaneChangeObjectSafety(
          {near}, {}, {outside}, parameters);
  check(missing_lane.safe,
        "목표 차로 다각형이 없으면 원본처럼 충돌 검사를 건너뜁니다");
  planning::Trajectory candidate_path(2);
  candidate_path[1].x = 100.0;
  planning::BehaviorLaneChangeCandidateSafetyInput candidate;
  candidate.prediction.lane_change_duration_s = 2.0;
  candidate.prediction.prepare_velocity_limit_mps = 1.0;
  candidate.prediction.lane_change_velocity_limit_mps = 1.0;
  auto candidate_filter = target_filter;
  candidate_filter.initial_polygon =
      {{-1.0, 2.0}, {1.0, 2.0}, {1.0, 3.0}, {-1.0, 3.0}};
  auto candidate_object = selectable(candidate_filter, 0.0, 4.0);
  candidate_object.target = outside;
  candidate.objects.push_back(candidate_object);
  const auto candidate_collision =
      planning::evaluateBehaviorLaneChangeCandidateSafety(
          candidate_path, candidate);
  check(!candidate_collision.safe &&
            !candidate_collision.object_behind_ego &&
            candidate_collision.collision.has_value(),
        "후보 경로의 예측·선별·객체 충돌을 연속 계산합니다");
  candidate.objects.clear();
  check(planning::evaluateBehaviorLaneChangeCandidateSafety(
            candidate_path, candidate).safe,
        "검사 대상 객체가 없으면 후보를 유지합니다");
  planning::BehaviorLaneChangePreparePlan prepared;
  prepared.phase = {2.0, 20.0, 6.0, 0.0, 0.0, 0.0};
  prepared.prepare_path.resize(21);
  for (std::size_t index = 0; index < prepared.prepare_path.size(); ++index) {
    prepared.prepare_path[index].x = static_cast<double>(index);
    prepared.prepare_path[index].longitudinal_velocity_mps = 6.0F;
  }
  planning::BehaviorLaneChangeShiftPlan shift_plan;
  shift_plan.phase = {5.0, 30.0, 2.0, 0.0, 0.0, 0.5};
  shift_plan.target_reference.resize(61);
  for (std::size_t index = 0; index < shift_plan.target_reference.size(); ++index) {
    shift_plan.target_reference[index].x = 20.0 + index;
    shift_plan.target_reference[index].y = 3.0;
    shift_plan.target_reference[index].longitudinal_velocity_mps = 6.0F;
  }
  shift_plan.change_end = {50.0, 3.0};
  shift_plan.lateral_shift_m = -3.0;
  prepared.shifts.push_back(shift_plan);
  auto invalid_prepared = prepared;
  invalid_prepared.prepare_path[19].orientation_z = std::sin(0.2);
  invalid_prepared.prepare_path[19].orientation_w = std::cos(0.2);
  planning::BehaviorLaneChangeCandidateSearchInput search;
  search.prepare_plans = {invalid_prepared, prepared};
  search.prepare_length_threshold_m = 1.0;
  search.shift_length_threshold_m = 2.0;
  search.safety.prediction.initial_velocity_mps = 6.0;
  search.safety.prediction.prepare_velocity_limit_mps = 6.0;
  search.safety.prediction.lane_change_velocity_limit_mps = 6.0;
  const auto searched = planning::searchBehaviorLaneChangeCandidates(search);
  check(searched.first_safe_index == 0 && searched.candidates.size() == 1 &&
            searched.candidates.front().safety.safe &&
            searched.candidates.front().path.path.size() == 81,
        "경로 생성에 실패한 뒤 같은 길이의 다음 후보를 구성해 안전 판정합니다");
  search.safety.objects = {candidate_object};
  search.safety.objects.front().filter.initial_polygon =
      {{0.0, 0.0}, {1.0, 0.0}};
  const auto malformed = planning::searchBehaviorLaneChangeCandidates(search);
  check(malformed.candidates.size() == 1 &&
            !malformed.first_safe_index &&
            !malformed.candidates.front().safety.safe,
        "잘못된 인식 다각형은 차로 변경 후보를 거절하고 계산을 계속합니다");
  using State = planning::BehaviorLaneChangeState;
  planning::BehaviorLaneChangeFailureInput failure;
  failure.waiting_approval = true;
  failure.near_regulatory_element = true;
  const auto regulatory_cancel =
      planning::decideBehaviorLaneChangeFailureTransition(failure);
  check(regulatory_cancel.transit_failure &&
            regulatory_cancel.module_state == State::Cancel &&
            regulatory_cancel.reason == "CloseToRegElement",
        "승인 대기 중 규제 요소에 가까우면 차로 변경을 취소합니다");
  failure.force_activated = true;
  const auto forced = planning::decideBehaviorLaneChangeFailureTransition(failure);
  check(!forced.transit_failure && forced.module_state == State::Normal &&
            forced.reason.empty(),
        "강제 활성은 실패 상태 전환보다 먼저 적용합니다");
  failure = {};
  failure.preparing = true;
  failure.can_return_current_lane = true;
  failure.force_deactivated = true;
  const auto deactivated =
      planning::decideBehaviorLaneChangeFailureTransition(failure);
  check(deactivated.module_state == State::Cancel &&
            deactivated.reason == "ForceDeactivation",
        "준비 중 복귀할 수 있으면 안전 여부와 관계없이 강제 해제를 적용합니다");
  failure = {};
  failure.post_process_safe = false;
  failure.can_return_current_lane = true;
  failure.near_terminal = true;
  failure.stop_required = true;
  const auto near_terminal =
      planning::decideBehaviorLaneChangeFailureTransition(failure);
  check(near_terminal.reported_state == State::Warning &&
            near_terminal.module_state == State::Stop &&
            near_terminal.reason == "TooNearTerminal",
        "중단이 늦은 불안전 후보는 경고를 기록하고 정지 상태로 보냅니다");
  failure.near_terminal = false;
  failure.abort_path_available = true;
  const auto aborting =
      planning::decideBehaviorLaneChangeFailureTransition(failure);
  check(aborting.module_state == State::Abort &&
            aborting.reason == "SafeToAbort",
        "복귀 경로가 있으면 차로 변경 중단 상태로 갑니다");
  failure.current_state = State::Abort;
  failure.abort_finished = true;
  const auto aborted =
      planning::decideBehaviorLaneChangeFailureTransition(failure);
  check(aborted.transit_failure && aborted.module_state == State::Cancel &&
            aborted.reason == "Aborted",
        "중단 경로를 끝내면 실패 종료 상태로 전환합니다");
  planning::BehaviorLaneChangeExecutionInput execution;
  execution.search = searched;
  planning::TrajectoryPoint fallback_point;
  fallback_point.x = -1.0;
  execution.fallback_path = {fallback_point};
  const auto waiting = planning::executeBehaviorLaneChange(execution);
  check(waiting.path.front().x == -1.0 &&
            !waiting.selected_candidate_index &&
            waiting.decision.reason == "WaitingForApproval",
        "승인 전에는 안전한 후보가 있어도 기존 경로를 유지합니다");
  execution.activated = true;
  const auto approved = planning::executeBehaviorLaneChange(execution);
  check(approved.selected_candidate_index == 0 &&
            approved.path.size() == 81,
        "승인 후 첫 안전 후보를 출력 경로에 적용합니다");
  execution.failure.post_process_safe = false;
  execution.failure.can_return_current_lane = true;
  execution.failure.abort_path_available = true;
  planning::TrajectoryPoint abort_point;
  abort_point.x = -2.0;
  execution.abort_path = planning::Trajectory{abort_point};
  const auto aborted_path = planning::executeBehaviorLaneChange(execution);
  check(aborted_path.decision.module_state == State::Abort &&
            aborted_path.path.front().x == -2.0,
        "안전 악화 뒤 생성한 중단 경로를 실제 출력에 적용합니다");
  execution.abort_path.reset();
  execution.failure.stop_required = true;
  planning::TrajectoryPoint stop_point;
  stop_point.x = -3.0;
  execution.stop_path = planning::Trajectory{stop_point};
  const auto stopped_path = planning::executeBehaviorLaneChange(execution);
  check(stopped_path.decision.module_state == State::Stop &&
            stopped_path.path.front().x == -3.0,
        "중단 경로가 없고 정지가 필요하면 정지 경로를 출력합니다");
  const auto rtc = planning::assembleBehaviorLaneChangeRtcSignals(
      {{false, false, false, false, false},
       {true, true, false, false, false},
       {false, false, false, true, true}}, true);
  check(rtc.activated && rtc.force_activated && rtc.force_deactivated &&
            rtc.ready,
        "여러 RTC 채널의 등록과 강제 신호를 각각 합칩니다");
  const auto automatic = planning::behaviorLaneChangeRtcChannelFromStatus(
      {true, false, false, true,
       planning::BehaviorLaneChangeRtcState::Waiting});
  const auto manual_force = planning::behaviorLaneChangeRtcChannelFromStatus(
      {false, true, true, false,
       planning::BehaviorLaneChangeRtcState::Running});
  const auto manual_release = planning::behaviorLaneChangeRtcChannelFromStatus(
      {true, false, false, false,
       planning::BehaviorLaneChangeRtcState::Running});
  check(automatic.approved && !automatic.force_activated &&
            manual_force.approved && manual_force.force_activated &&
            manual_release.force_deactivated,
        "원본 RTC 자동·수동 승인 및 강제 명령 상태를 변환합니다");
  planning::BehaviorLaneChangeAbortInput abort_input;
  for (int index = 0; index <= 60; ++index) {
    planning::TrajectoryPoint shifted_point;
    shifted_point.x = index;
    shifted_point.y = 2.0;
    abort_input.selected_path.push_back(shifted_point);
    shifted_point.y = 0.0;
    abort_input.current_lane_reference.push_back(shifted_point);
    abort_input.previous_module_path.push_back(shifted_point);
  }
  abort_input.ego = {5.0, 2.0};
  abort_input.ego_velocity_mps = 5.0;
  abort_input.delta_time_s = 1.0;
  abort_input.return_duration_s = 2.0;
  abort_input.maximum_lateral_jerk_mps3 = 1000.0;
  abort_input.maximum_lateral_acceleration_mps2 = 2.0;
  abort_input.distance_to_terminal_start_m = 100.0;
  const auto generated_abort =
      planning::generateBehaviorLaneChangeAbortPath(abort_input);
  check(generated_abort && generated_abort->size() > 30 &&
            std::abs(generated_abort->front().y - 2.0) < 1.0e-6 &&
            std::abs(generated_abort->back().y) < 1.0e-6,
        "중단 경로를 차로 변경 경로에서 현재 차로 경로까지 이어 만듭니다");
  execution.stop_path.reset();
  execution.abort_generation = abort_input;
  execution.use_rtc_channels = true;
  execution.rtc_channels = {{true, true, false, false, false}};
  const auto generated_execution =
      planning::executeBehaviorLaneChange(execution);
  check(generated_execution.decision.module_state == State::Abort &&
            generated_execution.path.size() == generated_abort->size(),
        "RTC 승인과 생성된 중단 경로를 실행 출력에 연결합니다");
  execution.rtc_channels.clear();
  execution.rtc_statuses = {{true, false, true, false,
      planning::BehaviorLaneChangeRtcState::Running}};
  const auto status_execution =
      planning::executeBehaviorLaneChange(execution);
  check(status_execution.decision.module_state == State::Abort &&
            status_execution.path.size() == generated_abort->size(),
        "RTC 원시 상태를 실행 입력에 직접 연결합니다");
  abort_input.maximum_lateral_jerk_mps3 = 0.001;
  check(!planning::generateBehaviorLaneChangeAbortPath(abort_input),
        "저크 한계를 넘는 중단 경로는 거부합니다");
  return EXIT_SUCCESS;
}
