#include "planning/behavior_path_shift.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace {

void check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

bool near(double first, double second, double tolerance = 1.0e-6) {
  return std::abs(first - second) <= tolerance;
}

}  // 익명 네임스페이스

int main() {
  planning::Trajectory path;
  for (int index = 0; index <= 100; ++index) {
    planning::TrajectoryPoint point;
    point.x = static_cast<double>(index);
    path.push_back(point);
  }
  const auto side_shift = planning::generateBehaviorSideShiftPath(
      path, 5, 8.0, 0.0, 2.0);
  check(side_shift && side_shift->line.start_index == 13 &&
            side_shift->line.end_index == 68 &&
            near(side_shift->path.front().y, 0.0) &&
            near(side_shift->path.back().y, 2.0),
        "사이드 시프트의 시작 시간·저크 거리와 최종 이동량을 계산합니다");
  const auto adjusted_side_shift =
      planning::generateBehaviorSideShiftPath(
          path, 5, 8.0, 1.0, 2.0);
  check(adjusted_side_shift &&
            near(adjusted_side_shift->path.front().y, 1.0) &&
            near(adjusted_side_shift->path.back().y, 2.0) &&
            !planning::generateBehaviorSideShiftPath(
                path, 99, 8.0, 0.0, 2.0),
        "이미 적용한 이동량을 유지하고 경로가 부족한 요청은 거부합니다");
  planning::BehaviorPathShiftParameters settings;
  const auto located = planning::behaviorShiftLineFromPositions(
      path, {20.2, 0.1}, {79.7, -0.2}, -3.0);
  check(located.start_index == 20 && located.end_index == 80,
        "이동선 위치를 가장 가까운 경로 인덱스로 바꿉니다");
  settings.base_offset_m = 0.01;
  const auto spline = planning::shiftBehaviorPath(
      path, {{0, 100, 20.0}}, settings);
  check(spline.path.size() == path.size() &&
            near(spline.shift_lengths_m[0], 0.01) &&
            near(spline.shift_lengths_m[25], 19.99 / 12.0 + 0.01) &&
            near(spline.shift_lengths_m[50], 10.005) &&
            near(spline.shift_lengths_m[75], 11.0 * 19.99 / 12.0 + 0.01) &&
            near(spline.shift_lengths_m[100], 20.0),
        "원본의 4점 자연 스플라인 횡이동 값을 따릅니다");
  check(near(spline.path[50].y, 10.005),
        "직선 경로에서 왼쪽 횡이동을 적용합니다");

  settings.type = planning::BehaviorPathShiftType::Linear;
  const auto linear = planning::shiftBehaviorPath(
      path, {{20, 80, -3.0}}, settings);
  check(near(linear.path[20].y, 0.01) &&
            near(linear.path[50].y, -1.495) &&
            near(linear.path[80].y, -3.0) &&
            near(linear.path[100].y, -3.0),
        "선형 횡이동의 시작·중간·끝 위치를 계산합니다");
  settings.type = planning::BehaviorPathShiftType::Spline;
  settings.offset_back = false;
  settings.base_offset_m = 0.0;
  const auto front_offset = planning::shiftBehaviorPath(
      path, {{20, 80, 3.0}}, settings);
  check(front_offset.shift_lengths_m.front() > 0.0 &&
            front_offset.shift_lengths_m[20] > 0.0 &&
            near(front_offset.shift_lengths_m[80], 0.0) &&
            near(front_offset.shift_lengths_m.back(), 0.0),
        "앞쪽 오프셋 모드는 이동 전 구간만 이동합니다");

  std::vector<planning::BehaviorPathShiftLine> lines{
      {10, 30, 2.0}, {35, 70, 4.0}, {20, 40, 3.0}};
  double offset = 0.01;
  planning::trimBehaviorPathShiftLines(lines, 42, offset);
  check(lines.size() == 1 && lines.front().end_index == 70 &&
            near(offset, 2.0),
        "원본 비교자 순서대로 지나간 횡이동량을 기본 오프셋으로 넘깁니다");

  settings.type = planning::BehaviorPathShiftType::Spline;
  settings.offset_back = true;
  settings.base_offset_m = 0.0;
  settings.velocity_mps = 6.0;
  settings.longitudinal_acceleration_mps2 = 1.0;
  settings.lateral_acceleration_limit_mps2 = 2.0;
  const auto limited = planning::shiftBehaviorPath(
      path, {{10, 40, 5.0}}, settings);
  check(near(limited.shift_lengths_m[10], 0.0) &&
            near(limited.shift_lengths_m[40], 5.0) &&
            limited.shift_lengths_m[25] > 0.0 &&
            limited.shift_lengths_m[25] < 5.0,
        "가속도 제한 횡이동도 양 끝 조건과 단조 진행을 유지합니다");
  const std::vector<double> source_distances{
      0.0, 3.6645406, 8.765561, 13.196938,
      17.967602, 24.462500, 30.0};
  const std::vector<double> source_shifts{
      0.0, 0.113095, 1.079422, 2.5, 3.920578, 4.886904, 5.0};
  planning::Trajectory source_path;
  for (const double distance : source_distances) {
    planning::TrajectoryPoint point;
    point.x = distance;
    source_path.push_back(point);
  }
  const auto source_case = planning::shiftBehaviorPath(
      source_path, {{0, 6, 5.0}}, settings);
  for (std::size_t index = 0; index < source_shifts.size(); ++index) {
    check(near(source_case.shift_lengths_m[index],
               source_shifts[index], 1.0e-5),
          "원본 가속도 제한 예제의 일곱 기준점을 재현합니다");
  }
  auto repeated = linear.path;
  repeated.insert(repeated.begin() + 51, repeated[50]);
  const auto cleaned = planning::finalizeBehaviorPathShift(repeated);
  check(cleaned.size() == linear.path.size() &&
            cleaned[50].orientation_w > 0.0,
        "횡이동 뒤 중복점을 지우고 경로 방향을 다시 계산합니다");
  planning::Trajectory reversed;
  for (const double coordinate : {0.0, 1.0, 0.5, 2.0}) {
    planning::TrajectoryPoint point;
    point.x = coordinate;
    reversed.push_back(point);
  }
  const auto forward = planning::finalizeBehaviorPathShift(reversed);
  check(forward.size() == 3 && near(forward[1].x, 1.0) &&
            near(forward[2].x, 2.0),
        "원본처럼 처음 발견한 역방향 점을 제거합니다");
  check(near(planning::requiredStartPullOutDistance(
                 1.0, 4.0, 2.0, 0.1, 5.0), std::sqrt(76.0)),
        "출발 횡이동 거리의 가속도·곡률 요구량을 합칩니다");
  check(near(planning::startPullOutShiftTime(80.0, -2.0, -4.0),
             11.16515139, 1.0e-6) &&
            near(planning::startPullOutShiftTime(0.0, -2.0, -4.0), 0.0),
        "원본의 출발 횡이동 저크 시간 예제를 재현합니다");
  planning::Trajectory curved(3);
  curved[0].x = 0.0;
  curved[1].x = 1.0;
  curved[2].x = 2.0;
  curved[2].y = 1.0;
  const double curvature = 2.0 / std::sqrt(10.0);
  const double expected_before = 1.0 +
      (1.5 - 1.0 / (1.0 + curvature)) ;
  check(near(planning::startPullOutDistanceBeforeShift(
                 curved, 1.5, 1.0), expected_before),
        "좌회전 경로의 횡이동 후 호길이를 이동 전 거리로 환산합니다");
  check(near(planning::startPullOutDistanceBeforeShift(
                 path, 12.5, 1.0), 12.5),
        "직선 경로에서는 이동 전후 호길이를 같게 둡니다");
  planning::Trajectory start_reference = path;
  for (auto &point : start_reference) {
    point.longitudinal_velocity_mps = 5.0F;
  }
  const auto pull_out = planning::generateStartPullOutCandidates(
      start_reference, {20.0, 2.0}, 2.0, true);
  check(pull_out.size() == 4,
        "출발 계획기의 횡가속도 네 표본으로 후보를 만듭니다");
  for (const auto &candidate : pull_out) {
    check(candidate.shifted && candidate.shift_end_index > 21 &&
              std::abs(candidate.path.front().y - 2.0) < 0.05 &&
              std::abs(candidate.path[candidate.shift_end_index].y) < 0.2 &&
              near(candidate.path.back().longitudinal_velocity_mps, 0.0),
          "출발 위치에서 차로 중심으로 이동하고 목표 종점에서 정지합니다");
    check(candidate.terminal_velocity_mps > 0.0 &&
              candidate.longitudinal_acceleration_mps2 <= 1.0,
          "횡가속도별 종료 속도와 종가속도 한계를 계산합니다");
  }
  const auto aligned = planning::generateStartPullOutCandidates(
      start_reference, {20.0, 0.0}, 0.0, false);
  check(!aligned.empty() && !aligned.front().shifted &&
            near(aligned.front().path[20].y, 0.0),
        "이미 중심선에 있는 차량에는 비이동 후보를 포함합니다");
  planning::StartPullOutParameters one_acceleration;
  one_acceleration.maximum_lateral_acceleration_mps2 =
      one_acceleration.minimum_lateral_acceleration_mps2;
  const auto single = planning::generateStartPullOutCandidates(
      start_reference, {20.0, 2.0}, 2.0, false, one_acceleration);
  check(single.size() == 1 && single.front().shifted,
        "횡가속도 범위가 한 값이면 후보도 한 번만 생성합니다");
  planning::Trajectory prepare(start_reference.begin(),
                               start_reference.begin() + 21);
  planning::Trajectory target(start_reference.begin() + 20,
                              start_reference.begin() + 81);
  for (auto &point : target) point.y = 3.0;
  const auto lane_change = planning::generateLaneChangeShiftCandidate(
      prepare, target, {20.0, 0.0}, {50.0, 3.0}, -3.0,
      6.0, 0.0, 0.5, 2.0);
  check(lane_change.has_value() && lane_change->path.size() == 81 &&
            std::abs(lane_change->shifted_target.front().y) < 0.1 &&
            near(lane_change->shifted_target[30].y, 3.0) &&
            lane_change->shifted_target[10].longitudinal_velocity_mps <= 2.0F,
        "준비 구간과 목표 차로의 횡이동 후보를 한 경로로 연결합니다");
  prepare[prepare.size() - 2].orientation_z =
      std::sin(0.5 * 0.4);
  prepare[prepare.size() - 2].orientation_w =
      std::cos(0.5 * 0.4);
  const auto rejected_change = planning::generateLaneChangeShiftCandidate(
      prepare, target, {20.0, 0.0}, {50.0, 3.0}, -3.0,
      6.0, 0.0, 0.5, 2.0);
  check(!rejected_change,
        "준비 경로와 변경 경로의 접합 자세가 5도를 넘으면 버립니다");
  const auto target_range = planning::laneChangeTargetReferenceArcRange(
      20.0, 30.0, 40.0, 100.0, 5.0, std::nullopt);
  check(target_range && near(target_range->start_m, 20.0) &&
            near(target_range->end_m, 90.0),
        "목표 차로에서 변경 후 전방 경로를 추출합니다");
  const auto goal_range = planning::laneChangeTargetReferenceArcRange(
      20.0, 30.0, 40.0, 100.0, 5.0, 60.0);
  check(goal_range && near(goal_range->end_m, 55.0),
        "목표점이 있는 차로에서는 목표점 앞 여유를 우선합니다");
  check(!planning::laneChangeTargetReferenceArcRange(
             20.0, 30.0, 40.0, 100.0, 5.0, 54.99),
        "목표점 앞에서 횡이동 길이가 부족하면 후보를 제외합니다");
  check(near(planning::laneChangeTargetReferenceInterval(60.0, 5.0), 2.0) &&
            near(planning::laneChangeTargetReferenceInterval(15.0, 8.0), 1.6),
        "최소 점 수와 주행 시간에 따른 재표본화 간격을 고릅니다");

  const auto acceleration_samples = planning::sampleLaneChangeAccelerations(
      -1.0, 1.0, 3);
  check(acceleration_samples.size() == 5 &&
            near(acceleration_samples.front(), 1.0) &&
            near(acceleration_samples[2], 0.0) &&
            near(acceleration_samples.back(), -1.0),
        "가속·감속 표본 사이에 영가속도를 추가하고 내림차순으로 정렬합니다");
  check(near(planning::laneChangeActualPrepareDuration(
                 1.0, 2.78, 1.0, 2.0, 4.0, 1.0), 3.0) &&
            near(planning::laneChangeActualPrepareDuration(
                     0.0, 5.0, 1.0, 2.0, 4.0, 1.0), 5.0) &&
            near(planning::laneChangeActualPrepareDuration(
                     0.0, 5.0, 0.0, 2.0, 4.0, 1.0), 4.0),
        "방향 지시등 시간과 최소 변경 속도 도달 시간을 함께 지킵니다");
  const auto far_prepare = planning::laneChangePrepareDurations(
      10.0, 3.79, 1.0, 3.0, 4.0);
  const auto near_prepare = planning::laneChangePrepareDurations(
      4.0, 3.79, 1.0, 3.0, 4.0);
  check(far_prepare.size() == 1 && near_prepare.size() == 9 &&
            near(near_prepare.front(), 4.0) &&
            near(near_prepare.back(), 0.0),
        "차로 종단 가까이에서만 준비 시간을 0.5초씩 줄여 표본을 만듭니다");
  check(!planning::laneChangeNeedsAccelerationSampling(
            1.0, false, 10.0, 20.0, 30.0) &&
            planning::laneChangeNeedsAccelerationSampling(
                1.0, false, 20.0, 20.0, 20.0) &&
            planning::laneChangeNeedsAccelerationSampling(
                1.0, true, 10.0, 20.0, 30.0),
        "차로 끝 거리와 정체 상태에 따라 가속도 표본을 선택합니다");
  const auto prepare_phases = planning::generateLaneChangePreparePhases(
      5.0, 10.0, 12.0, 2.78, 2.0, -1.0, 1.0, 5,
      true, 0.0, 50.0);
  check(prepare_phases.size() == 7 &&
            near(prepare_phases.front().terminal_velocity_mps, 7.0) &&
            near(prepare_phases.front().length_m, 12.0) &&
            near(prepare_phases.back().terminal_velocity_mps, 3.0),
        "준비 구간의 속도·거리와 양쪽 가속도 경계를 계산합니다");
  const auto single_prepare = planning::generateLaneChangePreparePhases(
      5.0, 10.0, 12.0, 2.78, 2.0, -1.0, 1.0, 5,
      false, 0.0, 50.0);
  check(single_prepare.size() == 1 &&
            near(single_prepare.front().length_m, 12.0),
        "표본이 필요 없는 준비 구간에는 최대 가속도만 사용합니다");
  const auto shift_phases = planning::generateLaneChangeShiftPhases(
      3.0, 7.0, 10.0, 12.0, 2.78, 1.0,
      0.5, 1.0, 2, 0.5, 100.0, false, 0.5);
  check(shift_phases.size() == 3 &&
            shift_phases.front().duration_s > shift_phases.back().duration_s &&
            shift_phases.front().length_m > 0.0 &&
            shift_phases.front().terminal_velocity_mps <= 10.0,
        "횡가속도가 커지면 변경 시간이 줄고 경로 속도 상한을 지킵니다");
  const auto terminal_deceleration = planning::generateLaneChangeShiftPhases(
      3.0, 7.0, 10.0, 12.0, 2.78, -1.0,
      0.5, 0.5, 2, 0.5, 100.0, true, 0.5);
  check(terminal_deceleration.size() == 1 &&
            near(terminal_deceleration.front().actual_longitudinal_acceleration_mps2,
                 -0.5),
        "차로 종단 부근에서는 준비 감속의 계수를 변경 구간에 적용합니다");
  planning::LaneChangeCandidateLengthGate length_gate(1.0, 2.0);
  check(length_gate.acceptsPrepare(10.0) &&
            length_gate.acceptsShift(10.0, 20.0) &&
            length_gate.acceptsPrepare(10.0),
        "경로 생성에 실패한 후보는 다음 길이 비교의 기준이 되지 않습니다");
  length_gate.recordConstructed(10.0, 20.0);
  check(!length_gate.acceptsPrepare(11.0) &&
            length_gate.acceptsPrepare(11.01) &&
            !length_gate.acceptsShift(10.5, 22.0) &&
            length_gate.acceptsShift(10.5, 22.01) &&
            length_gate.acceptsShift(11.01, 20.0),
        "마지막 생성 후보와 두 구간 길이를 엄격한 경계로 비교합니다");
  return EXIT_SUCCESS;
}
