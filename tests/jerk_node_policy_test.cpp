#include "planning/jerk_node_policy.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

int main() {
  const planning::Trajectory input{{0.0, 0.0, 2.0F, 0.0F},
                                   {1.0, 0.0, 0.0F, 0.0F},
                                   {2.0, 0.0, 0.0F, 0.0F}};
  planning::Trajectory output{{0.0, 0.0, 2.0F, 0.0F},
                              {0.9, 0.0, 0.01F, 0.0F},
                              {2.0, 0.0, 0.01F, 0.0F}};
  const auto status = planning::enforceJerkNodeStop(input, output, 0.2, 1.0);
  if (!status.stop_found || !status.stop_output_found ||
      status.velocity_before_enforcement != 0.01F ||
      output[0].longitudinal_velocity_mps != 2.0F ||
      output[1].longitudinal_velocity_mps != 0.0F ||
      output[2].longitudinal_velocity_mps != 0.0F) {
    std::cerr << "실패: 정지점 이후 속도 제한\n";
    return EXIT_FAILURE;
  }
  planning::Trajectory approach{{0.0, 0.0, 4.0F, 0.0F},
                                {1.0, 0.0, 4.0F, 0.0F},
                                {2.0, 0.0, 4.0F, 0.0F},
                                {3.0, 0.0, 0.0F, 0.0F}};
  planning::limitVelocityApproachingStop(approach, 2.0, 2.0);
  if (approach[0].longitudinal_velocity_mps != 4.0F ||
      approach[1].longitudinal_velocity_mps != 2.0F ||
      approach[2].longitudinal_velocity_mps != 2.0F) {
    std::cerr << "실패: 정지점 접근 속도 제한\n";
    return EXIT_FAILURE;
  }
  const planning::JerkInitialMotionParameters parameters;
  const auto first = planning::selectJerkInitialMotion(
      5.0, -0.5, 4.0, 20.0, false, std::nullopt, parameters);
  if (first.type != planning::JerkInitialType::EgoVelocity ||
      first.motion.velocity != 5.0 || first.motion.acceleration != 0.0) {
    std::cerr << "실패: 첫 입력은 현재 속도와 가속도 0으로 시작해야 합니다\n";
    return EXIT_FAILURE;
  }
  const auto deviation = planning::selectJerkInitialMotion(
      8.0, 0.2, 4.0, 20.0, true, planning::JerkMotion{1.0, -0.4}, parameters);
  if (deviation.type != planning::JerkInitialType::LargeDeviationReplan ||
      deviation.motion.velocity != 8.0 || deviation.motion.acceleration != -0.4) {
    std::cerr << "실패: 큰 속도 편차에서 다시 계획해야 합니다\n";
    return EXIT_FAILURE;
  }
  const auto engaging = planning::selectJerkInitialMotion(
      0.0, 0.0, 4.0, 10.0, false, planning::JerkMotion{0.0, 0.0},
      parameters, 0.7);
  if (engaging.type != planning::JerkInitialType::Engaging ||
      engaging.motion.velocity != 0.25 || engaging.motion.acceleration != 0.7) {
    std::cerr << "실패: 정지 상태의 출발 속도와 외부 가속도 요청\n";
    return EXIT_FAILURE;
  }
  const auto manual = planning::selectJerkInitialMotion(
      3.0, -2.0, 2.0, 0.2, false, planning::JerkMotion{3.0, 0.0}, parameters);
  if (manual.type != planning::JerkInitialType::EgoVelocity ||
      manual.motion.velocity != 2.0 || manual.motion.acceleration != -1.0) {
    std::cerr << "실패: 수동 운전에서는 목표 속도와 감속 한도를 지켜야 합니다\n";
    return EXIT_FAILURE;
  }
  const auto previous = planning::selectJerkInitialMotion(
      3.0, 0.2, 4.0, 10.0, true, planning::JerkMotion{3.1, -0.1}, parameters);
  if (previous.type != planning::JerkInitialType::PreviousOutput ||
      std::fabs(previous.motion.velocity - 3.1) > 1.0e-9) {
    std::cerr << "실패: 자율 운전 중 이전 출력 상태를 이어야 합니다\n";
    return EXIT_FAILURE;
  }
  const planning::Trajectory previous_path{
      {0.0, 0.0, 5.0F, 0.0F}, {2.0, 0.0, 3.0F, -1.0F},
      {4.0, 0.0, 1.0F, -2.0F}};
  const planning::TrajectoryPoint ego_at_segment_midpoint{1.0, 0.2};
  const auto projected = planning::projectJerkPreviousMotion(
      previous_path, ego_at_segment_midpoint, 3.0, 1.046);
  if (std::fabs(projected.velocity - 4.0) > 1.0e-6 ||
      std::fabs(projected.acceleration + 0.5) > 1.0e-6) {
    std::cerr << "실패: 이전 출력 선분에서 초기 운동 상태를 보간해야 합니다\n";
    return EXIT_FAILURE;
  }
  planning::Trajectory reversed_overlap{
      {0.0, 0.0, 1.0F}, {1.0, 0.0, 1.0F}, {2.0, 0.0, 1.0F},
      {2.0, 0.0, 3.0F}, {1.0, 0.0, 3.0F}, {0.0, 0.0, 3.0F}};
  for (std::size_t i = 3; i < reversed_overlap.size(); ++i) {
    reversed_overlap[i].orientation_z = 1.0;
    reversed_overlap[i].orientation_w = 0.0;
  }
  planning::TrajectoryPoint reverse_ego{1.0, 0.0};
  reverse_ego.orientation_z = 1.0;
  reverse_ego.orientation_w = 0.0;
  const auto reverse_motion = planning::projectJerkPreviousMotion(
      reversed_overlap, reverse_ego, 3.0, 1.046);
  if (std::fabs(reverse_motion.velocity - 3.0) > 1.0e-6) {
    std::cerr << "실패: 겹치는 경로에서는 차량 방향의 선분을 선택해야 합니다\n";
    return EXIT_FAILURE;
  }
  planning::Trajectory behind_input{
      {1.0, 0.0, 2.0F}, {0.0, 0.0, 2.0F}, {-1.0, 0.0, 0.0F}};
  for (auto &point : behind_input) {
    point.orientation_z = 1.0;
    point.orientation_w = 0.0;
  }
  const planning::Trajectory behind_optimized{
      behind_input[1], behind_input[2]};
  planning::JerkNodeOutputParameters behind_parameters;
  behind_parameters.max_velocity = 10.0;
  behind_parameters.nearest_distance = 3.0;
  behind_parameters.nearest_yaw = 1.046;
  behind_parameters.keep_closest_motion_behind = false;
  planning::Trajectory behind_internal;
  planning::finalizeNextJerkNodeTrajectory(
      behind_input, 1, behind_optimized, reversed_overlap, reverse_ego,
      2.0, behind_parameters, &behind_internal);
  if (behind_internal.empty() ||
      std::fabs(behind_internal.front().longitudinal_velocity_mps - 3.0) >
          1.0e-6) {
    std::cerr << "실패: 뒤쪽 출력은 이전 경로의 같은 방향 차로에서 보간해야 합니다\n";
    return EXIT_FAILURE;
  }
  std::cout << "정지점 정책 통과\n";
}
