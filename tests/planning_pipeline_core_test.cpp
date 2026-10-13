#include "planning/planning_pipeline_core.hpp"
#include "planning/behavior_path_postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {
void check(bool valid, const char *message) {
  if (!valid) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}
}

int main() {
  planning::PathOptimizerCoreParameters parameters;
  parameters.geometry.forward_length = 20.0;
  parameters.geometry.maximum_points = 20;
  parameters.minimum_drivable_width = 1.0;
  planning::PlanningPipelineCore pipeline(
      [](const planning::BehaviorGoalPath &path,
         const planning::TrajectoryPoint &, double) {
        return planning::postprocessBehaviorPath(path.points);
      }, parameters);
  planning::PlanningPipelineFrame frame;
  for (int index = 0; index <= 100; ++index) {
    planning::TrajectoryPoint point;
    point.x = index * 0.5;
    point.longitudinal_velocity_mps = 5.0F;
    frame.behavior_path.points.push_back(point);
    frame.behavior_path.lane_ids.push_back({11});
  }
  frame.left_bound = {{-10.0, 3.0}, {60.0, 3.0}};
  frame.right_bound = {{-10.0, -3.0}, {60.0, -3.0}};
  frame.ego.x = 10.0;
  frame.ego_speed_mps = 3.0;
  frame.motion_velocity_plans.push_back({{{20.0, 0.0}}, {}});
  frame.timestamp_s = 1.0;
  const auto first = pipeline.onFrame(frame);
  check(first.status == planning::PlanningPipelineStatus::Complete &&
            first.validation && first.validation->data_ready &&
            !first.behavior_velocity.empty() &&
            !first.elastic_band.empty() && !first.optimized.empty() &&
            !first.motion_velocity.empty() && !first.cruise_stopped.empty() &&
            first.optimizer_state_reset,
        "행동 속도부터 최종 검증까지 한 프레임에서 연결합니다");
  check(first.cruise_stopped.back().longitudinal_velocity_mps == 0.0F,
        "동작 속도 정지점이 최종 궤적까지 유지됩니다");
  frame.timestamp_s = 1.1;
  frame.ego.x = 11.0;
  frame.motion_velocity_plans.clear();
  planning::ObstacleCruiseStopCandidate obstacle;
  obstacle.classification = 1;
  obstacle.collision_distance_from_start = 15.0;
  obstacle.desired_margin = 5.0;
  frame.cruise_stop_candidates.push_back(obstacle);
  const auto second = pipeline.onFrame(frame);
  check(second.status == planning::PlanningPipelineStatus::Complete &&
            second.validation && second.validation->data_ready &&
            second.cruise_stop_distance_from_start_m &&
            std::abs(*second.cruise_stop_distance_from_start_m - 10.0) <
                1.0e-6 &&
            second.cruise_stopped.back().longitudinal_velocity_mps == 0.0F,
        "다음 프레임에서는 장애물 정지 후보를 최종 경로에 적용합니다");
  check(!second.optimizer_state_reset,
        "가까운 다음 프레임은 최적화 이전 상태를 재사용합니다");
  frame.timestamp_s = 1.2;
  frame.ego.x = 17.0;
  const auto moved = pipeline.onFrame(frame);
  check(moved.status == planning::PlanningPipelineStatus::Complete &&
            moved.optimizer_state_reset,
        "차량이 5m 넘게 이동하면 최적화 이전 상태를 초기화합니다");
  pipeline.reset();
  frame.timestamp_s = 0.5;
  const auto restarted = pipeline.onFrame(frame);
  check(restarted.status == planning::PlanningPipelineStatus::Complete &&
            restarted.optimizer_state_reset,
        "초기화하면 새 시계에서 종단 계산을 다시 시작합니다");
  frame.left_bound.clear();
  check(pipeline.onFrame(frame).status ==
            planning::PlanningPipelineStatus::InvalidInput,
        "지도 경계가 없는 프레임은 불완전한 계산으로 표시합니다");
  frame.left_bound = {{-10.0, 3.0}, {60.0, 3.0}};
  frame.cruise_stop_candidates.clear();
  int generated_calls = 0;
  planning::PlanningPipelineCore generated_pipeline(
      [](const planning::BehaviorGoalPath &path,
         const planning::TrajectoryPoint &, double) {
        return planning::postprocessBehaviorPath(path.points);
      }, parameters, {}, planning::InvalidTrajectoryPolicy::UsePreviousResult,
      {}, [&](const planning::Trajectory &optimized,
              const planning::TrajectoryPoint &, double) {
        ++generated_calls;
        check(!optimized.empty(),
              "동작 속도 생성기는 최적화된 경로를 입력받습니다");
        return std::vector<planning::MotionVelocityPlan>{
            {{{20.0, 0.0}}, {}}};
      });
  const auto generated = generated_pipeline.onFrame(frame);
  check(generated.status == planning::PlanningPipelineStatus::Complete &&
            generated_calls == 1 &&
            generated.cruise_stopped.back().longitudinal_velocity_mps == 0.0F,
        "최적화 뒤 같은 프레임에서 생성한 동작 속도 계획을 적용합니다");
  int cruise_calls = 0;
  int cruise_stage_order = 0;
  planning::PlanningPipelineCore cruise_pipeline(
      [](const planning::BehaviorGoalPath &path,
         const planning::TrajectoryPoint &, double) {
        return planning::postprocessBehaviorPath(path.points);
      }, parameters, {}, planning::InvalidTrajectoryPolicy::UsePreviousResult,
      {}, {}, [&](const planning::Trajectory &motion,
                  const planning::TrajectoryPoint &, double, double) {
        ++cruise_calls;
        check(++cruise_stage_order == 1,
              "순항 정지 후보를 감속 단계보다 먼저 분류합니다");
        check(!motion.empty(),
              "순항 후보 생성기는 동작 속도 적용 뒤 경로를 입력받습니다");
        planning::ObstacleCruiseStopCandidate candidate;
        candidate.classification = 1;
        candidate.collision_distance_from_start = 30.0;
        candidate.desired_margin = 5.0;
        return std::vector<planning::ObstacleCruiseStopCandidate>{candidate};
      }, [&](const planning::Trajectory &source,
              const planning::Trajectory &motion,
              const planning::TrajectoryPoint &, double, double) {
        check(++cruise_stage_order == 2,
              "순항 정지 분류 뒤 감속 경로를 계산합니다");
        const auto source_before_stop = std::find_if(
            source.begin(), source.end(),
            [](const auto &point) { return point.x >= 38.0; });
        const auto after_stop = std::find_if(
            motion.begin(), motion.end(),
            [](const auto &point) { return point.x >= 38.0; });
        check(source_before_stop != source.end() &&
                  source_before_stop->longitudinal_velocity_mps > 0.0F &&
                  after_stop != motion.end() &&
                  after_stop->longitudinal_velocity_mps == 0.0F,
              "원본처럼 원본 궤적으로 후보를 고르고 정지 삽입 뒤 감속합니다");
        return motion;
      });
  const auto cruise = cruise_pipeline.onFrame(frame);
  check(cruise.status == planning::PlanningPipelineStatus::Complete &&
            cruise_calls == 1 && cruise_stage_order == 2 &&
            cruise.cruise_candidate_count == 1 &&
            cruise.cruise_stop_distance_from_start_m,
        "같은 프레임의 순항 정지 후보를 최종 경로에 연결합니다");
  planning::PlanningPipelineFrame callback_frame = frame;
  callback_frame.cruise_callback =
      planning::PlanningPipelineFrame::CruiseCallback{};
  callback_frame.cruise_callback->timestamp_s = 1.35;
  callback_frame.cruise_callback->ego = frame.ego;
  callback_frame.cruise_callback->ego.x = 12.5;
  callback_frame.cruise_callback->ego_speed_mps = 1.5;
  int callback_checks = 0;
  planning::PlanningPipelineCore callback_pipeline(
      [&](const planning::BehaviorGoalPath &path,
          const planning::TrajectoryPoint &ego, double stamp) {
        check(ego.x == frame.ego.x && stamp == frame.timestamp_s,
              "행동 경로 단계는 원래 프레임 입력을 유지합니다");
        return planning::postprocessBehaviorPath(path.points);
      }, parameters, {}, planning::InvalidTrajectoryPolicy::UsePreviousResult,
      {}, {}, [&](const planning::Trajectory &,
                  const planning::TrajectoryPoint &ego, double speed,
                  double stamp) {
        check(ego.x == 12.5 && speed == 1.5 && stamp == 1.35,
              "순항 후보는 별도 콜백 입력을 받습니다");
        ++callback_checks;
        return std::vector<planning::ObstacleCruiseStopCandidate>{};
      }, [&](const planning::Trajectory &,
              const planning::Trajectory &motion,
              const planning::TrajectoryPoint &ego, double speed,
              double stamp) {
        check(ego.x == 12.5 && speed == 1.5 && stamp == 1.35,
              "순항 감속도 같은 별도 콜백 입력을 받습니다");
        ++callback_checks;
        return motion;
      });
  const auto callback_result = callback_pipeline.onFrame(callback_frame);
  check(callback_result.status == planning::PlanningPipelineStatus::Complete &&
            callback_checks == 2,
        "별도 순항 콜백 입력으로 종단 계획을 완성합니다");
  int slowdown_calls = 0;
  planning::PlanningPipelineCore slowdown_pipeline(
      [](const planning::BehaviorGoalPath &path,
         const planning::TrajectoryPoint &, double) {
        return planning::postprocessBehaviorPath(path.points);
      }, parameters, {}, planning::InvalidTrajectoryPolicy::UsePreviousResult,
      {}, {}, {},
      [&](const planning::Trajectory &,
          const planning::Trajectory &motion,
          const planning::TrajectoryPoint &, double, double) {
        ++slowdown_calls;
        auto slowed = motion;
        for (auto &point : slowed)
          point.longitudinal_velocity_mps = std::min(
              point.longitudinal_velocity_mps, 2.0F);
        return slowed;
      });
  const auto slowed = slowdown_pipeline.onFrame(frame);
  check(slowed.status == planning::PlanningPipelineStatus::Complete &&
            slowdown_calls == 1 &&
            slowed.cruise_slowed.back().longitudinal_velocity_mps <= 2.0F &&
            slowed.cruise_stopped.back().longitudinal_velocity_mps <= 2.0F,
        "순항 감속 단계를 정지 적용 뒤 최종 검증 앞에 적용합니다");
}
