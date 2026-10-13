// SPDX-License-Identifier: Apache-2.0
// 사각지대 객체 종류·도달 시간·통과 판단선과 정지 유지 경계를 검사합니다.

#include "planning/blind_spot_core.hpp"

#include <cmath>
#include <stdexcept>

int main() {
  using namespace planning;
  const auto check = [](bool condition) {
    if (!condition) throw std::runtime_error("사각지대 계산 검사 실패");
  };
  BlindSpotInput input;
  input.default_stop_arc_m = 20.0;
  input.critical_stop_arc_m = 22.0;
  input.stop_detection_lane_arc_m = 20.0;
  input.ego_velocity_mps = 0.0;
  input.maximum_stop_acceleration_mps2 = -2.5;
  input.objects = {{"자전거", 6, true, 28.0, 2.0}};
  BlindSpotCore core;
  auto decision = core.evaluate(input);
  check(decision.kind == BlindSpotDecisionKind::Unsafe &&
        decision.collision_object_id == "자전거" &&
        std::abs(*decision.collision_ttc_s) < 1e-9 &&
        std::abs(decision.ego_time_to_stop_s - 4.0) < 1e-9 &&
        decision.stop_arc_m == 20.0);
  Trajectory path(31);
  for (std::size_t i = 0; i < path.size(); ++i) {
    path[i].x = static_cast<double>(i);
    path[i].longitudinal_velocity_mps = 5.0F;
  }
  const auto stopped = applyBlindSpotDecision(path, decision);
  check(stopped.stop_arc_m == 20.0 &&
        stopped.trajectory[20].longitudinal_velocity_mps == 0.0F &&
        stopped.trajectory[19].longitudinal_velocity_mps == 5.0F);

  input.now_s = 1.0;
  input.objects.clear();
  check(core.evaluate(input).kind == BlindSpotDecisionKind::Unsafe);
  input.now_s = 1.1;
  check(core.evaluate(input).kind == BlindSpotDecisionKind::Safe);
  check(!applyBlindSpotDecision(path, core.evaluate(input)).stop_arc_m);

  input.ego_path_arc_m = 21.0;
  input.ego_velocity_mps = 1.0;
  decision = core.evaluate(input);
  check(decision.stop_arc_m == 22.0);
  input.objects = {{"차량", 1, true, 28.0, 2.0},
                   {"영역 밖", 7, false, 28.0, 2.0},
                   {"늦은 보행자", 7, true, 40.0, 2.0}};
  check(core.evaluate(input).kind == BlindSpotDecisionKind::Safe);
  input.objects = {{"경계", 7, true, 45.0, 2.0}};
  check(core.evaluate(input).kind == BlindSpotDecisionKind::Safe);

  BlindSpotCore pass_core;
  input.objects.clear();
  input.ego_path_arc_m = 0.0;
  input.default_stop_arc_m = 10.0;
  input.critical_stop_arc_m = 10.0;
  input.ego_velocity_mps = 10.0;
  input.vehicle_front_m = 3.0;
  check(pass_core.evaluate(input).kind == BlindSpotDecisionKind::OverPassJudge);
  input.ego_velocity_mps = 0.0;
  input.objects = {{"보행자", 7, true, 28.0, 2.0}};
  check(pass_core.evaluate(input).kind == BlindSpotDecisionKind::OverPassJudge);
  pass_core.reset();
  input.default_stop_arc_m = 20.0;
  input.critical_stop_arc_m = 22.0;
  input.vehicle_front_m = 0.0;
  check(pass_core.evaluate(input).kind == BlindSpotDecisionKind::Unsafe);
}
