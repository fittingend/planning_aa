// SPDX-License-Identifier: Apache-2.0
// 원본 사각지대 모듈의 통과 판단선·TTC·정지 유지 계산입니다.

#include "planning/blind_spot_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace planning {

BlindSpotCore::BlindSpotCore(BlindSpotParameters parameters)
    : parameters_(parameters) {
  if (!std::isfinite(parameters.ttc_min_s) ||
      !std::isfinite(parameters.ttc_max_s) ||
      parameters.ttc_min_s >= parameters.ttc_max_s ||
      !std::isfinite(parameters.ego_minimum_velocity_mps) ||
      parameters.ego_minimum_velocity_mps <= 0.0 ||
      !std::isfinite(parameters.release_margin_s) ||
      parameters.release_margin_s < 0.0)
    throw std::invalid_argument("사각지대 설정이 유효하지 않습니다");
}

void BlindSpotCore::reset() {
  stopped_ = false;
  passed_judge_line_ = false;
  release_start_s_.reset();
}

BlindSpotDecision BlindSpotCore::evaluate(const BlindSpotInput &input) {
  if (!std::isfinite(input.now_s) ||
      !std::isfinite(input.ego_velocity_mps) ||
      !std::isfinite(input.ego_path_arc_m) ||
      !std::isfinite(input.default_stop_arc_m) ||
      !std::isfinite(input.critical_stop_arc_m) ||
      !std::isfinite(input.vehicle_front_m) ||
      !std::isfinite(input.maximum_stop_acceleration_mps2) ||
      input.maximum_stop_acceleration_mps2 >= 0.0 ||
      !std::isfinite(input.response_delay_s) ||
      input.response_delay_s < 0.0 ||
      !std::isfinite(input.ego_detection_lane_arc_m) ||
      !std::isfinite(input.stop_detection_lane_arc_m))
    throw std::invalid_argument("사각지대 입력이 유효하지 않습니다");

  BlindSpotDecision result;
  result.stop_arc_m = input.ego_path_arc_m > input.default_stop_arc_m ?
      input.critical_stop_arc_m : input.default_stop_arc_m;
  if (parameters_.use_pass_judge_line && passed_judge_line_) {
    result.kind = BlindSpotDecisionKind::OverPassJudge;
    return result;
  }
  const double judge_distance = input.ego_velocity_mps *
      input.ego_velocity_mps /
      (2.0 * -input.maximum_stop_acceleration_mps2) +
      input.response_delay_s * input.ego_velocity_mps;
  const double distance_to_stop = result.stop_arc_m +
      input.vehicle_front_m - input.ego_path_arc_m;
  if (parameters_.use_pass_judge_line && !stopped_ &&
      distance_to_stop + 0.1 < judge_distance) {
    passed_judge_line_ = true;
    result.kind = BlindSpotDecisionKind::OverPassJudge;
    return result;
  }

  result.ego_time_to_stop_s =
      (input.stop_detection_lane_arc_m - input.ego_detection_lane_arc_m) /
      std::max(parameters_.ego_minimum_velocity_mps,
               input.ego_velocity_mps);
  for (const auto &object : input.objects) {
    if (object.classification != 5 && object.classification != 6 &&
        object.classification != 7) continue;
    if (!object.inside_detection_area) continue;
    if (!std::isfinite(object.arc_on_detection_lane_m) ||
        !std::isfinite(object.longitudinal_velocity_mps))
      throw std::invalid_argument("사각지대 객체 입력이 유효하지 않습니다");
    const double object_time =
        (object.arc_on_detection_lane_m -
         input.stop_detection_lane_arc_m) /
        object.longitudinal_velocity_mps;
    const double ttc = result.ego_time_to_stop_s - object_time;
    if (parameters_.ttc_min_s < ttc && ttc < parameters_.ttc_max_s) {
      result.collision_object_id = object.id;
      result.collision_ttc_s = ttc;
      break;
    }
  }

  if (result.collision_object_id) {
    stopped_ = true;
    release_start_s_.reset();
  } else if (stopped_) {
    if (!release_start_s_) release_start_s_ = input.now_s;
    else if (input.now_s - *release_start_s_ >
             parameters_.release_margin_s) {
      stopped_ = false;
      release_start_s_.reset();
    }
  }
  result.kind = stopped_ ? BlindSpotDecisionKind::Unsafe :
                           BlindSpotDecisionKind::Safe;
  return result;
}

TrajectoryStopResult applyBlindSpotDecision(
    const Trajectory &path, const BlindSpotDecision &decision) {
  if (decision.kind != BlindSpotDecisionKind::Unsafe)
    return {path, std::nullopt};
  return insertZeroVelocityAtArc(path, decision.stop_arc_m);
}

}  // planning 네임스페이스
