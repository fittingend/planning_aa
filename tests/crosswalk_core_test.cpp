#include "planning/crosswalk_core.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool value, const char *message) {
  if (!value) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}
}

int main() {
  planning::Trajectory path(21);
  for (std::size_t i = 0; i < path.size(); ++i) {
    path[i].x = static_cast<double>(i);
    path[i].longitudinal_velocity_mps = 5.0F;
  }
  const std::vector<planning::CrosswalkPoint> polygon{
      {12.0, -2.0}, {16.0, -2.0}, {16.0, 2.0}, {12.0, 2.0}};
  const auto result = planning::planCrosswalkDefaultStop(path, polygon, {2.0, 1.0});
  check(result.entry_arc_m && std::abs(*result.entry_arc_m - 12.0) < 1e-9,
        "횡단보도 진입점을 찾습니다");
  check(result.exit_arc_m && std::abs(*result.exit_arc_m - 16.0) < 1e-9,
        "횡단보도 출구점을 찾습니다");
  check(result.stop_arc_m && std::abs(*result.stop_arc_m - 9.0) < 1e-9 &&
            result.trajectory[9].longitudinal_velocity_mps == 0.0F &&
            result.trajectory[8].longitudinal_velocity_mps == 5.0F,
        "차량 앞부분과 여유 거리를 적용합니다");
  check(planning::planCrosswalkDefaultStop(path, {{30, -1}, {31, -1},
      {31, 1}, {30, 1}}).stop_arc_m == std::nullopt,
      "경로 밖 횡단보도는 정지하지 않습니다");
  check(planning::classifyCrosswalkPass(5.47902, 1.35978) ==
            planning::CrosswalkCollisionState::Yield,
        "실측 충돌 시각은 양보 상태입니다");
  check(planning::classifyCrosswalkPass(2.0, 7.0) ==
            planning::CrosswalkCollisionState::EgoPassFirst,
        "차량 선통과를 판정합니다");
  check(planning::classifyCrosswalkPass(20.0, 2.0) ==
            planning::CrosswalkCollisionState::EgoPassLater,
        "보행자 선통과를 판정합니다");
  check(planning::classifyCrosswalkPass(2.0, 6.2,
      planning::CrosswalkCollisionState::EgoPassFirst) ==
            planning::CrosswalkCollisionState::EgoPassFirst,
        "선통과 상태의 추가 여유를 제거합니다");
  bool rejected = false;
  try { planning::classifyCrosswalkPass(-1.0, 2.0); }
  catch (const std::invalid_argument &) { rejected = true; }
  check(rejected, "음수 도착 시각을 거부합니다");

  const auto gentle_brake = planning::calcCrosswalkDecelDistance(
      16.7, 0.0, 0.0, -0.5, 1.0, -0.5);
  const auto hard_brake = planning::calcCrosswalkDecelDistance(
      16.7, 0.0, 0.0, -2.5, 1.5, -1.5);
  const auto already_braking = planning::calcCrosswalkDecelDistance(
      16.7, 0.0, -2.5, -0.5, 1.0, -0.5);
  const auto gentle_slow = planning::calcCrosswalkDecelDistance(
      16.7, 10.0, 0.0, -0.5, 1.0, -0.5);
  const auto hard_slow = planning::calcCrosswalkDecelDistance(
      16.7, 10.0, 0.0, -2.5, 1.5, -1.5);
  check(gentle_brake && std::abs(*gentle_brake - 287.224) < 0.001 &&
            hard_brake && std::abs(*hard_brake - 69.6947) < 0.001 &&
            already_braking && std::abs(*already_braking - 217.429) < 0.001 &&
            gentle_slow && std::abs(*gentle_slow - 189.724) < 0.001 &&
            hard_slow && std::abs(*hard_slow - 58.028) < 0.001,
        "원본의 감속 거리 다섯 사례를 재현합니다");
  check(!planning::calcCrosswalkDecelDistance(
            16.7, 20.0, 0.0, -0.5, 1.0, -0.5),
        "가속이 필요한 조건은 감속 거리 계산에서 제외합니다");

  planning::CrosswalkPredictedObject object;
  object.length_m = 0.8;
  object.width_m = 0.8;
  object.velocity_y_mps = 2.0;
  object.predicted_paths = {{{11.0, -2.0, 1.57},
                             {11.0, 0.0, 1.57},
                             {11.0, 2.0, 1.57}}};
  planning::CrosswalkCollisionParameters collision_parameters;
  collision_parameters.vehicle_front_m = 1.0;
  collision_parameters.vehicle_back_m = 1.0;
  collision_parameters.vehicle_width_m = 2.0;
  const auto collision = planning::estimateCrosswalkCollision(
      path, {0.0, 0.0}, 12.0, 16.0, 2.0, object,
      collision_parameters);
  check(collision && collision->ego_distance_m > 10.0 &&
            collision->ego_distance_m < 12.0 &&
            collision->time_to_collision_s > 5.0,
        "예측 객체와 차량 통과 영역의 충돌 시각을 계산합니다");
  const auto object_stop = planning::planCrosswalkObjectStop(
      path, polygon, {0.0, 0.0}, *collision, 0.0, 0.0,
      {2.0, 1.0}, {3.0, 3.0, 1.0});
  check(object_stop.stop_arc_m && *object_stop.stop_arc_m < *result.stop_arc_m &&
            object_stop.trajectory[9].longitudinal_velocity_mps == 0.0F,
        "객체 충돌점에 따라 기본 정지점보다 앞서 정지합니다");
  const auto held = planning::applyCrosswalkRestartSuppression(
      path, result, {8.25, 0.0}, true);
  check(held.stop_arc_m && std::abs(*held.stop_arc_m - 8.25) < 1e-9,
        "정지점 가까이 멈춘 차량의 재출발을 억제합니다");
  const auto moving = planning::applyCrosswalkRestartSuppression(
      path, result, {8.25, 0.0}, false);
  check(moving.stop_arc_m && *moving.stop_arc_m == *result.stop_arc_m,
        "주행 중인 차량에는 재출발 억제를 적용하지 않습니다");
  planning::Trajectory long_path(41);
  for (std::size_t i = 0; i < long_path.size(); ++i) {
    long_path[i].x = static_cast<double>(i);
    long_path[i].longitudinal_velocity_mps = 5.0F;
  }
  const std::vector<planning::CrosswalkStuckVehicle> stuck{{{18.0, 0.0}, 2.0, 0.5, true}};
  const auto stuck_stop = planning::planCrosswalkStuckVehicleStop(
      long_path, polygon, {0.0, 0.0}, 2.0, 0.0, stuck,
      false, false, {2.0, 1.0});
  check(stuck_stop.stop_arc_m && *stuck_stop.stop_arc_m >= 9.0 &&
            stuck_stop.trajectory[20].longitudinal_velocity_mps == 0.0F,
        "횡단보도 뒤 저속 차량 때문에 통과 공간이 부족하면 정지합니다");
  const auto skipped_stuck = planning::planCrosswalkStuckVehicleStop(
      long_path, polygon, {0.0, 0.0}, 2.0, 0.0, stuck,
      true, true, {2.0, 1.0});
  check(!skipped_stuck.stop_arc_m,
        "신호등이 있는 회전 차로에서는 정체 차량 판단을 건너뜁니다");
  const auto clear_stuck = planning::planCrosswalkStuckVehicleStop(
      long_path, polygon, {0.0, 0.0}, 2.0, 0.0,
      {{{38.0, 0.0}, 2.0, 0.5, true}}, false, false, {2.0, 1.0});
  check(!clear_stuck.stop_arc_m,
        "횡단보도 뒤 통과 공간이 충분하면 정지하지 않습니다");
  object.predicted_paths = {{{30.0, -2.0, 1.57},
                             {30.0, 0.0, 1.57},
                             {30.0, 2.0, 1.57}}};
  check(!planning::estimateCrosswalkCollision(
            path, {0.0, 0.0}, 12.0, 16.0, 2.0, object,
            collision_parameters),
        "차량 통과 영역 밖 객체는 충돌하지 않습니다");
}
