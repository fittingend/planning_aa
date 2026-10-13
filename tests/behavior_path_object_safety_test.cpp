#include "planning/behavior_path_object_safety.hpp"

#include <algorithm>
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
  const std::vector<planning::BehaviorSafetyPose> ego{
      {0.0, 0.0, 0.0, 0.0, 5.0},
      {2.0, 10.0, 0.0, 0.0, 5.0}};
  planning::BehaviorSafetyObject object;
  object.predicted = {{1.0, 5.0, 0.0, 0.0, 0.0}};
  const auto direct = planning::firstBehaviorPathObjectCollision(
      ego, {}, object);
  check(direct && direct->kind ==
            planning::BehaviorSafetyCollisionKind::DirectOverlap,
        "같은 시각에 차체와 객체 외곽이 직접 겹칩니다");
  check(!planning::firstBehaviorPathObjectCollision(
            {{0.0, 5.0, 0.0, 0.0, 0.0}}, {}, object),
        "자차 예측 자세가 한 점이면 원본처럼 보간 결과가 없습니다");
  const std::vector<planning::BehaviorSafetyPose> delayed_ego{
      {1.0, 5.0, 0.0, 0.0, 0.0},
      {2.0, 5.0, 0.0, 0.0, 0.0}};
  object.predicted = {{0.5, 5.0, 0.0, 0.0, 0.0}};
  check(planning::firstBehaviorPathObjectCollision(
            delayed_ego, {}, object).has_value(),
        "첫 자차 표본보다 앞선 양의 시각은 첫 자세를 사용합니다");
  object.predicted = {{1.0, 17.0, 0.0, 0.0, 0.0}};
  const auto extended = planning::firstBehaviorPathObjectCollision(
      ego, {}, object);
  check(extended && extended->kind ==
            planning::BehaviorSafetyCollisionKind::ExtendedOverlap &&
            extended->object_in_front && extended->rss_distance_m > 0.0,
        "전방 정차 객체에 RSS 종방향 거리를 더합니다");
  planning::BehaviorSafetyRssParameters limited_ego;
  limited_ego.ego_velocity_limit_mps = 1.0;
  check(!planning::firstBehaviorPathObjectCollision(
            ego, {}, object, limited_ego),
        "외부 최고 속도 제한을 RSS 자차 정지 거리에 반영합니다");
  object.predicted = {{1.0, -10.0, 0.0, 0.0, 10.0}};
  const auto rear = planning::firstBehaviorPathObjectCollision(
      ego, {}, object);
  check(rear && rear->kind ==
            planning::BehaviorSafetyCollisionKind::ExtendedOverlap &&
            !rear->object_in_front && rear->rss_distance_m > 0.0,
        "뒤쪽 고속 객체의 외곽을 앞으로 확장합니다");
  object.predicted = {{1.0, 17.0, 8.0, 0.0, 0.0}};
  check(!planning::firstBehaviorPathObjectCollision(ego, {}, object),
        "횡방향 여유 밖의 객체는 안전합니다");
  object.predicted = {{3.0, 5.0, 0.0, 0.0, 0.0}};
  check(!planning::firstBehaviorPathObjectCollision(ego, {}, object),
        "자차 예측 시간 밖의 객체 표본은 비교하지 않습니다");
  object.predicted = {{1.0, 5.0, 0.0, 1.0, 0.0}};
  planning::BehaviorSafetyRssParameters narrow_yaw;
  narrow_yaw.yaw_difference_threshold_rad = 0.2;
  check(!planning::firstBehaviorPathObjectCollision(
            ego, {}, object, narrow_yaw),
        "설정한 방향 차이 밖의 객체는 제외합니다");

  const std::vector<planning::BehaviorSafetyPose> stationary_ego{
      {0.0, 0.0, 0.0, 0.0, 0.0},
      {2.0, 0.0, 0.0, 0.0, 0.0}};
  object.length_m = 4.0;
  object.width_m = 4.0;
  object.predicted = {{1.0, 3.0, -2.0, 0.0, 0.0}};
  const auto bounding_box = planning::firstBehaviorPathObjectCollision(
      stationary_ego, {}, object);
  check(bounding_box && bounding_box->kind ==
            planning::BehaviorSafetyCollisionKind::DirectOverlap,
        "직사각형 객체 외형은 자차 외형과 직접 겹칩니다");
  object.footprint = {{-2.0, -2.0}, {2.0, -2.0}, {2.0, 2.0}};
  const auto box_with_unused_footprint =
      planning::firstBehaviorPathObjectCollision(stationary_ego, {}, object);
  check(box_with_unused_footprint && box_with_unused_footprint->kind ==
            planning::BehaviorSafetyCollisionKind::DirectOverlap,
        "직사각형 종류에서는 별도 다각형 외곽을 사용하지 않습니다");
  object.shape_type = 2;
  const auto triangle = planning::firstBehaviorPathObjectCollision(
      stationary_ego, {}, object);
  check(!triangle || triangle->kind !=
            planning::BehaviorSafetyCollisionKind::DirectOverlap,
        "다각형 객체의 빈 모서리를 직접 충돌로 잘못 세지 않습니다");
  object.length_m = 0.0;
  object.width_m = 0.0;
  const auto dimensionless_polygon = planning::behaviorSafetyWorldPolygon(
      {0.0, 20.0, 0.0, 0.0, 0.0}, object);
  object.predicted = {{1.0, 20.0, 0.0, 0.0, 0.0}};
  check(dimensionless_polygon.size() >= 3 &&
            !planning::firstBehaviorPathObjectCollision(
                stationary_ego, {}, object),
        "다각형 외곽이 있으면 기록된 0m 크기 값도 허용합니다");
  object.shape_type = 0;
  object.footprint.clear();
  object.length_m = 2.0;
  object.width_m = 2.0;
  object.predicted = {{1.0, 4.5, 1.8, 0.0, 0.0}};
  const auto box_corner = planning::firstBehaviorPathObjectCollision(
      stationary_ego, {}, object);
  check(box_corner && box_corner->kind ==
            planning::BehaviorSafetyCollisionKind::DirectOverlap,
        "객체 직사각형 모서리는 자차 외곽에 직접 닿습니다");
  object.shape_type = 1;
  const auto circle_corner = planning::firstBehaviorPathObjectCollision(
      stationary_ego, {}, object);
  check(!circle_corner || circle_corner->kind !=
            planning::BehaviorSafetyCollisionKind::DirectOverlap,
        "원기둥의 빈 모서리를 직접 충돌로 잘못 세지 않습니다");
  planning::BehaviorSafetyObject corner_object;
  corner_object.length_m = 1.0;
  corner_object.width_m = 1.0;
  corner_object.predicted = {{0.5, 5.5, 2.5, 0.0, 0.0}};
  planning::BehaviorSafetyRssParameters corner_rss;
  corner_rss.reaction_time_s = 0.0;
  corner_rss.safety_time_margin_s = 0.0;
  corner_rss.minimum_longitudinal_m = 6.0;
  corner_rss.lateral_margin_m = 0.0;
  check(!planning::firstBehaviorPathObjectCollision(
            stationary_ego, {}, corner_object, corner_rss),
        "직선 확장 외곽은 꺾인 경로 옆 객체와 겹치지 않습니다");
  corner_rss.extension_policy =
      planning::BehaviorSafetyExtensionPolicy::AlongPath;
  const std::vector<planning::BehaviorSafetyPose> corner_path{
      {0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 4.0, 0.0, 0.0, 0.0},
      {0.0, 4.0, 8.0, 1.5707963267948966, 0.0}};
  const auto along_corner = planning::firstBehaviorPathObjectCollision(
      stationary_ego, {}, corner_object, corner_rss, {}, corner_path);
  check(along_corner && along_corner->kind ==
            planning::BehaviorSafetyCollisionKind::ExtendedOverlap,
        "경로를 따라 꺾은 RSS 외곽에서 객체 확장 충돌을 찾습니다");
  auto beyond_path = corner_object;
  beyond_path.predicted = {{0.5, 5.0, 0.0, 0.0, 0.0}};
  const std::vector<planning::BehaviorSafetyPose> short_path{
      {0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 1.0, 0.0, 0.0, 0.0}};
  check(planning::firstBehaviorPathObjectCollision(
            stationary_ego, {}, beyond_path, corner_rss, {}, short_path)
            .has_value(),
        "앞으로 늘릴 경로가 부족하면 원본처럼 직사각형 확장으로 돌아갑니다");
  object.width_m = 0.0;
  check(planning::behaviorSafetyWorldPolygon(object.predicted.front(), object).size() >= 6,
        "원기둥 외형은 사용하지 않는 폭 값이 0이어도 만듭니다");
  object.shape_type = 0;
  object.length_m = 4.0;
  object.width_m = 1.8;
  const std::vector<planning::BehaviorSafetyPredictedPath> alternatives{
      {0.2, {{1.0, 5.0, 0.0, 0.0, 0.0}}},
      {0.8, {{1.0, 5.0, 10.0, 0.0, 0.0}}}};
  check(!planning::firstBehaviorPathObjectCollisionAcrossPaths(
            ego, {}, object, alternatives, false),
        "최고 신뢰도 예측 경로 하나만 고릅니다");
  const auto all_paths = planning::firstBehaviorPathObjectCollisionAcrossPaths(
      ego, {}, object, alternatives, true);
  check(all_paths && all_paths->kind ==
            planning::BehaviorSafetyCollisionKind::DirectOverlap &&
            all_paths->predicted_path_index == 0,
        "모든 예측 경로 검사는 낮은 신뢰도 충돌도 찾습니다");
  object.length_m = 4.0;
  object.width_m = 2.0;
  const auto turned_box = planning::behaviorSafetyWorldPolygon(
      {0.0, 10.0, 20.0, std::acos(-1.0) * 0.5, 0.0}, object);
  double minimum_x = 1.0e9, maximum_x = -1.0e9;
  double minimum_y = 1.0e9, maximum_y = -1.0e9;
  for (const auto &[x, y] : turned_box) {
    minimum_x = std::min(minimum_x, x);
    maximum_x = std::max(maximum_x, x);
    minimum_y = std::min(minimum_y, y);
    maximum_y = std::max(maximum_y, y);
  }
  check(turned_box.size() == 5 &&
            std::abs(minimum_x - 9.0) < 1.0e-9 &&
            std::abs(maximum_x - 11.0) < 1.0e-9 &&
            std::abs(minimum_y - 18.0) < 1.0e-9 &&
            std::abs(maximum_y - 22.0) < 1.0e-9,
        "인식 직사각형을 객체 방향에 따라 세계 좌표로 회전합니다");
  object.shape_type = 1;
  object.length_m = 2.0;
  const auto turned_cylinder = planning::behaviorSafetyWorldPolygon(
      {0.0, 10.0, 20.0, std::acos(-1.0) * 0.5, 0.0}, object);
  const auto fixed_cylinder = planning::behaviorSafetyWorldPolygon(
      {0.0, 10.0, 20.0, 0.0, 0.0}, object);
  check(turned_cylinder.size() == 7 &&
            turned_cylinder == fixed_cylinder,
        "원본 원기둥 육각형은 객체 방향과 관계없이 30도에서 시작합니다");
  object.shape_type = 0;
  object.length_m = 2.0;
  object.width_m = 2.0;
  const auto nearby = planning::behaviorSafetyWorldPolygon(
      {0.0, 6.0, 0.0, 0.0, 0.0}, object);
  check(!planning::behaviorFootprintCollidesWithObjects(
            {0.0, 0.0, 0.0, 0.0, 0.0}, {}, {nearby}, 1.2) &&
            planning::behaviorFootprintCollidesWithObjects(
                {0.0, 0.0, 0.0, 0.0, 0.0}, {}, {nearby}, 1.22),
        "차량 외곽에서 1.21m 떨어진 정지 객체의 여유 경계를 구분합니다");
  const auto overlapping = planning::behaviorSafetyWorldPolygon(
      {0.0, 3.0, 0.0, 0.0, 0.0}, object);
  check(planning::behaviorFootprintCollidesWithObjects(
            {0.0, 0.0, 0.0, 0.0, 0.0}, {}, {overlapping}, 0.2) &&
            !planning::behaviorFootprintCollidesWithObjects(
                {0.0, 0.0, 0.0, 0.0, 0.0}, {}, {overlapping}, 0.0),
        "원본의 엄격한 거리 미만 비교를 유지합니다");
  object.shape_type = 0;
  object.length_m = 4.0;
  object.width_m = 1.8;
  object.predicted = {{0.5, 2.5, 0.0, 0.0, 0.0},
                      {1.5, 7.5, 0.0, 0.0, 0.0}};
  const planning::BehaviorSafetyRegions target_lane{
      {{6.0, -3.0}, {11.0, -3.0}, {11.0, 3.0}, {6.0, 3.0}}};
  const auto in_lane = planning::firstBehaviorPathObjectCollision(
      ego, {}, object, {}, target_lane);
  check(in_lane && in_lane->object_sample_index == 1 &&
            in_lane->kind ==
                planning::BehaviorSafetyCollisionKind::DirectOverlap,
        "차로 밖의 첫 충돌은 건너뛰고 차로 안의 뒤쪽 표본을 선택합니다");
  const planning::BehaviorSafetyRegions away_lane{
      {{20.0, -3.0}, {25.0, -3.0}, {25.0, 3.0}, {20.0, 3.0}}};
  check(!planning::firstBehaviorPathObjectCollision(
            ego, {}, object, {}, away_lane),
        "충돌 객체 다각형이 차로 밖이면 안전으로 판정합니다");
  return EXIT_SUCCESS;
}
