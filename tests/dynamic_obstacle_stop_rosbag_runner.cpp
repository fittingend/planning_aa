// SPDX-License-Identifier: Apache-2.0
// 실측 예측 객체를 동적 장애물 정지 필터와 연속 결정 상태에 전달합니다.

#include "planning/dynamic_obstacle_stop_core.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace {
planning::DynamicObjectType objectType(const int label) {
  using Type = planning::DynamicObjectType;
  switch (label) {
    case 1: return Type::Car;
    case 2: return Type::Truck;
    case 3: return Type::Bus;
    case 4: return Type::Trailer;
    case 5: return Type::Motorcycle;
    case 6: return Type::Bicycle;
    case 7: return Type::Pedestrian;
    default: return Type::Unknown;
  }
}
}  // 익명 네임스페이스

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  std::ifstream input(argv[1]);
  std::ofstream output(argv[2]);
  if (!input || !output) return 2;
  std::size_t frame_count = 0;
  input >> frame_count;
  planning::DynamicObstacleStopCore planner;
  const planning::DynamicObstacleStopParameters parameters;
  for (std::size_t frame = 0; frame < frame_count; ++frame) {
    std::size_t point_count = 0, object_count = 0;
    double now = 0.0, ego_speed = 0.0, ego_acceleration = 0.0;
    planning::TrajectoryPoint ego;
    input >> point_count >> object_count >> now >> ego.x >> ego.y >>
        ego.orientation_z >> ego.orientation_w >> ego_speed >>
        ego_acceleration;
    planning::Trajectory path(point_count);
    for (auto &point : path)
      input >> point.x >> point.y >> point.orientation_z >>
          point.orientation_w >> point.longitudinal_velocity_mps;
    std::vector<planning::DynamicObject> objects(object_count);
    for (auto &object : objects) {
      int label = 0;
      input >> object.id >> label >> object.x >> object.y >> object.yaw >>
          object.velocity >> object.length >> object.width;
      object.type = objectType(label);
    }
    if (!input || path.size() < 2)
      throw std::runtime_error("동적 장애물 정지 기록 입력이 잘못되었습니다");
    path = planning::resampleMotionVelocityPlanningInput(path);
    const auto filtered = planning::filterDynamicStopObjects(
        objects, path, ego, 0.0, std::nullopt, parameters);
    const auto collisions = planning::findDynamicStopCollisions(
        path, {ego.x, ego.y}, filtered, parameters);
    const double braking_distance = planning::dynamicStopMinimumBrakingDistance(
        std::max(0.0, ego_speed), ego_acceleration, -1.0, 1.0, -1.0);
    const auto plan = planner.plan(path, ego, objects, braking_distance,
                                   0.0, now);
    output << objects.size() << ' ' << filtered.size() << ' '
           << collisions.size() << ' ' << plan.stop_points.size() << '\n';
  }
}
