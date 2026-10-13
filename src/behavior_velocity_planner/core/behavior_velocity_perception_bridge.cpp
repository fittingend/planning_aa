// SPDX-License-Identifier: Apache-2.0
// 예측 객체의 종류·외형·예측 경로를 플러그인별 자료형으로 복사합니다.

#include "planning/behavior_velocity_perception_bridge.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace planning {

void applyBehaviorVelocityPerceptionBatch(
    BehaviorVelocityMapInput &input,
    const std::vector<PlanningPerceivedObject> &objects) {
  input.crosswalk_objects.clear();
  input.dynamic_stop_objects.clear();
  input.crosswalk_occlusion_mask_objects.clear();
  input.occlusion_spot_vehicles.clear();
  input.crosswalk_stuck_vehicles.clear();
  input.intersection_stuck_vehicles.clear();
  input.intersection_predicted_vehicles.clear();
  input.no_stopping_vehicles.clear();
  input.blind_spot_map_objects.clear();
  input.run_out_objects.clear();
  input.crosswalk_objects.reserve(objects.size());
  for (const auto &object : objects) {
    if (!std::isfinite(object.x) || !std::isfinite(object.y) ||
        !std::isfinite(object.yaw_rad) ||
        !std::isfinite(object.velocity_x_mps) ||
        !std::isfinite(object.velocity_y_mps) ||
        !std::isfinite(object.length_m) ||
        !std::isfinite(object.width_m)) {
      throw std::invalid_argument("행동 속도 인식 객체 값이 유효하지 않습니다");
    }
    const double speed = std::hypot(object.velocity_x_mps,
                                    object.velocity_y_mps);
    const bool vehicle = object.classification <= 3;
    BehaviorCrosswalkObject crosswalk;
    crosswalk.id = object.id;
    crosswalk.position = {object.x, object.y};
    crosswalk.classification = object.classification;
    crosswalk.predicted.length_m = object.length_m;
    crosswalk.predicted.width_m = object.width_m;
    crosswalk.predicted.velocity_x_mps = object.velocity_x_mps;
    crosswalk.predicted.velocity_y_mps = object.velocity_y_mps;
    for (const auto &path : object.predicted_paths) {
      std::vector<CrosswalkPose2d> poses;
      poses.reserve(path.poses.size());
      for (const auto &pose : path.poses) {
        poses.push_back({pose.x, pose.y, pose.yaw_rad});
      }
      crosswalk.predicted.predicted_paths.push_back(std::move(poses));
    }
    input.crosswalk_objects.push_back(std::move(crosswalk));
    DynamicObject dynamic;
    dynamic.id = object.id;
    switch (object.classification) {
      case 1: dynamic.type = DynamicObjectType::Car; break;
      case 2: dynamic.type = DynamicObjectType::Truck; break;
      case 3: dynamic.type = DynamicObjectType::Bus; break;
      case 4: dynamic.type = DynamicObjectType::Trailer; break;
      case 5: dynamic.type = DynamicObjectType::Motorcycle; break;
      case 6: dynamic.type = DynamicObjectType::Bicycle; break;
      case 7: dynamic.type = DynamicObjectType::Pedestrian; break;
      default: dynamic.type = DynamicObjectType::Unknown; break;
    }
    dynamic.x = object.x;
    dynamic.y = object.y;
    dynamic.yaw = object.yaw_rad;
    dynamic.velocity = object.velocity_x_mps;
    dynamic.length = object.length_m;
    dynamic.width = object.width_m;
    if (dynamic.length > 0.0 && dynamic.width > 0.0) {
      input.dynamic_stop_objects.push_back(std::move(dynamic));
    }
    input.crosswalk_occlusion_mask_objects.push_back(
        {{object.x, object.y, object.yaw_rad},
         object.length_m, object.width_m,
         object.velocity_x_mps});
    if (object.classification >= 1 && object.classification <= 4 &&
        object.length_m > 0.0 && object.width_m > 0.0) {
      input.occlusion_spot_vehicles.push_back(
          {{object.x, object.y, object.yaw_rad},
           object.length_m, object.width_m, speed});
    }
    if (vehicle) {
      input.crosswalk_stuck_vehicles.push_back(
          {{object.x, object.y}, object.length_m, speed, true});
      input.intersection_stuck_vehicles.push_back(
          {{object.x, object.y}, speed, true, object.classification});
      input.no_stopping_vehicles.push_back(
          {{object.x, object.y}, object.yaw_rad,
           object.length_m, object.width_m, speed,
           object.classification, true});
      IntersectionPredictedVehicle predicted;
      predicted.length_m = object.length_m;
      predicted.width_m = object.width_m;
      predicted.classification = object.classification;
      predicted.initial_pose = CrosswalkPose2d{
          object.x, object.y, object.yaw_rad};
      predicted.initial_speed_mps = speed;
      if (!object.predicted_paths.empty()) {
        const auto &path = object.predicted_paths.front();
        predicted.time_step_s = path.time_step_s;
        predicted.confidence = path.confidence;
        for (const auto &pose : path.poses) {
          predicted.predicted_poses.push_back(
              {pose.x, pose.y, pose.yaw_rad});
        }
      }
      input.intersection_predicted_vehicles.push_back(std::move(predicted));
    }
    input.blind_spot_map_objects.push_back(
        {object.id, object.classification, {object.x, object.y},
         object.velocity_x_mps});
    RunOutCollisionObject run_out;
    run_out.id = object.id;
    run_out.pose = {object.x, object.y, object.yaw_rad};
    run_out.predicted_paths = object.predicted_paths;
    run_out.classification = object.classification;
    run_out.shape_type = object.shape_type;
    run_out.length_m = object.length_m;
    run_out.width_m = object.width_m;
    run_out.minimum_velocity_mps = speed;
    run_out.maximum_velocity_mps = speed;
    input.run_out_objects.push_back(std::move(run_out));
  }
}

}  // planning 네임스페이스
