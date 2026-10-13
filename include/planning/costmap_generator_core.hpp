// SPDX-License-Identifier: Apache-2.0
// 주차 격자의 지도·점군·객체 layer를 ROS 없이 생성합니다.

#pragma once

#include "planning/behavior_path_object_safety.hpp"
#include "planning/crosswalk_occlusion.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace planning {

struct CostmapPoint3D {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct CostmapObject {
  BehaviorSafetyPolygon outline;
  double classification_probability = 1.0;
  // 0: bounding box, 1: cylinder (source also uses box), 2: polygon.
  // -1 keeps a caller-supplied generic world outline.
  int shape_type = -1;
  double center_x = 0.0;
  double center_y = 0.0;
  double yaw_rad = 0.0;
  double length_m = 0.0;
  double width_m = 0.0;
};

struct CostmapGeneratorParameters {
  double length_x_m = 70.0;
  double length_y_m = 70.0;
  double resolution_m = 0.3;
  double minimum_point_height_m = 0.0;
  double maximum_point_height_m = 2.5;
  double object_expansion_m = 0.5;
  int object_blur_kernel_cells = 9;
  bool use_wayarea = true;
  bool use_parkinglot = true;
  bool use_objects = true;
  bool use_points = true;
  bool activate_by_scenario = false;
};

struct CostmapGeneratorInput {
  double stamp_s = 0.0;
  double ego_x = 0.0;
  double ego_y = 0.0;
  double ego_z = 0.0;
  bool ego_in_parking_lot = false;
  bool parking_activated = false;
  std::vector<CostmapPoint3D> points_map_frame;
  std::vector<CostmapObject> objects_map_frame;
};

struct CostmapGeneratorResult {
  double stamp_s = 0.0;
  std::string frame_id = "map";
  CrosswalkOccupancyGrid occupancy;
  std::vector<float> primitives;
  std::vector<float> points;
  std::vector<float> objects;
  std::vector<float> combined;
  double center_x = 0.0;
  double center_y = 0.0;
};

class CostmapGeneratorCore {
 public:
  explicit CostmapGeneratorCore(CostmapGeneratorParameters parameters = {});
  void onMapAreas(std::vector<BehaviorSafetyPolygon> road,
                  std::vector<BehaviorSafetyPolygon> parking);
  // 점과 객체는 map frame으로 변환된 현재 polling 입력을 받습니다.
  std::optional<CostmapGeneratorResult> tick(
      const CostmapGeneratorInput &input);

 private:
  CostmapGeneratorParameters parameters_;
  std::vector<BehaviorSafetyPolygon> road_;
  std::vector<BehaviorSafetyPolygon> parking_;
  bool map_received_ = false;
  double center_x_ = 0.0;
  double center_y_ = 0.0;
};

}  // namespace planning
