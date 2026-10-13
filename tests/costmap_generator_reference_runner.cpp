// SPDX-License-Identifier: Apache-2.0
// 원본 costmap probe와 같은 지도·주차 활성 상태의 격자를 독립 생성합니다.

#include "planning/costmap_map_adapter.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 5 && argc != 8 && argc != 10 && argc != 11 &&
      argc != 12 && argc != 13 && argc != 14 && argc != 15) return 2;
  try {
    const bool second_point = argc == 12 &&
        std::string(argv[8]) == "--second-point";
    std::ifstream map_file(argv[1], std::ios::binary);
    if (!map_file) throw std::runtime_error("지도 파일이 없습니다");
    const std::vector<std::uint8_t> map{
        std::istreambuf_iterator<char>(map_file),
        std::istreambuf_iterator<char>()};
    planning::CostmapGeneratorParameters parameters;
    parameters.activate_by_scenario = true;
    parameters.use_objects = argc == 10 || argc == 11 ||
                             (argc == 12 && !second_point) ||
                             argc == 13 || argc == 14 || argc == 15;
    parameters.use_points = argc == 8 || second_point || argc == 13 ||
                            argc == 14 || argc == 15;
    planning::CostmapMapAdapter generator(map, parameters);
    planning::CostmapGeneratorInput input;
    input.ego_x = std::stod(argv[2]);
    input.ego_y = std::stod(argv[3]);
    input.ego_z = std::stod(argv[4]);
    input.parking_activated = true;
    if (parameters.use_objects) {
      planning::CostmapObject object;
      object.shape_type = (argc == 11 || argc == 12 ||
                           argc == 14 || argc == 15) ?
          std::stoi(argv[parameters.use_points ? 13 : 10]) : 0;
      const bool asymmetric_polygon = object.shape_type == 3;
      if (asymmetric_polygon) object.shape_type = 2;
      object.center_x = std::stod(argv[5]);
      object.center_y = std::stod(argv[6]);
      object.length_m = std::stod(argv[7]);
      object.width_m = std::stod(argv[8]);
      object.classification_probability = std::stod(argv[9]);
      if (argc == 12 || argc == 15)
        object.yaw_rad = std::stod(argv[parameters.use_points ? 14 : 11]);
      if (object.shape_type == 2) {
        const double half_length = object.length_m * 0.5;
        const double half_width = object.width_m * 0.5;
        for (const auto &[dx, dy] :
             std::vector<std::pair<double, double>>{
                 {half_length, half_width}, {half_length, -half_width},
                 {-half_length, asymmetric_polygon ? -half_width * 0.5 : -half_width},
                 {-half_length, asymmetric_polygon ? half_width * 0.5 : half_width}})
          object.outline.emplace_back(object.center_x + dx,
                                      object.center_y + dy);
      }
      input.objects_map_frame.push_back(object);
    }
    if (parameters.use_points) {
      const int first = parameters.use_objects ? 10 : 5;
      planning::CostmapPoint3D point;
      // The probe serializes sensor_msgs/PointCloud2 XYZ as float32.
      point.x = static_cast<float>(std::stod(argv[first]));
      point.y = static_cast<float>(std::stod(argv[first + 1]));
      point.z = static_cast<float>(std::stod(argv[first + 2]));
      input.points_map_frame.push_back(point);
      if (second_point) {
        planning::CostmapPoint3D point2;
        point2.x = static_cast<float>(std::stod(argv[9]));
        point2.y = static_cast<float>(std::stod(argv[10]));
        point2.z = static_cast<float>(std::stod(argv[11]));
        input.points_map_frame.push_back(point2);
      }
    }
    const auto result = generator.tick(input);
    if (!result) throw std::runtime_error("독립 격자를 생성하지 못했습니다");
    const auto &grid = result->occupancy;
    std::cout << std::setprecision(17)
              << grid.width << ' ' << grid.height << ' '
              << grid.resolution_m << ' ' << grid.origin_x << ' '
              << grid.origin_y << '\n';
    for (const auto value : grid.values)
      std::cout << static_cast<int>(value) << ' ';
    std::cout << '\n';
    for (const auto *layer : {&result->points, &result->objects,
                              &result->primitives, &result->combined}) {
      for (const auto value : *layer)
        std::cout << value << ' ';
      std::cout << '\n';
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
