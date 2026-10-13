#pragma once

// 순항 계획의 점군 복셀·군집·경로 충돌점 계산입니다.
// 좌표계 변환은 호출자가 끝낸 뒤 점을 전달합니다.

#include "planning/trajectory.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace planning {

struct CruisePoint3D {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct CruisePointcloudParameters {
  double voxel_x_m = 0.05;
  double voxel_y_m = 0.05;
  double voxel_z_m = 100000.0;
  double cluster_tolerance_m = 1.0;
  std::size_t minimum_cluster_size = 1;
  std::size_t maximum_cluster_size = 100000;
  double vehicle_width_m = 1.83;
  double stop_lateral_margin_m = 0.3;
  double slowdown_lateral_margin_m = 1.1;
};

struct CruisePointcloudObstacle {
  std::optional<CruisePoint3D> stop_collision_point;
  std::optional<CruisePoint3D> slowdown_front_collision_point;
  std::optional<CruisePoint3D> slowdown_back_collision_point;
  double distance_from_ego_m = 0.0;
  double signed_lateral_distance_m = 0.0;
  std::size_t cluster_size = 0;
};

std::vector<CruisePoint3D> voxelizeCruisePointcloud(
    const std::vector<CruisePoint3D> &points,
    const CruisePointcloudParameters &parameters = {});

std::vector<std::vector<std::size_t>> clusterCruisePointcloud(
    const std::vector<CruisePoint3D> &points,
    const CruisePointcloudParameters &parameters = {});

std::vector<CruisePointcloudObstacle> convertCruisePointcloudToObstacles(
    const std::vector<CruisePoint3D> &transformed_points,
    const Trajectory &trajectory, std::size_t ego_index,
    const CruisePointcloudParameters &parameters = {});

}  // planning 네임스페이스
