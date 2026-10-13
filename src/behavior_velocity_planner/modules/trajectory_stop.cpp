// Copyright 2022 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 경로 중간의 정지점과 자세를 보간합니다.

#include "planning/trajectory_stop.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace planning {

TrajectoryStopResult insertZeroVelocityAtArc(
    const Trajectory &trajectory, double stop_arc_m) {
  if (trajectory.empty() || !std::isfinite(stop_arc_m)) {
    throw std::invalid_argument("정지점의 경로 입력이 잘못되었습니다");
  }
  TrajectoryStopResult result{trajectory, std::nullopt};
  if (stop_arc_m <= 0.0) {
    for (auto &point : result.trajectory) point.longitudinal_velocity_mps = 0.0F;
    result.stop_arc_m = 0.0;
    return result;
  }
  double traveled = 0.0;
  for (std::size_t i = 0; i + 1 < trajectory.size(); ++i) {
    const double length = std::hypot(
        trajectory[i + 1].x - trajectory[i].x,
        trajectory[i + 1].y - trajectory[i].y);
    if (length <= 1.0e-12) continue;
    if (stop_arc_m > traveled + length) {
      traveled += length;
      continue;
    }
    const double ratio = std::clamp((stop_arc_m - traveled) / length, 0.0, 1.0);
    std::size_t stop_index = i;
    if (ratio >= 1.0 - 1.0e-9) {
      stop_index = i + 1;
    } else if (ratio > 1.0e-9) {
      TrajectoryPoint point = result.trajectory[i];
      const auto &next = trajectory[i + 1];
      point.x += ratio * (next.x - point.x);
      point.y += ratio * (next.y - point.y);
      point.z += ratio * (next.z - point.z);
      double qx = next.orientation_x;
      double qy = next.orientation_y;
      double qz = next.orientation_z;
      double qw = next.orientation_w;
      const double dot = point.orientation_x * qx + point.orientation_y * qy +
                         point.orientation_z * qz + point.orientation_w * qw;
      if (dot < 0.0) {
        qx = -qx;
        qy = -qy;
        qz = -qz;
        qw = -qw;
      }
      point.orientation_x += ratio * (qx - point.orientation_x);
      point.orientation_y += ratio * (qy - point.orientation_y);
      point.orientation_z += ratio * (qz - point.orientation_z);
      point.orientation_w += ratio * (qw - point.orientation_w);
      const double norm = std::sqrt(
          point.orientation_x * point.orientation_x +
          point.orientation_y * point.orientation_y +
          point.orientation_z * point.orientation_z +
          point.orientation_w * point.orientation_w);
      if (norm > 1.0e-12) {
        point.orientation_x /= norm;
        point.orientation_y /= norm;
        point.orientation_z /= norm;
        point.orientation_w /= norm;
      }
      point.longitudinal_velocity_mps = static_cast<float>(
          trajectory[i].longitudinal_velocity_mps + ratio *
          (next.longitudinal_velocity_mps - trajectory[i].longitudinal_velocity_mps));
      point.acceleration_mps2 = static_cast<float>(
          trajectory[i].acceleration_mps2 + ratio *
          (next.acceleration_mps2 - trajectory[i].acceleration_mps2));
      result.trajectory.insert(result.trajectory.begin() + i + 1, point);
      stop_index = i + 1;
    }
    for (std::size_t j = stop_index; j < result.trajectory.size(); ++j) {
      result.trajectory[j].longitudinal_velocity_mps = 0.0F;
    }
    result.stop_arc_m = stop_arc_m;
    return result;
  }
  return result;
}

}  // planning 네임스페이스
