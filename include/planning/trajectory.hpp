#pragma once

#include <vector>

namespace planning {

// 내부 작업용 자료형이며 브리지 계약을 정의하지 않습니다.
struct TrajectoryPoint {
  double x = 0.0;
  double y = 0.0;
  float longitudinal_velocity_mps = 0.0F;
  float acceleration_mps2 = 0.0F;
  double z = 0.0;
  double orientation_x = 0.0;
  double orientation_y = 0.0;
  double orientation_z = 0.0;
  double orientation_w = 1.0;
  float heading_rate_rps = 0.0F;
  float lateral_velocity_mps = 0.0F;
  float front_wheel_angle_rad = 0.0F;
  float rear_wheel_angle_rad = 0.0F;
};

using Trajectory = std::vector<TrajectoryPoint>;

}  // planning 네임스페이스
