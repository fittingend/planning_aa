// 속도 방지턱 다각형과 차량 길이를 이용해 감속 구간을 계산합니다.

#pragma once

#include "planning/trajectory.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace planning {

struct SpeedBumpPoint {
  double x = 0.0;
  double y = 0.0;
};

struct SpeedBumpFeature {
  std::int64_t regulatory_id = 0;
  std::vector<SpeedBumpPoint> polygon;
  double height_m = 0.5;
  std::optional<double> speed_kph;
};

struct SpeedBumpParameters {
  double front_offset_m = 3.79;
  double rear_overhang_m = 1.1;
  double slow_start_margin_m = 1.0;
  double slow_end_margin_m = 1.0;
  double min_height_m = 0.05;
  double max_height_m = 0.30;
  double min_speed_mps = 1.39;
  double max_speed_mps = 2.78;
};

struct SpeedBumpResult {
  Trajectory trajectory;
  bool slowed = false;
};

SpeedBumpResult planSpeedBump(
    const Trajectory &path, const SpeedBumpFeature &feature,
    SpeedBumpParameters parameters = {});

}  // planning 네임스페이스
