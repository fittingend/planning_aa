// Copyright 2020 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 차로 주행 정지선 모듈의 경로 속도 변경과 상태 계산입니다.

#pragma once

#include "planning/trajectory.hpp"

#include <optional>

namespace planning {

struct StopLineSegment {
  double x1 = 0.0;
  double y1 = 0.0;
  double x2 = 0.0;
  double y2 = 0.0;
};

struct StopLineParameters {
  double stop_margin_m = 0.0;
  double stop_duration_s = 1.0;
  double hold_stop_margin_m = 2.0;
  double front_offset_m = 3.79;
  double line_extension_m = 5.0;
  bool linear_insert = false;
  bool copy_previous_orientation = false;
};

enum class StopLineState { Approach, Stopped, Start };

struct StopLineResult {
  Trajectory trajectory;
  std::optional<double> stop_arc_m;
  double ego_arc_m = 0.0;
  StopLineState state = StopLineState::Approach;
};

class StopLineCore {
 public:
  StopLineCore(StopLineSegment line, StopLineParameters parameters = {});

  StopLineResult apply(
      const Trajectory &path, double ego_x, double ego_y,
      bool vehicle_stopped, double now_s);
  StopLineState state() const noexcept { return state_; }

 private:
  StopLineSegment line_;
  StopLineParameters parameters_;
  StopLineState state_ = StopLineState::Approach;
  std::optional<double> stopped_since_s_;
};

}  // planning 네임스페이스
