// Copyright 2020 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 감지 영역의 장애물 관측 후 2초 정지 유지와 정지점 생성을 계산합니다.

#include "planning/detection_area_core.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace planning {
namespace {

bool within(CrosswalkPoint point, const std::vector<CrosswalkPoint> &polygon) {
  if (polygon.size() < 3) return false;
  bool inside = false;
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const auto first = polygon[index];
    const auto second = polygon[(index + 1) % polygon.size()];
    if ((first.y > point.y) != (second.y > point.y) &&
        point.x < first.x + (point.y - first.y) *
                      (second.x - first.x) / (second.y - first.y)) {
      inside = !inside;
    }
  }
  return inside;
}

}  // 익명 네임스페이스

DetectionAreaCore::DetectionAreaCore(
    StopLineSegment stop_line,
    std::vector<std::vector<CrosswalkPoint>> detection_areas,
    DetectionAreaParameters parameters)
    : stop_line_(stop_line),
      detection_areas_(std::move(detection_areas)),
      parameters_(parameters) {
  if (!std::isfinite(parameters_.state_clear_time_s) ||
      !std::isfinite(parameters_.stop_margin_m) ||
      !std::isfinite(parameters_.front_offset_m) ||
      !std::isfinite(parameters_.stop_line_extension_m) ||
      !std::isfinite(parameters_.distance_to_judge_over_stop_line_m) ||
      parameters_.state_clear_time_s < 0.0 ||
      parameters_.front_offset_m < 0.0 ||
      parameters_.stop_line_extension_m < 0.0 ||
      parameters_.distance_to_judge_over_stop_line_m < 0.0) {
    throw std::invalid_argument("감지 영역 설정이 잘못되었습니다");
  }
}

DetectionAreaResult DetectionAreaCore::apply(
    const Trajectory &path, double ego_x, double ego_y,
    double now_s, const std::vector<CrosswalkPoint> &obstacle_points) {
  if (!std::isfinite(now_s)) {
    throw std::invalid_argument("감지 영역 시각에는 유한한 값이 필요합니다");
  }
  DetectionAreaResult result;
  result.trajectory = path;
  for (const auto &point : obstacle_points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      throw std::invalid_argument("감지 영역 장애물 위치가 잘못되었습니다");
    }
    for (const auto &area : detection_areas_) {
      if (within(point, area)) {
        last_obstacle_found_s_ = now_s;
        break;
      }
    }
  }
  if (!last_obstacle_found_s_ ||
      now_s - *last_obstacle_found_s_ >= parameters_.state_clear_time_s ||
      now_s < *last_obstacle_found_s_) {
    last_obstacle_found_s_.reset();
    stopped_ = false;
    return result;
  }
  StopLineParameters stop_parameters;
  stop_parameters.stop_margin_m = parameters_.stop_margin_m;
  stop_parameters.front_offset_m = parameters_.front_offset_m;
  stop_parameters.line_extension_m = parameters_.stop_line_extension_m;
  stop_parameters.linear_insert = parameters_.linear_insert;
  stop_parameters.copy_previous_orientation =
      parameters_.copy_previous_orientation;
  StopLineCore stop_line(stop_line_, stop_parameters);
  const auto stopped = stop_line.apply(path, ego_x, ego_y, false, now_s);
  if (!stopped.stop_arc_m) return result;
  if (!stopped_ &&
      stopped.ego_arc_m - *stopped.stop_arc_m >
          parameters_.distance_to_judge_over_stop_line_m) return result;
  stopped_ = true;
  result.trajectory = stopped.trajectory;
  result.stop_arc_m = stopped.stop_arc_m;
  result.stopped = true;
  return result;
}

}  // planning 네임스페이스
