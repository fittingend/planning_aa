// Copyright 2024 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 적색 신호 정지선 뒤의 예측 경로를 제거하는 계산입니다.

#include "planning/out_of_lane_core.hpp"

#include <boost/geometry.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {
namespace {
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Line = bg::model::linestring<Point>;
using Polygon = bg::model::polygon<Point>;

Polygon asPolygon(const OutOfLanePolygon &vertices) {
  Polygon polygon;
  for (const auto &vertex : vertices)
    polygon.outer().emplace_back(vertex.x, vertex.y);
  if (vertices.size() >= 3) bg::correct(polygon);
  return polygon;
}

Line asLine(const std::vector<MotionVelocityPoint2D> &vertices) {
  Line line;
  for (const auto &vertex : vertices)
    line.emplace_back(vertex.x, vertex.y);
  return line;
}
}  // 익명 네임스페이스

OutOfLanePredictedPath cutOutOfLanePredictedPathAtStopLines(
    OutOfLanePredictedPath path,
    const std::vector<OutOfLaneStopLine> &stop_lines,
    const double object_front_overhang) {
  if (!std::isfinite(object_front_overhang) || object_front_overhang < 0.0)
    throw std::invalid_argument("객체 앞쪽 길이가 유효하지 않습니다");
  if (path.poses.size() < 2) return path;
  std::size_t earliest_segment = path.poses.size();
  const OutOfLaneStopLine *chosen = nullptr;
  for (const auto &stop_line : stop_lines) {
    if (stop_line.vertices.size() < 2 || stop_line.lane_polygon.size() < 3)
      continue;
    const auto line = asLine(stop_line.vertices);
    const auto lane = asPolygon(stop_line.lane_polygon);
    for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
      const auto &first = path.poses[i];
      const auto &second = path.poses[i + 1];
      const Line segment{{first.x, first.y}, {second.x, second.y}};
      if (bg::intersects(segment, line) &&
          bg::within(Point(first.x, first.y), lane)) {
        if (i < earliest_segment) {
          earliest_segment = i;
          chosen = &stop_line;
        }
        break;
      }
    }
  }
  if (!chosen) return path;
  // 원본은 교차점의 다음 경로점을 선택한 뒤 앞쪽 길이만큼 뒤로 걷습니다.
  std::size_t cut_index = earliest_segment + 1;
  double arc_length = 0.0;
  while (cut_index > 0 && arc_length < object_front_overhang) {
    arc_length += std::hypot(
        path.poses[cut_index].x - path.poses[cut_index - 1].x,
        path.poses[cut_index].y - path.poses[cut_index - 1].y);
    --cut_index;
  }
  path.poses.resize(cut_index);
  return path;
}

}  // planning 네임스페이스
