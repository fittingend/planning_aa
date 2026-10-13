// Copyright 2022 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 점유 격자 윤곽 추출의 ROS 없는 자료 변환입니다.

#include "planning/obstacle_velocity_limiter_grid.hpp"

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {
namespace {
bool insidePolygon(MotionVelocityPoint2D point,
                   const VelocityLimiterPolygon &polygon) {
  if (polygon.size() < 3) return false;
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1;
       i < polygon.size(); j = i++) {
    const auto &a = polygon[i];
    const auto &b = polygon[j];
    const double cross = (point.x - a.x) * (b.y - a.y) -
                         (point.y - a.y) * (b.x - a.x);
    const double dot = (point.x - a.x) * (point.x - b.x) +
                       (point.y - a.y) * (point.y - b.y);
    if (std::abs(cross) < 1e-9 && dot <= 0.0) return true;
    if ((a.y > point.y) != (b.y > point.y) &&
        point.x < (b.x - a.x) * (point.y - a.y) /
                          (b.y - a.y) + a.x)
      inside = !inside;
  }
  return inside;
}
}  // 익명 네임스페이스

VelocityLimiterGridObstacles extractVelocityLimiterGridObstacleGeometry(
    const VelocityLimiterOccupancyGrid &grid, const int threshold,
    const VelocityLimiterObstacleMasks &masks) {
  if (grid.width == 0 || grid.height == 0 ||
      grid.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      grid.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      grid.cells.size() != static_cast<std::size_t>(grid.width) * grid.height ||
      !std::isfinite(grid.resolution) || grid.resolution <= 0.0 ||
      !std::isfinite(grid.origin.x) || !std::isfinite(grid.origin.y))
    throw std::invalid_argument("장애물 점유 격자가 유효하지 않습니다");
  // grid_map 영상의 행은 지도 X를, 열은 지도 Y를 역방향으로 담습니다.
  cv::Mat image(static_cast<int>(grid.width),
                static_cast<int>(grid.height), CV_8UC1, cv::Scalar(0));
  for (std::uint32_t y = 0; y < grid.height; ++y) {
    for (std::uint32_t x = 0; x < grid.width; ++x) {
      if (grid.cells[static_cast<std::size_t>(y) * grid.width + x] < threshold)
        continue;
      const MotionVelocityPoint2D position{
          grid.origin.x + (x + 0.5) * grid.resolution,
          grid.origin.y + (y + 0.5) * grid.resolution};
      if (!masks.positive.empty() && !insidePolygon(position, masks.positive))
        continue;
      bool excluded = false;
      for (const auto &negative : masks.negative) {
        if (insidePolygon(position, negative)) {
          excluded = true;
          break;
        }
      }
      if (!excluded)
        image.at<std::uint8_t>(grid.width - 1 - x,
                               grid.height - 1 - y) = 127;
    }
  }
  cv::dilate(image, image, cv::Mat(), cv::Point(-1, -1), 2);
  cv::erode(image, image, cv::Mat(), cv::Point(-1, -1), 2);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
  VelocityLimiterGridObstacles obstacles;
  for (const auto &contour : contours) {
    const auto world = [&](const cv::Point &pixel) {
      return MotionVelocityPoint2D{
          (grid.width - 1.0 - pixel.y) * grid.resolution + grid.origin.x,
          (grid.height - 1.0 - pixel.x) * grid.resolution + grid.origin.y};
    };
    if (contour.size() == 1)
      obstacles.points.push_back(world(contour.front()));
    for (std::size_t i = 1; i < contour.size(); ++i) {
      obstacles.segments.push_back({world(contour[i - 1]),
                                    world(contour[i])});
    }
  }
  return obstacles;
}

std::vector<VelocityLimiterSegment> extractVelocityLimiterGridObstacles(
    const VelocityLimiterOccupancyGrid &grid, const int threshold,
    const VelocityLimiterObstacleMasks &masks) {
  return extractVelocityLimiterGridObstacleGeometry(
      grid, threshold, masks).segments;
}

MotionVelocityPlan planVelocityLimiterOccupancyGrid(
    const Trajectory &path, const std::vector<double> &time_from_start,
    const std::size_t ego_index, const double current_ego_velocity,
    const VelocityLimiterOccupancyGrid &grid, const int threshold,
    const VelocityLimiterObstacleMasks &masks,
    const ObstacleVelocityLimiterParameters &parameters,
    const BicycleVelocityProjectionParameters &bicycle_parameters) {
  auto prepared = path;
  if (parameters.calculate_steering_angles)
    calculateVelocityLimiterSteeringAngles(
        prepared, bicycle_parameters.wheel_base);
  const auto sampled = sampleObstacleVelocityLimiterPath(
      prepared, time_from_start, ego_index, parameters);
  const auto obstacles = extractVelocityLimiterGridObstacleGeometry(
      grid, threshold, masks);
  // 원본의 한 점 선문자열은 충돌 검사에서 교차점이 없어 감속에 사용되지 않습니다.
  if (parameters.model == VelocityLimiterProjectionModel::Bicycle)
    return planObstacleVelocityLimitBicycle(
        sampled.path, sampled.time_from_start, {}, obstacles.segments,
        current_ego_velocity, parameters, bicycle_parameters);
  return planObstacleVelocityLimitParticle(
      sampled.path, sampled.time_from_start, {}, obstacles.segments,
      current_ego_velocity, parameters);
}

}  // planning 네임스페이스
