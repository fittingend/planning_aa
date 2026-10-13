// SPDX-License-Identifier: Apache-2.0
// 일반 가림 영역의 감지 경로를 Akima 보간으로 1m 간격에 재표본화합니다.

#include "planning/occlusion_spot_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace planning {
namespace {

std::vector<double> akimaSlopes(const std::vector<double> &keys,
                               const std::vector<double> &values) {
  std::vector<double> segments;
  segments.reserve(keys.size() - 1);
  for (std::size_t index = 0; index + 1 < keys.size(); ++index)
    segments.push_back((values[index + 1] - values[index]) /
                       (keys[index + 1] - keys[index]));
  std::vector<double> slopes(keys.size(), 0.0);
  slopes.front() = segments.front();
  slopes.back() = segments.back();
  for (std::size_t index = 1; index + 1 < keys.size(); ++index) {
    if (index == 1 || index + 2 == keys.size()) {
      slopes[index] = (segments[index - 1] + segments[index]) / 2.0;
      continue;
    }
    const double first =
        std::abs(segments[index + 1] - segments[index]);
    const double second =
        std::abs(segments[index - 1] - segments[index - 2]);
    slopes[index] = first + second < 1.0e-5 ?
        (segments[index - 1] + segments[index]) / 2.0 :
        (first * segments[index - 1] +
         second * segments[index]) / (first + second);
  }
  return slopes;
}

double interpolateAkima(const std::vector<double> &keys,
                        const std::vector<double> &values,
                        const std::vector<double> &slopes,
                        std::size_t segment, double query) {
  const double length = keys[segment + 1] - keys[segment];
  const double secant =
      (values[segment + 1] - values[segment]) / length;
  const double a =
      (slopes[segment] + slopes[segment + 1] - 2.0 * secant) /
      (length * length);
  const double b =
      (3.0 * secant - 2.0 * slopes[segment] -
       slopes[segment + 1]) / length;
  const double ds = query - keys[segment];
  return values[segment] +
      (slopes[segment] + (b + a * ds) * ds) * ds;
}

}  // 익명 네임스페이스

Trajectory resampleOcclusionDetectionPath(
    const Trajectory &path, double maximum_length_m,
    double interval_m) {
  if (!std::isfinite(maximum_length_m) || maximum_length_m <= 0.0 ||
      !std::isfinite(interval_m) || interval_m < 0.1)
    throw std::invalid_argument("가림 감지 경로 보간 간격이 유효하지 않습니다");
  if (path.size() < 2) return path;
  Trajectory clipped;
  clipped.push_back(path.front());
  double length = 0.0;
  for (std::size_t index = 1; index < path.size(); ++index) {
    const double distance = std::hypot(
        path[index].x - clipped.back().x,
        path[index].y - clipped.back().y);
    if (!std::isfinite(distance))
      throw std::invalid_argument("가림 감지 경로 좌표가 유효하지 않습니다");
    if (distance < 1.0e-3) continue;
    if (length + distance > maximum_length_m) break;
    length += distance;
    clipped.push_back(path[index]);
  }
  if (clipped.size() < 2 || length / interval_m > 100000.0)
    return clipped;
  std::vector<double> keys(clipped.size(), 0.0);
  std::vector<double> x, y;
  x.reserve(clipped.size());
  y.reserve(clipped.size());
  x.push_back(clipped.front().x);
  y.push_back(clipped.front().y);
  for (std::size_t index = 1; index < clipped.size(); ++index) {
    keys[index] = keys[index - 1] + std::hypot(
        clipped[index].x - clipped[index - 1].x,
        clipped[index].y - clipped[index - 1].y);
    x.push_back(clipped[index].x);
    y.push_back(clipped[index].y);
  }
  const auto slope_x = akimaSlopes(keys, x);
  const auto slope_y = akimaSlopes(keys, y);
  std::vector<double> samples;
  for (double arc = 0.0; arc < keys.back(); arc += interval_m)
    samples.push_back(arc);
  if (samples.empty()) return clipped;
  if (keys.back() - samples.back() < 0.1)
    samples.back() = keys.back();
  else samples.push_back(keys.back());
  Trajectory result;
  result.reserve(samples.size());
  for (const double arc : samples) {
    const auto upper = std::upper_bound(keys.begin(), keys.end(), arc);
    const std::size_t segment = std::min(
        static_cast<std::size_t>(upper - keys.begin() - 1),
        clipped.size() - 2);
    const double fraction =
        (arc - keys[segment]) /
        (keys[segment + 1] - keys[segment]);
    auto point = clipped[segment];
    point.x = interpolateAkima(
        keys, x, slope_x, segment, arc);
    point.y = interpolateAkima(
        keys, y, slope_y, segment, arc);
    point.z = clipped[segment].z + fraction *
        (clipped[segment + 1].z - clipped[segment].z);
    if (arc == keys.back())
      point.longitudinal_velocity_mps =
          clipped.back().longitudinal_velocity_mps;
    result.push_back(point);
  }
  const double original_yaw = std::atan2(
      2.0 * (clipped.front().orientation_w *
                 clipped.front().orientation_z +
             clipped.front().orientation_x *
                 clipped.front().orientation_y),
      1.0 - 2.0 * (clipped.front().orientation_y *
                       clipped.front().orientation_y +
                   clipped.front().orientation_z *
                       clipped.front().orientation_z));
  const bool forward =
      std::cos(original_yaw) *
              (clipped[1].x - clipped[0].x) +
          std::sin(original_yaw) *
              (clipped[1].y - clipped[0].y) >= 0.0;
  for (std::size_t index = 0; index < result.size(); ++index) {
    const auto previous = index == 0 ? 0 : index - 1;
    const auto next = std::min(index + 1, result.size() - 1);
    const double yaw = std::atan2(
        result[next].y - result[previous].y,
        result[next].x - result[previous].x) +
        (forward ? 0.0 : std::acos(-1.0));
    result[index].orientation_x = 0.0;
    result[index].orientation_y = 0.0;
    result[index].orientation_z = std::sin(yaw / 2.0);
    result[index].orientation_w = std::cos(yaw / 2.0);
  }
  return result;
}

}  // planning 네임스페이스
