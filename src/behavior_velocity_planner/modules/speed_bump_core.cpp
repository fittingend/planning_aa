// 원본 SpeedBump 모듈의 다각형 교차와 일정 속도 구간을 이관합니다.

#include "planning/speed_bump_core.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace planning {
namespace {

double cross(SpeedBumpPoint a, SpeedBumpPoint b) {
  return a.x * b.y - a.y * b.x;
}

bool within(SpeedBumpPoint point, const std::vector<SpeedBumpPoint> &polygon) {
  bool inside = false;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto a = polygon[i];
    const auto b = polygon[(i + 1) % polygon.size()];
    if ((a.y > point.y) != (b.y > point.y) &&
        point.x < a.x + (point.y - a.y) * (b.x - a.x) / (b.y - a.y)) {
      inside = !inside;
    }
  }
  return inside;
}

std::vector<double> arcs(const Trajectory &path) {
  std::vector<double> result(path.size());
  for (std::size_t i = 1; i < path.size(); ++i) {
    result[i] = result[i - 1] +
        std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return result;
}

std::size_t insertAt(Trajectory &path, double arc) {
  const auto lengths = arcs(path);
  auto upper = std::lower_bound(lengths.begin(), lengths.end(), arc);
  std::size_t index = static_cast<std::size_t>(upper - lengths.begin());
  if (index == lengths.size()) return lengths.size() - 1;
  if (index == 0 || std::abs(*upper - arc) < 0.05) return index;
  if (std::abs(lengths[index - 1] - arc) < 0.05) return index - 1;
  const double ratio = (arc - lengths[index - 1]) /
      (lengths[index] - lengths[index - 1]);
  auto point = path[index - 1];
  point.x += ratio * (path[index].x - point.x);
  point.y += ratio * (path[index].y - point.y);
  point.z += ratio * (path[index].z - point.z);
  point.longitudinal_velocity_mps = static_cast<float>(
      point.longitudinal_velocity_mps + ratio *
      (path[index].longitudinal_velocity_mps - point.longitudinal_velocity_mps));
  path.insert(path.begin() + index, point);
  return index;
}

}  // 익명 네임스페이스

SpeedBumpResult planSpeedBump(
    const Trajectory &path, const SpeedBumpFeature &feature,
    SpeedBumpParameters parameters) {
  SpeedBumpResult result{path, false};
  if (path.size() < 2 || feature.polygon.size() < 3) return result;
  const auto lengths = arcs(path);
  std::vector<double> crossings;
  for (std::size_t i = 0; i + 1 < path.size() && crossings.size() < 2; ++i) {
    const SpeedBumpPoint a{path[i].x, path[i].y};
    const SpeedBumpPoint segment{path[i + 1].x - a.x, path[i + 1].y - a.y};
    for (std::size_t j = 0; j < feature.polygon.size(); ++j) {
      const auto b = feature.polygon[j];
      const auto end = feature.polygon[(j + 1) % feature.polygon.size()];
      const SpeedBumpPoint edge{end.x - b.x, end.y - b.y};
      const double denominator = cross(segment, edge);
      if (std::abs(denominator) < 1.0e-9) continue;
      const SpeedBumpPoint delta{b.x - a.x, b.y - a.y};
      const double t = cross(delta, edge) / denominator;
      const double u = cross(delta, segment) / denominator;
      if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
        crossings.push_back(lengths[i] + t * (lengths[i + 1] - lengths[i]));
      }
    }
  }
  std::sort(crossings.begin(), crossings.end());
  const bool first_inside = within({path.front().x, path.front().y}, feature.polygon);
  const bool last_inside = within({path.back().x, path.back().y}, feature.polygon);
  std::optional<double> entry, exit;
  if (crossings.size() >= 2) {
    entry = crossings[0];
    exit = crossings[1];
  } else if (crossings.size() == 1) {
    if (last_inside) entry = crossings[0];
    else if (first_inside) exit = crossings[0];
    else return result;
  } else if (!first_inside || !last_inside) {
    return result;
  }
  const double start = entry ? *entry - parameters.front_offset_m -
      parameters.slow_start_margin_m : 0.0;
  const double end = exit ? *exit + parameters.rear_overhang_m +
      parameters.slow_end_margin_m : lengths.back();
  if (start > lengths.back() || end < 0.0 || start > end) return result;
  const double slope = (parameters.min_speed_mps - parameters.max_speed_mps) /
      (parameters.max_height_m - parameters.min_height_m);
  const double speed = feature.speed_kph
      ? *feature.speed_kph / 3.6
      : std::clamp(parameters.max_speed_mps + slope *
          (feature.height_m - parameters.min_height_m),
          parameters.min_speed_mps, parameters.max_speed_mps);
  const auto start_index = insertAt(result.trajectory, std::max(0.0, start));
  const auto end_index = insertAt(
      result.trajectory, std::min(arcs(result.trajectory).back(), end));
  if (start_index > end_index) return SpeedBumpResult{path, false};
  for (std::size_t i = start_index; i <= end_index; ++i) {
    result.trajectory[i].longitudinal_velocity_mps = std::min<float>(
        result.trajectory[i].longitudinal_velocity_mps,
        static_cast<float>(speed));
  }
  result.slowed = true;
  return result;
}

}  // planning 네임스페이스
