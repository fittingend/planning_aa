// SPDX-License-Identifier: Apache-2.0
// 원본 돌발 진입 모듈의 예측 경로 없는 객체와 점군 객체 생성입니다.

#include "planning/run_out_obstacle_creator.hpp"
#include "planning/obstacle_cruise_collision.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace planning {
namespace {
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;

struct VoxelKey {
  std::int64_t x;
  std::int64_t y;
  bool operator==(const VoxelKey &other) const {
    return x == other.x && y == other.y;
  }
};
struct VoxelHash {
  std::size_t operator()(const VoxelKey &key) const {
    const auto a = std::hash<std::int64_t>{}(key.x);
    const auto b = std::hash<std::int64_t>{}(key.y);
    return a ^ (b + 0x9e3779b97f4a7c15ULL + (a << 6) + (a >> 2));
  }
};

double yaw(const TrajectoryPoint &point) {
  return std::atan2(
      2.0 * (point.orientation_w * point.orientation_z +
             point.orientation_x * point.orientation_y),
      1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                   point.orientation_z * point.orientation_z));
}

void validate(const RunOutCreatorParameters &p) {
  if (!std::isfinite(p.minimum_velocity_mps) ||
      !std::isfinite(p.maximum_velocity_mps) ||
      !std::isfinite(p.prediction_step_s) ||
      !std::isfinite(p.prediction_horizon_s) ||
      !std::isfinite(p.point_diameter_m) ||
      !std::isfinite(p.point_path_interval_m) ||
      !std::isfinite(p.voxel_size_m) ||
      p.minimum_velocity_mps < 0.0 ||
      p.maximum_velocity_mps < p.minimum_velocity_mps ||
      p.prediction_step_s <= 0.0 || p.prediction_horizon_s <= 0.0 ||
      p.point_diameter_m <= 0.0 || p.point_path_interval_m <= 0.0 ||
      p.voxel_size_m <= 0.0)
    throw std::invalid_argument("돌발 진입 객체 생성 매개변수가 유효하지 않습니다");
}

CruisePredictedPath straightPrediction(
    const CruisePredictedPose &pose, const RunOutCreatorParameters &p) {
  CruisePredictedPath prediction;
  prediction.time_step_s = p.prediction_step_s;
  prediction.confidence = 1.0;
  const auto count = static_cast<std::size_t>(
      p.prediction_horizon_s / p.prediction_step_s);
  prediction.poses.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const double distance = p.maximum_velocity_mps * p.prediction_step_s * i;
    prediction.poses.push_back({
        pose.x + distance * std::cos(pose.yaw_rad),
        pose.y + distance * std::sin(pose.yaw_rad), pose.yaw_rad});
  }
  return prediction;
}

std::size_t nearestPoint(const Trajectory &path, double x, double y) {
  std::size_t best_index = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double distance = std::hypot(path[i].x - x, path[i].y - y);
    if (distance < best) {
      best = distance;
      best_index = i;
    }
  }
  return best_index;
}

double facingPath(const Trajectory &path, double x, double y) {
  const auto &nearest = path[nearestPoint(path, x, y)];
  const double angle = yaw(nearest);
  const double longitudinal =
      (x - nearest.x) * std::cos(angle) +
      (y - nearest.y) * std::sin(angle);
  const double target_x = nearest.x + longitudinal * std::cos(angle);
  const double target_y = nearest.y + longitudinal * std::sin(angle);
  return std::atan2(target_y - y, target_x - x);
}

std::size_t nearestSegment(const Trajectory &path, double x, double y) {
  std::size_t best_index = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const bg::model::segment<Point> segment(
        Point(path[i].x, path[i].y),
        Point(path[i + 1].x, path[i + 1].y));
    const double distance = bg::distance(Point(x, y), segment);
    if (distance < best) {
      best = distance;
      best_index = i;
    }
  }
  return best_index;
}

std::vector<RunOutPoint> voxelize(
    const std::vector<RunOutPoint> &points, double size_m) {
  struct Sum { double x = 0.0; double y = 0.0; std::size_t count = 0; };
  std::unordered_map<VoxelKey, Sum, VoxelHash> buckets;
  for (const auto &point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) continue;
    const VoxelKey key{
        static_cast<std::int64_t>(std::floor(point.x / size_m)),
        static_cast<std::int64_t>(std::floor(point.y / size_m))};
    auto &sum = buckets[key];
    sum.x += point.x;
    sum.y += point.y;
    ++sum.count;
  }
  std::vector<RunOutPoint> result;
  result.reserve(buckets.size());
  for (const auto &[key, sum] : buckets) {
    (void)key;
    result.push_back({sum.x / sum.count, sum.y / sum.count, 0.0});
  }
  std::sort(result.begin(), result.end(), [](const auto &a, const auto &b) {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
  });
  return result;
}
}  // 익명 네임스페이스

std::vector<RunOutCollisionObject> createRunOutObjectsWithoutPath(
    const Trajectory &path,
    const std::vector<RunOutCollisionObject> &objects,
    const RunOutCreatorParameters &parameters) {
  validate(parameters);
  if (path.empty()) return {};
  auto result = objects;
  for (auto &object : result) {
    object.pose.yaw_rad = facingPath(path, object.pose.x, object.pose.y);
    object.minimum_velocity_mps = parameters.minimum_velocity_mps;
    object.maximum_velocity_mps = parameters.maximum_velocity_mps;
    object.predicted_paths = {straightPrediction(object.pose, parameters)};
  }
  return result;
}

std::vector<RunOutCollisionObject> createRunOutObjectsFromPoints(
    const Trajectory &path, const std::vector<RunOutPoint> &points,
    const std::vector<std::vector<CruisePredictedPose>> &detection_polygons,
    const RunOutCreatorParameters &parameters, const std::string &id_prefix) {
  validate(parameters);
  if (path.size() < 2 || detection_polygons.empty()) return {};
  const auto reduced = voxelize(points, parameters.voxel_size_m);
  std::vector<Polygon> polygons;
  for (const auto &vertices : detection_polygons) {
    if (vertices.size() < 3) continue;
    Polygon polygon;
    for (const auto &vertex : vertices)
      polygon.outer().emplace_back(vertex.x, vertex.y);
    bg::correct(polygon);
    polygons.push_back(std::move(polygon));
  }
  const auto interpolated = resampleCruiseCollisionTrajectory(
      path, 0, parameters.point_path_interval_m, 0.0);
  std::vector<std::optional<RunOutPoint>> selected(interpolated.size());
  for (const auto &point : reduced) {
    bool inside = false;
    for (const auto &polygon : polygons)
      inside |= bg::covered_by(Point(point.x, point.y), polygon);
    if (!inside) continue;
    auto index = nearestSegment(interpolated, point.x, point.y);
    if (index + 2 == interpolated.size()) {
      const auto &last = interpolated.back();
      if ((point.x - last.x) * std::cos(yaw(last)) +
          (point.y - last.y) * std::sin(yaw(last)) > 0.0)
        index = interpolated.size() - 1;
    }
    const auto &base = interpolated[index];
    const auto angle = yaw(base);
    const double lateral = std::abs(
        -std::sin(angle) * (point.x - base.x) +
         std::cos(angle) * (point.y - base.y));
    if (selected[index]) {
      const auto &previous = *selected[index];
      const double previous_lateral = std::abs(
          -std::sin(angle) * (previous.x - base.x) +
           std::cos(angle) * (previous.y - base.y));
      if (lateral >= previous_lateral) continue;
    }
    selected[index] = point;
  }
  std::vector<RunOutCollisionObject> result;
  static std::atomic<std::uint64_t> next_batch{0};
  const auto batch = next_batch.fetch_add(1);
  for (std::size_t i = 0; i < selected.size(); ++i) {
    if (!selected[i]) continue;
    RunOutCollisionObject object;
    object.id = id_prefix + ":" + std::to_string(batch) + ":" +
                std::to_string(i);
    object.pose = {selected[i]->x, selected[i]->y,
                   facingPath(path, selected[i]->x, selected[i]->y)};
    object.classification = 7;
    object.shape_type = 1;
    object.length_m = parameters.point_diameter_m;
    object.width_m = parameters.point_diameter_m;
    object.minimum_velocity_mps = parameters.minimum_velocity_mps;
    object.maximum_velocity_mps = parameters.maximum_velocity_mps;
    object.predicted_paths = {straightPrediction(object.pose, parameters)};
    result.push_back(std::move(object));
  }
  return result;
}

}  // planning 네임스페이스
