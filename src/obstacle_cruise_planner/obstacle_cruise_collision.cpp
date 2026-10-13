// SPDX-License-Identifier: Apache-2.0
// 원본 순항 계획의 한 단계 자차 외형과 예측 장애물 외형 교점을 구합니다.

#include "planning/obstacle_cruise_collision.hpp"
#include "planning/jerk_resample.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {
namespace {
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;

class NaturalSpline1D {
 public:
  NaturalSpline1D(std::vector<double> keys, const std::vector<double> &values)
      : keys_(std::move(keys)), values_(values), second_(values.size(), 0.0) {
    if (keys_.size() < 2 || keys_.size() != values_.size())
      throw std::invalid_argument("순항 예측 스플라인 입력이 유효하지 않습니다");
    const auto unknowns = values_.size() - 2;
    if (unknowns == 0) return;
    std::vector<double> lower(unknowns, 0.0), diagonal(unknowns),
        upper(unknowns, 0.0), rhs(unknowns);
    for (std::size_t row = 0; row < unknowns; ++row) {
      const auto i = row + 1;
      const double left = keys_[i] - keys_[i - 1];
      const double right = keys_[i + 1] - keys_[i];
      diagonal[row] = 2.0 * (left + right);
      rhs[row] = 6.0 * ((values_[i + 1] - values_[i]) / right -
                        (values_[i] - values_[i - 1]) / left);
      if (row > 0) lower[row] = left;
      if (row + 1 < unknowns) upper[row] = right;
    }
    for (std::size_t row = 1; row < unknowns; ++row) {
      const double factor = lower[row] / diagonal[row - 1];
      diagonal[row] -= factor * upper[row - 1];
      rhs[row] -= factor * rhs[row - 1];
    }
    second_[unknowns] = rhs.back() / diagonal.back();
    for (std::size_t row = unknowns - 1; row > 0; --row)
      second_[row] =
          (rhs[row - 1] - upper[row - 1] * second_[row + 1]) /
          diagonal[row - 1];
  }

  double at(const double key) const {
    const auto upper = std::upper_bound(keys_.begin(), keys_.end(), key);
    const auto i = std::clamp<std::size_t>(
        upper == keys_.begin() ? 0 :
            std::distance(keys_.begin(), upper) - 1,
        0, keys_.size() - 2);
    const double length = keys_[i + 1] - keys_[i];
    const double left = (keys_[i + 1] - key) / length;
    const double right = (key - keys_[i]) / length;
    return left * values_[i] + right * values_[i + 1] +
           ((left * left * left - left) * second_[i] +
            (right * right * right - right) * second_[i + 1]) *
               length * length / 6.0;
  }

 private:
  std::vector<double> keys_, values_, second_;
};

double yaw(const TrajectoryPoint &point) {
  return std::atan2(
      2.0 * (point.orientation_w * point.orientation_z +
             point.orientation_x * point.orientation_y),
      1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                   point.orientation_z * point.orientation_z));
}

Point offsetPoint(const double x, const double y, const double angle,
                  const double forward, const double left) {
  return {x + std::cos(angle) * forward - std::sin(angle) * left,
          y + std::sin(angle) * forward + std::cos(angle) * left};
}

Polygon vehicleFootprint(const TrajectoryPoint &pose,
                         const double front, const double rear,
                         const double width) {
  Polygon polygon;
  const double angle = yaw(pose);
  for (const auto &point : std::vector<Point>{
           offsetPoint(pose.x, pose.y, angle, front, width * 0.5),
           offsetPoint(pose.x, pose.y, angle, front, -width * 0.5),
           offsetPoint(pose.x, pose.y, angle, -rear, -width * 0.5),
           offsetPoint(pose.x, pose.y, angle, -rear, width * 0.5)})
    polygon.outer().push_back(point);
  bg::correct(polygon);
  return polygon;
}

Polygon objectFootprint(const CruisePredictedPose &pose,
                        const CruiseObjectShape &shape) {
  Polygon polygon;
  if (shape.type == 0) {
    for (const auto &point : std::vector<Point>{
             offsetPoint(pose.x, pose.y, pose.yaw_rad,
                         shape.length_m * 0.5, shape.width_m * 0.5),
             offsetPoint(pose.x, pose.y, pose.yaw_rad,
                         shape.length_m * 0.5, -shape.width_m * 0.5),
             offsetPoint(pose.x, pose.y, pose.yaw_rad,
                         -shape.length_m * 0.5, -shape.width_m * 0.5),
             offsetPoint(pose.x, pose.y, pose.yaw_rad,
                         -shape.length_m * 0.5, shape.width_m * 0.5)})
      polygon.outer().push_back(point);
  } else if (shape.type == 1) {
    constexpr int sides = 6;
    for (int i = 0; i < sides; ++i) {
      const double angle = 2.0 * std::acos(-1.0) * i / sides +
                           std::acos(-1.0) / sides;
      polygon.outer().emplace_back(
          pose.x + std::cos(angle) * shape.length_m * 0.5,
          pose.y + std::sin(angle) * shape.length_m * 0.5);
    }
  } else if (shape.type == 2) {
    for (const auto &offset : shape.footprint)
      polygon.outer().push_back(offsetPoint(
          pose.x, pose.y, pose.yaw_rad, offset.x, offset.y));
  }
  bg::correct(polygon);
  return polygon;
}

std::vector<Polygon> trajectoryPolygons(const Trajectory &path,
                                        const double front, const double rear,
                                        const double width,
                                        const std::optional<CruisePredictedPose> &ego_pose,
                                        const double interval_m,
                                        const double time_to_convergence_s,
                                        const double lateral_margin_m = 0.0) {
  std::vector<Polygon> regions;
  Polygon previous;
  double lateral_error = 0.0, yaw_error = 0.0;
  if (ego_pose) {
    std::size_t nearest = 0;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path.size(); ++i) {
      const double squared =
          std::pow(path[i].x - ego_pose->x, 2) +
          std::pow(path[i].y - ego_pose->y, 2);
      if (squared < best) {
        best = squared;
        nearest = i;
      }
    }
    std::size_t segment = nearest == path.size() - 1 ? nearest - 1 : nearest;
    if (nearest > 0 && nearest + 1 < path.size()) {
      const double projection =
          (ego_pose->x - path[nearest].x) *
              (path[nearest + 1].x - path[nearest].x) +
          (ego_pose->y - path[nearest].y) *
              (path[nearest + 1].y - path[nearest].y);
      if (projection <= 0.0) segment = nearest - 1;
    }
    const auto &reference = path[segment];
    const double angle = yaw(reference);
    lateral_error = -std::sin(angle) * (ego_pose->x - reference.x) +
                    std::cos(angle) * (ego_pose->y - reference.y);
    yaw_error = std::remainder(
        ego_pose->yaw_rad - angle, 2.0 * std::acos(-1.0));
  }
  double elapsed = 0.0;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto indexedFootprint = [&](const TrajectoryPoint &pose) {
      const bool first_moving = i == 0 &&
          pose.longitudinal_velocity_mps > 1e-3;
      Polygon polygon = vehicleFootprint(
          pose, front, rear,
          first_moving ? width : width + 2.0 * lateral_margin_m);
      if (first_moving && lateral_margin_m > 0.0) {
        const double angle = yaw(pose);
        polygon.outer().push_back(offsetPoint(
            pose.x, pose.y, angle, front,
            width * 0.5 + lateral_margin_m));
        polygon.outer().push_back(offsetPoint(
            pose.x, pose.y, angle, front,
            -width * 0.5 - lateral_margin_m));
      }
      return polygon;
    };
    const auto current = indexedFootprint(path[i]);
    Polygon points = current;
    if (ego_pose && time_to_convergence_s > 0.0 &&
        elapsed < time_to_convergence_s) {
      const double ratio =
          (time_to_convergence_s - elapsed) / time_to_convergence_s;
      const double angle = yaw(path[i]);
      TrajectoryPoint adjusted = path[i];
      adjusted.x -= std::sin(angle) * lateral_error * ratio;
      adjusted.y += std::cos(angle) * lateral_error * ratio;
      const double adjusted_angle = angle + yaw_error * ratio;
      adjusted.orientation_x = 0.0;
      adjusted.orientation_y = 0.0;
      adjusted.orientation_z = std::sin(adjusted_angle * 0.5);
      adjusted.orientation_w = std::cos(adjusted_angle * 0.5);
      const auto corrected = indexedFootprint(adjusted);
      for (const auto &point : corrected.outer())
        points.outer().push_back(point);
      if (std::abs(path[i].longitudinal_velocity_mps) > 1e-9)
        elapsed += interval_m /
                   std::abs(path[i].longitudinal_velocity_mps);
      else
        elapsed = std::numeric_limits<double>::infinity();
    }
    Polygon indexed = points;
    for (const auto &point : previous.outer()) points.outer().push_back(point);
    Polygon hull;
    bg::convex_hull(points, hull);
    bg::correct(hull);
    regions.push_back(std::move(hull));
    previous = std::move(indexed);
  }
  return regions;
}
}  // 익명 네임스페이스

std::vector<CruiseTimedPose> resampleCruisePredictedPath(
    const CruisePredictedPath &path, const double interval_s,
    const double horizon_s) {
  if (path.poses.size() < 2 || path.time_step_s <= 0.0 ||
      interval_s <= 0.0 || horizon_s <= 0.0)
    return {};
  std::vector<double> times, xs, ys;
  times.reserve(path.poses.size());
  xs.reserve(path.poses.size());
  ys.reserve(path.poses.size());
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    times.push_back(path.time_step_s * i);
    xs.push_back(path.poses[i].x);
    ys.push_back(path.poses[i].y);
  }
  const NaturalSpline1D spline_x(times, xs), spline_y(times, ys);
  const double end = std::min(horizon_s, times.back());
  std::vector<CruiseTimedPose> samples;
  for (double t = 0.0; t <= end + 1e-6; t += interval_s) {
    const auto segment = std::min<std::size_t>(
        static_cast<std::size_t>(std::floor(t / path.time_step_s)),
        path.poses.size() - 2);
    const double ratio = std::clamp(
        (t - path.time_step_s * segment) / path.time_step_s,
        0.0, 1.0);
    const double angle = path.poses[segment].yaw_rad + ratio *
        std::remainder(path.poses[segment + 1].yaw_rad -
                           path.poses[segment].yaw_rad,
                       2.0 * std::acos(-1.0));
    samples.push_back({{spline_x.at(t), spline_y.at(t), angle}, t});
  }
  return samples;
}

Trajectory resampleCruiseCollisionTrajectory(
    const Trajectory &trajectory, const std::size_t start_index,
    const double interval_m, const double extension_m) {
  if (trajectory.size() < 2 || start_index >= trajectory.size() - 1 ||
      interval_m <= 0.0 || extension_m < 0.0)
    throw std::invalid_argument("순항 충돌 경로 재표본화 입력이 유효하지 않습니다");
  const Trajectory trimmed(trajectory.begin() + start_index, trajectory.end());
  Trajectory cleaned;
  for (const auto &point : trimmed) {
    if (!cleaned.empty() &&
        std::hypot(point.x - cleaned.back().x,
                   point.y - cleaned.back().y) < 1e-6)
      continue;
    cleaned.push_back(point);
  }
  if (cleaned.size() < 2) return trimmed;
  double length = 0.0;
  for (std::size_t i = 1; i < cleaned.size(); ++i)
    length += std::hypot(cleaned[i].x - cleaned[i - 1].x,
                         cleaned[i].y - cleaned[i - 1].y);
  std::vector<double> samples;
  for (double arc = 0.0; arc < length; arc += interval_m)
    samples.push_back(arc);
  if (samples.empty()) samples.push_back(0.0);
  if (length - samples.back() < 1e-3)
    samples.back() = length;
  else
    samples.push_back(length);
  Trajectory result = resampleJerkAtArcs(cleaned, samples);
  // 원본 resamplePoseVector의 insertOrientation은 보간된 두 위치의
  // 접선 방향을 각 경로점에 넣습니다.
  for (std::size_t i = 0; i + 1 < result.size(); ++i) {
    const double angle = std::atan2(
        result[i + 1].y - result[i].y,
        result[i + 1].x - result[i].x);
    result[i].orientation_x = 0.0;
    result[i].orientation_y = 0.0;
    result[i].orientation_z = std::sin(angle * 0.5);
    result[i].orientation_w = std::cos(angle * 0.5);
  }
  result.back().orientation_x = result[result.size() - 2].orientation_x;
  result.back().orientation_y = result[result.size() - 2].orientation_y;
  result.back().orientation_z = result[result.size() - 2].orientation_z;
  result.back().orientation_w = result[result.size() - 2].orientation_w;
  if (extension_m > 0.0) {
    const auto goal = result.back();
    const double angle = yaw(goal);
    for (double extra = 0.0; extra <= extension_m - interval_m;
         extra += interval_m) {
      auto point = goal;
      point.x += extra * std::cos(angle);
      point.y += extra * std::sin(angle);
      result.push_back(point);
    }
    auto point = goal;
    point.x += extension_m * std::cos(angle);
    point.y += extension_m * std::sin(angle);
    result.push_back(point);
  }
  return result;
}

double cruiseCurveStopMargin(
    const Trajectory &trajectory,
    const CruisePredictedPose &collision_point,
    const CruisePredictedPose &object_pose,
    const CruiseObjectShape &shape,
    const double vehicle_width_m, const double vehicle_front_m,
    const double safe_margin_m, const double minimum_margin_m,
    const double additional_margin_m) {
  if (trajectory.size() < 2 || vehicle_width_m <= 0.0 ||
      vehicle_front_m < 0.0 || safe_margin_m < 0.0 ||
      minimum_margin_m < 0.0 || additional_margin_m < 0.0 ||
      minimum_margin_m > safe_margin_m || shape.length_m < 0.0 ||
      shape.width_m < 0.0)
    throw std::invalid_argument("곡선 정지 여유 입력이 유효하지 않습니다");
  std::size_t object_segment = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < trajectory.size(); ++i) {
    const bg::model::segment<Point> segment(
        Point(trajectory[i].x, trajectory[i].y),
        Point(trajectory[i + 1].x, trajectory[i + 1].y));
    const double distance = bg::distance(
        Point(collision_point.x, collision_point.y), segment);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      object_segment = i;
    }
  }
  std::size_t first = object_segment;
  double length = 0.0;
  for (int i = static_cast<int>(object_segment); i >= 0; --i) {
    first = static_cast<std::size_t>(i);
    if (length > safe_margin_m + vehicle_front_m) break;
    length += std::hypot(
        trajectory[i + 1].x - trajectory[i].x,
        trajectory[i + 1].y - trajectory[i].y);
  }
  const Trajectory short_path(trajectory.begin() + first,
                              trajectory.begin() + object_segment + 2);
  if (short_path.size() < 2) return safe_margin_m;
  const auto resampled = resampleCruiseCollisionTrajectory(
      short_path, 0, 0.5, 0.0);
  const auto polygon = objectFootprint(object_pose, shape);
  if (polygon.outer().empty()) return minimum_margin_m;
  const auto distance_from_straight = [&](const TrajectoryPoint &point) {
    const auto angle = yaw(point);
    const bg::model::segment<Point> straight(
        Point(point.x, point.y),
        offsetPoint(point.x, point.y, angle, safe_margin_m + 3.0, 0.0));
    return bg::distance(straight, polygon);
  };
  std::size_t collision_index = resampled.size();
  for (std::size_t i = 0; i < resampled.size(); ++i) {
    if (distance_from_straight(resampled[i]) < vehicle_width_m * 0.5) {
      collision_index = i;
      break;
    }
  }
  if (collision_index == resampled.size()) return minimum_margin_m;
  if (collision_index == 0) return safe_margin_m;
  const double previous_distance =
      distance_from_straight(resampled[collision_index - 1]);
  const double next_distance =
      distance_from_straight(resampled[collision_index]);
  const double segment_length = std::hypot(
      resampled[collision_index].x - resampled[collision_index - 1].x,
      resampled[collision_index].y - resampled[collision_index - 1].y);
  const double denominator = next_distance - previous_distance;
  const double partial = std::abs(denominator) < 1e-9 ? 0.0 :
      (next_distance - vehicle_width_m * 0.5) / denominator *
          segment_length;
  double accumulated = 0.0;
  double projected_arc = 0.0;
  double projection_error = std::numeric_limits<double>::infinity();
  double collision_arc = 0.0;
  for (std::size_t i = 0; i + 1 < resampled.size(); ++i) {
    if (i == collision_index) collision_arc = accumulated;
    const double dx = resampled[i + 1].x - resampled[i].x;
    const double dy = resampled[i + 1].y - resampled[i].y;
    const double segment = std::hypot(dx, dy);
    if (segment < 1e-9) continue;
    const double ratio = std::clamp(
        ((collision_point.x - resampled[i].x) * dx +
         (collision_point.y - resampled[i].y) * dy) /
            (segment * segment), 0.0, 1.0);
    const double error = std::hypot(
        collision_point.x - resampled[i].x - ratio * dx,
        collision_point.y - resampled[i].y - ratio * dy);
    if (error < projection_error) {
      projection_error = error;
      projected_arc = accumulated + ratio * segment;
    }
    accumulated += segment;
  }
  if (collision_index + 1 == resampled.size()) collision_arc = accumulated;
  return std::clamp(partial + projected_arc - collision_arc -
                        vehicle_front_m + additional_margin_m,
                    minimum_margin_m, safe_margin_m);
}

double cruiseObjectLateralDistance(
    const Trajectory &path, const CruisePredictedPose &object_pose,
    const CruiseObjectShape &shape, const double vehicle_front_m,
    const double vehicle_rear_m, const double vehicle_width_m,
    const std::optional<CruisePredictedPose> current_ego_pose,
    const double interval_m, const double time_to_convergence_s) {
  if (path.size() < 2 || vehicle_front_m < 0.0 || vehicle_rear_m < 0.0 ||
      vehicle_width_m <= 0.0)
    throw std::invalid_argument("순항 객체 정밀 횡거리 입력이 유효하지 않습니다");
  const auto regions = trajectoryPolygons(
      path, vehicle_front_m, vehicle_rear_m, vehicle_width_m,
      current_ego_pose, interval_m, time_to_convergence_s);
  const auto object = objectFootprint(object_pose, shape);
  double distance = std::numeric_limits<double>::infinity();
  for (const auto &region : regions)
    distance = std::min(distance, bg::distance(region, object));
  return distance;
}

std::optional<CruiseStopCollisionPoint> findCruiseCurrentStopCollision(
    const Trajectory &path, const CruisePredictedPose &object_pose,
    const CruiseObjectShape &shape, const double vehicle_front_m,
    const double vehicle_rear_m, const double vehicle_width_m,
    const double lateral_margin_m, const bool driving_forward,
    const std::optional<CruisePredictedPose> current_ego_pose,
    const double interval_m, const double time_to_convergence_s) {
  if (path.size() < 2 || vehicle_front_m < 0.0 || vehicle_rear_m < 0.0 ||
      vehicle_width_m <= 0.0 || lateral_margin_m < 0.0 ||
      !std::isfinite(lateral_margin_m) || shape.length_m < 0.0 ||
      shape.width_m < 0.0)
    throw std::invalid_argument("순항 정지 교점 입력이 유효하지 않습니다");
  const auto regions = trajectoryPolygons(
      path, vehicle_front_m, vehicle_rear_m, vehicle_width_m,
      current_ego_pose, interval_m, time_to_convergence_s,
      lateral_margin_m);
  const auto object = objectFootprint(object_pose, shape);
  double arc = 0.0;
  for (std::size_t index = 0; index < regions.size(); ++index) {
    if (index > 0)
      arc += std::hypot(path[index].x - path[index - 1].x,
                        path[index].y - path[index - 1].y);
    std::vector<Polygon> intersections;
    bg::intersection(regions[index], object, intersections);
    const double angle = yaw(path[index]);
    const Point bumper = offsetPoint(
        path[index].x, path[index].y, angle,
        driving_forward ? vehicle_front_m : -vehicle_rear_m, 0.0);
    double maximum_length = -1.0;
    CruiseStopCollisionPoint selected;
    for (const auto &intersection : intersections) {
      if (bg::area(intersection) <= 0.0) continue;
      for (const auto &point : intersection.outer()) {
        const double length = std::abs(
            std::cos(angle) * (point.x() - bumper.x()) +
            std::sin(angle) * (point.y() - bumper.y()));
        if (length > maximum_length) {
          maximum_length = length;
          selected = {point.x(), point.y(), arc - length, index};
        }
      }
    }
    if (maximum_length >= 0.0) return selected;
  }
  return std::nullopt;
}

std::optional<CruiseSlowdownCollisionInterval>
findCruiseSlowdownCollisionInterval(
    const Trajectory &path, const CruisePredictedPose &object_pose,
    const CruiseObjectShape &shape, const double vehicle_front_m,
    const double vehicle_rear_m, const double vehicle_width_m,
    const double lateral_margin_m,
    const std::optional<CruisePredictedPose> current_ego_pose,
    const double interval_m, const double time_to_convergence_s) {
  if (path.size() < 2 || vehicle_front_m < 0.0 || vehicle_rear_m < 0.0 ||
      vehicle_width_m <= 0.0 || lateral_margin_m < 0.0 ||
      !std::isfinite(lateral_margin_m) || shape.length_m < 0.0 ||
      shape.width_m < 0.0)
    throw std::invalid_argument("순항 감속 교점 입력이 유효하지 않습니다");
  const auto regions = trajectoryPolygons(
      path, vehicle_front_m, vehicle_rear_m, vehicle_width_m,
      current_ego_pose, interval_m, time_to_convergence_s,
      lateral_margin_m);
  const auto object = objectFootprint(object_pose, shape);
  std::optional<CruiseSlowdownCollisionInterval> found;
  double accumulated = 0.0;
  for (std::size_t index = 0; index < regions.size(); ++index) {
    if (index > 0)
      accumulated += std::hypot(path[index].x - path[index - 1].x,
                                path[index].y - path[index - 1].y);
    std::vector<Polygon> intersections;
    bg::intersection(regions[index], object, intersections);
    bool has_area = false;
    double smallest = std::numeric_limits<double>::infinity();
    double largest = -std::numeric_limits<double>::infinity();
    CruisePredictedPose front_point;
    CruisePredictedPose back_point;
    const auto segment = index == 0 ? 0 : index - 1;
    const double dx = path[segment + 1].x - path[segment].x;
    const double dy = path[segment + 1].y - path[segment].y;
    const double length = std::hypot(dx, dy);
    if (length < 1e-9) continue;
    const double segment_arc = index == 0 ? 0.0 : accumulated - length;
    for (const auto &polygon : intersections) {
      if (bg::area(polygon) <= 0.0) continue;
      has_area = true;
      for (const auto &point : polygon.outer()) {
        const double arc = segment_arc +
            ((point.x() - path[segment].x) * dx +
             (point.y() - path[segment].y) * dy) / length;
        if (arc < smallest) {
          smallest = arc;
          front_point = {point.x(), point.y(), 0.0};
        }
        if (arc > largest) {
          largest = arc;
          back_point = {point.x(), point.y(), 0.0};
        }
      }
    }
    if (has_area) {
      if (!found)
        found = CruiseSlowdownCollisionInterval{
            smallest, largest, front_point, back_point};
      else {
        found->back_arc_m = largest;
        found->back_point = back_point;
      }
    } else if (found) {
      break;
    }
  }
  return found;
}

std::vector<CruiseCollisionPoint> findCruiseCollisionPoints(
    const Trajectory &path, const std::vector<CruiseTimedPose> &prediction,
    const CruiseObjectShape &shape, const double vehicle_front_m,
    const double vehicle_rear_m, const double vehicle_width_m,
    const double current_elapsed_s, const double maximum_prediction_s,
    const double maximum_distance_m, const bool driving_forward,
    const std::optional<CruisePredictedPose> current_ego_pose,
    const double interval_m, const double time_to_convergence_s) {
  if (path.size() < 2 || vehicle_front_m < 0.0 || vehicle_rear_m < 0.0 ||
      vehicle_width_m <= 0.0 || maximum_prediction_s < 0.0 ||
      maximum_distance_m < 0.0 || shape.length_m < 0.0 ||
      shape.width_m < 0.0 || !std::isfinite(current_elapsed_s))
    throw std::invalid_argument("순항 충돌점 입력이 유효하지 않습니다");
  const auto regions = trajectoryPolygons(
      path, vehicle_front_m, vehicle_rear_m, vehicle_width_m,
      current_ego_pose, interval_m, time_to_convergence_s);
  std::vector<CruiseCollisionPoint> result;
  for (const auto &sample : prediction) {
    if (sample.time_from_object_stamp_s > maximum_prediction_s) break;
    if (sample.time_from_object_stamp_s < current_elapsed_s) continue;
    const auto object = objectFootprint(sample.pose, shape);
    if (object.outer().empty()) continue;
    for (std::size_t index = 0; index < regions.size(); ++index) {
      if (std::hypot(path[index].x - sample.pose.x,
                     path[index].y - sample.pose.y) > maximum_distance_m)
        continue;
      std::vector<Polygon> intersections;
      bg::intersection(regions[index], object, intersections);
      double best = std::numeric_limits<double>::infinity();
      CruiseCollisionPoint selected;
      const auto first = index == 0 ? 0 : index - 1;
      const auto last = index == 0 ? 1 : index;
      double dx = path[last].x - path[first].x;
      double dy = path[last].y - path[first].y;
      if (!driving_forward) {
        dx = -dx;
        dy = -dy;
      }
      const double length = std::hypot(dx, dy);
      if (length < 1e-9) continue;
      const auto &origin = driving_forward ? path[first] : path[last];
      for (const auto &intersection : intersections) {
        if (bg::area(intersection) <= 0.0) continue;
        for (const auto &point : intersection.outer()) {
          const double distance =
              ((point.x() - origin.x) * dx +
               (point.y() - origin.y) * dy) / length;
          if (distance < best) {
            best = distance;
            selected = {point.x(), point.y(),
                        sample.time_from_object_stamp_s, index};
          }
        }
      }
      if (std::isfinite(best)) {
        result.push_back(selected);
        break;
      }
    }
  }
  return result;
}

}  // planning 네임스페이스
