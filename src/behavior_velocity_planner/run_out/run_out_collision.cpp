// SPDX-License-Identifier: Apache-2.0
// 원본 돌발 진입 객체 예측과 주행 외형 교차의 독립 구현입니다.

#include "planning/run_out_collision.hpp"
#include "planning/analytical_velocity_smoother.hpp"
#include "planning/jerk_path_limits.hpp"
#include "planning/jerk_resample.hpp"
#include "planning/jerk_smoother.hpp"

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
using Segment = bg::model::segment<Point>;

Point offset(double x, double y, double yaw, double forward, double left) {
  return {x + std::cos(yaw) * forward - std::sin(yaw) * left,
          y + std::sin(yaw) * forward + std::cos(yaw) * left};
}

double yaw(const TrajectoryPoint &point) {
  return std::atan2(2.0 * (point.orientation_w * point.orientation_z +
                           point.orientation_x * point.orientation_y),
                    1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                                 point.orientation_z * point.orientation_z));
}

Polygon rectangle(const CruisePredictedPose &pose, double front, double rear,
                  double left, double right) {
  Polygon poly;
  for (const auto &point : {
           offset(pose.x, pose.y, pose.yaw_rad, front, left),
           offset(pose.x, pose.y, pose.yaw_rad, front, -right),
           offset(pose.x, pose.y, pose.yaw_rad, -rear, -right),
           offset(pose.x, pose.y, pose.yaw_rad, -rear, left)})
    poly.outer().push_back(point);
  bg::correct(poly);
  return poly;
}

double normalize(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

Polygon rangedObject(const CruisePredictedPose &minimum,
                     const CruisePredictedPose &maximum,
                     double half_length, double half_width) {
  const double distance = std::hypot(maximum.x - minimum.x,
                                     maximum.y - minimum.y);
  const double angle = distance < std::numeric_limits<double>::epsilon() ?
      minimum.yaw_rad : std::atan2(maximum.y - minimum.y,
                                   maximum.x - minimum.x);
  return rectangle({minimum.x, minimum.y, angle},
                   distance + half_length, half_length,
                   half_width, half_width);
}

double projectArc(const Trajectory &path, const Point &point) {
  double arc = 0.0, selected = 0.0;
  double nearest = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double dx = path[i].x - path[i - 1].x;
    const double dy = path[i].y - path[i - 1].y;
    const double squared = dx * dx + dy * dy;
    if (squared < 1e-12) continue;
    const double fraction = std::clamp(
        ((point.x() - path[i - 1].x) * dx +
         (point.y() - path[i - 1].y) * dy) / squared, 0.0, 1.0);
    const double distance = std::hypot(
        point.x() - path[i - 1].x - fraction * dx,
        point.y() - path[i - 1].y - fraction * dy);
    if (distance < nearest) {
      nearest = distance;
      selected = arc + fraction * std::sqrt(squared);
    }
    arc += std::sqrt(squared);
  }
  return selected;
}
}  // 익명 네임스페이스

RunOutVelocityRange runOutVelocityRange(
    double velocity_x_mps, double velocity_y_mps,
    double variance_x, double variance_y,
    double standard_deviation_multiplier) {
  if (variance_x < 0.0 || variance_y < 0.0 ||
      standard_deviation_multiplier < 0.0)
    throw std::invalid_argument("돌발 진입 객체 속도 공분산이 유효하지 않습니다");
  const double x = std::abs(velocity_x_mps);
  const double y = std::abs(velocity_y_mps);
  const double dx = standard_deviation_multiplier * std::sqrt(variance_x);
  const double dy = standard_deviation_multiplier * std::sqrt(variance_y);
  return {std::hypot(std::max(0.0, x - dx), std::max(0.0, y - dy)),
          std::hypot(x + dx, y + dy)};
}

Trajectory prepareRunOutCollisionPath(
    const Trajectory &path, double ego_x, double ego_y,
    double extension_m, double detection_distance_m) {
  if (path.empty()) return {};
  if (!std::isfinite(ego_x) || !std::isfinite(ego_y) ||
      extension_m < 0.0 || detection_distance_m <= 0.0)
    throw std::invalid_argument("돌발 진입 경로 준비 입력이 유효하지 않습니다");
  Trajectory extended = path;
  const auto &goal = path.back();
  const double angle = yaw(goal);
  for (double distance = 0.1; distance < extension_m; distance += 0.1) {
    auto point = goal;
    point.x += distance * std::cos(angle);
    point.y += distance * std::sin(angle);
    extended.push_back(point);
  }
  const auto nearest = std::min_element(extended.begin(), extended.end(),
      [&](const auto &a, const auto &b) {
        return std::hypot(a.x - ego_x, a.y - ego_y) <
               std::hypot(b.x - ego_x, b.y - ego_y);
      });
  Trajectory result;
  double distance = 0.0;
  for (auto it = nearest; it != extended.end(); ++it) {
    if (it != nearest)
      distance += std::hypot(it->x - (it - 1)->x,
                             it->y - (it - 1)->y);
    result.push_back(*it);
    if (distance > detection_distance_m) break;
  }
  return result;
}

std::optional<Trajectory> smoothRunOutCollisionPath(
    const Trajectory &path, const CruisePredictedPose &ego_pose,
    double ego_velocity_mps, double ego_acceleration_mps2,
    std::optional<double> external_maximum_velocity_mps) {
  if (path.size() < 2 || !std::isfinite(ego_pose.x) ||
      !std::isfinite(ego_pose.y) || !std::isfinite(ego_pose.yaw_rad) ||
      !std::isfinite(ego_velocity_mps) ||
      !std::isfinite(ego_acceleration_mps2) ||
      (external_maximum_velocity_mps &&
       (!std::isfinite(*external_maximum_velocity_mps) ||
        *external_maximum_velocity_mps < 0.0)))
    throw std::invalid_argument("돌발 진입 경로 평활화 입력이 유효하지 않습니다");
  const auto lateral = applyJerkLateralAccelerationLimit(
      path, ego_velocity_mps, ego_acceleration_mps2, {});
  const auto steering = applyJerkSteeringRateLimit(lateral, {});
  TrajectoryPoint pose;
  pose.x = ego_pose.x;
  pose.y = ego_pose.y;
  pose.orientation_z = std::sin(ego_pose.yaw_rad * 0.5);
  pose.orientation_w = std::cos(ego_pose.yaw_rad * 0.5);
  const auto resampled = resampleJerkNodeInput(
      steering, pose, 3.0, 1.046, {}, 0.1);
  if (resampled.size() < 2) return std::nullopt;
  std::size_t closest = 0;
  double best = std::numeric_limits<double>::infinity();
  bool constrained = false;
  for (std::size_t i = 0; i < resampled.size(); ++i) {
    const double distance = std::hypot(
        resampled[i].x - ego_pose.x, resampled[i].y - ego_pose.y);
    const bool inside = distance <= 3.0 &&
        std::fabs(std::remainder(yaw(resampled[i]) - ego_pose.yaw_rad,
                                 2.0 * std::acos(-1.0))) <= 1.046;
    if (inside && (!constrained || distance < best)) {
      constrained = true;
      closest = i;
      best = distance;
    } else if (!constrained && distance < best) {
      closest = i;
      best = distance;
    }
  }
  const Trajectory clipped(resampled.begin() + closest, resampled.end());
  const auto smoothed = runJerkSmoother(
      clipped, ego_velocity_mps, ego_acceleration_mps2);
  if (!smoothed.output) return std::nullopt;
  Trajectory output(resampled.begin(), resampled.begin() + closest);
  output.insert(output.end(), smoothed.output->begin(), smoothed.output->end());
  if (external_maximum_velocity_mps) {
    for (std::size_t i = closest; i < output.size(); ++i)
      output[i].longitudinal_velocity_mps = std::min(
          static_cast<double>(output[i].longitudinal_velocity_mps),
          *external_maximum_velocity_mps);
  }
  return output;
}

Trajectory smoothRunOutCollisionPathAnalytical(
    const Trajectory &path, const CruisePredictedPose &ego_pose,
    double ego_velocity_mps, double ego_acceleration_mps2,
    std::optional<double> external_maximum_velocity_mps,
    bool include_deceleration) {
  if (path.size() < 2 || !std::isfinite(ego_pose.x) ||
      !std::isfinite(ego_pose.y) || !std::isfinite(ego_pose.yaw_rad) ||
      !std::isfinite(ego_velocity_mps) ||
      !std::isfinite(ego_acceleration_mps2) ||
      (external_maximum_velocity_mps &&
       (!std::isfinite(*external_maximum_velocity_mps) ||
        *external_maximum_velocity_mps < 0.0))) {
    throw std::invalid_argument("Analytical 돌발 진입 경로 입력이 유효하지 않습니다");
  }
  const auto lateral = applyAnalyticalLateralAccelerationLimit(path);
  const auto steering = applyJerkSteeringRateLimit(lateral, {});
  std::size_t closest = 0;
  double best = std::numeric_limits<double>::infinity();
  bool constrained = false;
  for (std::size_t index = 0; index < steering.size(); ++index) {
    const double distance = std::hypot(
        steering[index].x - ego_pose.x,
        steering[index].y - ego_pose.y);
    const bool inside = distance <= 3.0 &&
        std::fabs(std::remainder(yaw(steering[index]) - ego_pose.yaw_rad,
                                 2.0 * std::acos(-1.0))) <= 1.046;
    if (inside && (!constrained || distance < best)) {
      constrained = true;
      closest = index;
      best = distance;
    } else if (!constrained && distance < best) {
      closest = index;
      best = distance;
    }
  }
  const Trajectory clipped(steering.begin() + closest, steering.end());
  const auto smoothed = include_deceleration ?
      smoothAnalyticalVelocity(
          clipped, ego_velocity_mps, ego_acceleration_mps2) :
      applyAnalyticalForwardJerkFilter(
          clipped, 0, ego_velocity_mps, ego_acceleration_mps2);
  Trajectory output(steering.begin(), steering.begin() + closest);
  output.insert(output.end(), smoothed.begin(), smoothed.end());
  if (external_maximum_velocity_mps) {
    for (std::size_t index = closest; index < output.size(); ++index) {
      output[index].longitudinal_velocity_mps = static_cast<float>(
          std::min(static_cast<double>(
                       output[index].longitudinal_velocity_mps),
                   *external_maximum_velocity_mps));
    }
  }
  return output;
}

std::vector<RunOutCollisionObject> RunOutObjectFilterCore::filter(
    const Trajectory &path, const CruisePredictedPose &ego_pose,
    double now_s, const std::vector<RunOutCollisionObject> &objects,
    const RunOutObjectFilterParameters &parameters) {
  if (!std::isfinite(now_s) || parameters.ego_cut_line_length_m < 0.0 ||
      parameters.partition_search_distance_m < 0.0 ||
      parameters.vehicle_width_m <= 0.0 ||
      parameters.path_footprint_extra_margin_m < 0.0 ||
      parameters.keep_on_path_s < 0.0)
    throw std::invalid_argument("돌발 진입 객체 제외 매개변수가 유효하지 않습니다");
  std::vector<RunOutCollisionObject> remaining;
  const auto left = offset(ego_pose.x, ego_pose.y, ego_pose.yaw_rad,
                           0.0, parameters.ego_cut_line_length_m * 0.5);
  const auto right = offset(ego_pose.x, ego_pose.y, ego_pose.yaw_rad,
                            0.0, -parameters.ego_cut_line_length_m * 0.5);
  const Segment cut_line(left, right);
  Trajectory partition_path;
  if (!path.empty()) {
    partition_path.push_back(path.front());
    double accumulated = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
      accumulated += std::hypot(path[i].x - path[i - 1].x,
                                path[i].y - path[i - 1].y);
      if (accumulated > 1.0) {
        partition_path.push_back(path[i]);
        accumulated = 0.0;
      }
    }
  }
  for (const auto &object : objects) {
    if (!parameters.partition_segments.empty() && !partition_path.empty()) {
      const auto nearest = std::min_element(
          partition_path.begin(), partition_path.end(),
          [&](const auto &a, const auto &b) {
            return std::hypot(a.x - object.pose.x, a.y - object.pose.y) <
                   std::hypot(b.x - object.pose.x, b.y - object.pose.y);
          });
      const Segment object_link(
          Point(nearest->x, nearest->y),
          Point(object.pose.x, object.pose.y));
      bool separated = false;
      for (const auto &edge : parameters.partition_segments) {
        const Segment partition(Point(edge.x1, edge.y1),
                                Point(edge.x2, edge.y2));
        if (bg::distance(Point(ego_pose.x, ego_pose.y), partition) >
            parameters.partition_search_distance_m) continue;
        if (bg::intersects(object_link, partition)) {
          separated = true;
          break;
        }
      }
      if (separated) continue;
    }
    if (parameters.exclude_crossing_ego_cut_line &&
        !object.predicted_paths.empty()) {
      const auto best = std::max_element(object.predicted_paths.begin(),
          object.predicted_paths.end(), [](const auto &a, const auto &b) {
            return a.confidence < b.confidence;
          });
      bool crosses = false;
      for (std::size_t i = 1; i < best->poses.size(); ++i) {
        if (bg::intersects(cut_line,
                Segment(Point(best->poses[i - 1].x, best->poses[i - 1].y),
                        Point(best->poses[i].x, best->poses[i].y)))) {
          crosses = true;
          break;
        }
      }
      if (crosses) continue;
    }
    if (!parameters.exclude_objects_already_in_path || path.empty()) {
      remaining.push_back(object);
      continue;
    }
    const auto nearest = std::min_element(path.begin(), path.end(),
        [&](const auto &a, const auto &b) {
          return std::hypot(a.x - object.pose.x, a.y - object.pose.y) <
                 std::hypot(b.x - object.pose.x, b.y - object.pose.y);
        });
    const double angle = yaw(*nearest);
    const double lateral = -std::sin(angle) *
                               (object.pose.x - nearest->x) +
                           std::cos(angle) *
                               (object.pose.y - nearest->y);
    const double half_width = 0.5 *
        (parameters.vehicle_width_m + parameters.path_footprint_extra_margin_m);
    if (std::abs(lateral) > half_width) {
      first_inside_path_s_.erase(object.id);
      remaining.push_back(object);
      continue;
    }
    auto [found, inserted] = first_inside_path_s_.try_emplace(object.id, now_s);
    if (inserted || now_s - found->second < parameters.keep_on_path_s)
      remaining.push_back(object);
  }
  return remaining;
}

void RunOutObjectFilterCore::reset() { first_inside_path_s_.clear(); }

std::optional<CruisePredictedPose> predictRunOutObjectPose(
    const std::vector<CruisePredictedPath> &paths, double travel_time_s,
    double velocity_mps) {
  if (!std::isfinite(travel_time_s) || !std::isfinite(velocity_mps))
    throw std::invalid_argument("돌발 진입 객체 예측 시간이 유효하지 않습니다");
  if (paths.empty()) return std::nullopt;
  const auto best = std::max_element(paths.begin(), paths.end(),
      [](const auto &left, const auto &right) {
        return left.confidence < right.confidence;
      });
  const auto &poses = best->poses;
  if (poses.size() < 2) return std::nullopt;
  if (travel_time_s < std::numeric_limits<float>::epsilon() ||
      velocity_mps < std::numeric_limits<float>::epsilon()) return poses.front();
  double elapsed = 0.0;
  for (std::size_t i = 1; i < poses.size(); ++i) {
    const double length = std::hypot(poses[i].x - poses[i - 1].x,
                                     poses[i].y - poses[i - 1].y);
    const double duration = length / velocity_mps;
    if (elapsed + duration > travel_time_s) {
      const double ratio = (travel_time_s - elapsed) / duration;
      return CruisePredictedPose{
          poses[i - 1].x + ratio * (poses[i].x - poses[i - 1].x),
          poses[i - 1].y + ratio * (poses[i].y - poses[i - 1].y),
          normalize(poses[i - 1].yaw_rad + ratio *
                    normalize(poses[i].yaw_rad - poses[i - 1].yaw_rad))};
    }
    elapsed += duration;
  }
  return poses.back();
}

std::optional<RunOutCollision> findRunOutCollision(
    const Trajectory &path, const std::vector<RunOutCollisionObject> &objects,
    const RunOutCollisionParameters &parameters) {
  if (path.size() < 2) return std::nullopt;
  if (parameters.minimum_ego_velocity_mps <= 0.0 ||
      parameters.detection_span_m < 0.0 ||
      parameters.detection_distance_m <= 0.0 ||
      parameters.maximum_prediction_time_s <= 0.0 ||
      parameters.vehicle_front_m <= 0.0 ||
      parameters.vehicle_rear_m < 0.0 ||
      parameters.vehicle_left_m <= 0.0 ||
      parameters.vehicle_right_m <= 0.0 ||
      parameters.passing_margin_m < 0.0)
    throw std::invalid_argument("돌발 진입 충돌 매개변수가 유효하지 않습니다");
  double travel_time = 0.0;
  double since_check = 0.0;
  double path_arc = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double length = std::hypot(path[i].x - path[i - 1].x,
                                     path[i].y - path[i - 1].y);
    travel_time += length / std::max(static_cast<double>(path[i - 1].longitudinal_velocity_mps),
                                     parameters.minimum_ego_velocity_mps);
    since_check += length;
    path_arc += length;
    if (path_arc > parameters.detection_distance_m) break;
    if (i != 1 && since_check < parameters.detection_span_m) continue;
    since_check = 0.0;
    const CruisePredictedPose ego{path[i].x, path[i].y, yaw(path[i])};
    const auto vehicle = rectangle(ego, parameters.vehicle_front_m,
        parameters.vehicle_rear_m, parameters.vehicle_left_m,
        parameters.vehicle_right_m);
    std::optional<RunOutCollision> closest;
    double closest_object_arc = std::numeric_limits<double>::infinity();
    for (const auto &object : objects) {
      if (std::find(parameters.target_classifications.begin(),
                    parameters.target_classifications.end(),
                    object.classification) ==
          parameters.target_classifications.end()) continue;
      if (object.shape_type != 0 && object.shape_type != 1) continue;
      auto predictions = object.predicted_paths;
      for (auto &prediction : predictions) {
        if (prediction.time_step_s > std::numeric_limits<double>::epsilon()) {
          const auto count = static_cast<std::size_t>(
              parameters.maximum_prediction_time_s / prediction.time_step_s);
          prediction.poses.resize(std::min(count, prediction.poses.size()));
        }
      }
      const auto minimum = predictRunOutObjectPose(
          predictions, travel_time, object.minimum_velocity_mps);
      const auto maximum = predictRunOutObjectPose(
          predictions, travel_time, object.maximum_velocity_mps);
      if (!minimum || !maximum) continue;
      const auto shape = rangedObject(*minimum, *maximum,
          object.length_m * 0.5,
          (object.shape_type == 1 ? object.length_m : object.width_m) * 0.5);
      std::vector<Point> intersections;
      bg::intersection(vehicle, shape, intersections);
      if (intersections.empty()) continue;
      const double side = -std::sin(ego.yaw_rad) * (object.pose.x - ego.x) +
                          std::cos(ego.yaw_rad) * (object.pose.y - ego.y);
      const Point front = offset(ego.x, ego.y, ego.yaw_rad,
                                 parameters.vehicle_front_m, 0.0);
      std::vector<Point> unsuppressed;
      for (const auto &point : intersections) {
        bool suppressed = false;
        for (const auto &vertices : parameters.suppressed_polygons) {
          if (vertices.size() < 3) continue;
          Polygon region;
          for (const auto &vertex : vertices)
            region.outer().emplace_back(vertex.x, vertex.y);
          bg::correct(region);
          if (bg::within(point, region)) {
            suppressed = true;
            break;
          }
        }
        if (!suppressed) unsuppressed.push_back(point);
      }
      if (unsuppressed.empty()) continue;
      std::vector<Point> same_side;
      for (const auto &point : unsuppressed) {
        const double lateral = -std::sin(ego.yaw_rad) *
                                   (point.x() - ego.x) +
                               std::cos(ego.yaw_rad) *
                                   (point.y() - ego.y);
        if (side * lateral > 0.0) same_side.push_back(point);
      }
      const auto &candidate_points = same_side.empty() ?
          unsuppressed : same_side;
      std::optional<RunOutCollision> nearest_point;
      for (const auto &point : candidate_points) {
        const double collision_arc = projectArc(path, point);
        if (!nearest_point || collision_arc < nearest_point->collision_arc_m) {
          const double longitudinal = std::cos(ego.yaw_rad) *
                                          (point.x() - front.x()) +
                                      std::sin(ego.yaw_rad) *
                                          (point.y() - front.y());
          nearest_point = RunOutCollision{
              object.id, collision_arc, travel_time,
              point.x(), point.y(), std::abs(longitudinal)};
        }
      }
      const double object_arc = projectArc(path,
          Point(object.pose.x, object.pose.y));
      if (nearest_point &&
          nearest_point->distance_from_vehicle_front_m <
              parameters.passing_margin_m &&
          object_arc < closest_object_arc) {
        closest_object_arc = object_arc;
        closest = *nearest_point;
      }
    }
    if (closest) return closest;
  }
  return std::nullopt;
}

}  // planning 네임스페이스
