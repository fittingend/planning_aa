// Copyright 2024 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 경로 속도와 교차로 기본 속도를 적용한 점유 시각을 비교합니다.

#include "planning/intersection_collision_core.hpp"
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

Polygon box(double x, double y, double yaw,
            double front, double rear, double width) {
  Polygon shape;
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  for (const auto &pair : {
           std::pair<double, double>{front, width / 2.0},
           {front, -width / 2.0},
           {-rear, -width / 2.0},
           {-rear, width / 2.0}}) {
    bg::append(shape.outer(), Point(
        x + cosine * pair.first - sine * pair.second,
        y + sine * pair.first + cosine * pair.second));
  }
  bg::correct(shape);
  return shape;
}

std::vector<double> arcs(const Trajectory &path) {
  std::vector<double> result(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double step = std::hypot(path[i].x - path[i - 1].x,
                                   path[i].y - path[i - 1].y);
    if (step <= 1.0e-9) throw std::invalid_argument("경로에 중복 좌표가 있습니다");
    result[i] = result[i - 1] + step;
  }
  return result;
}

double projectArc(const Trajectory &path, const std::vector<double> &arc,
                  double x, double y) {
  double nearest = std::numeric_limits<double>::infinity();
  double selected = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i + 1].x - path[i].x;
    const double dy = path[i + 1].y - path[i].y;
    const double length2 = dx * dx + dy * dy;
    const double ratio = std::clamp(
        ((x - path[i].x) * dx + (y - path[i].y) * dy) / length2,
        0.0, 1.0);
    const double error = std::hypot(
        x - path[i].x - ratio * dx,
        y - path[i].y - ratio * dy);
    if (error < nearest) {
      nearest = error;
      selected = arc[i] + ratio * std::sqrt(length2);
    }
  }
  return selected;
}

CrosswalkPose2d poseAtArc(
    const Trajectory &path, const std::vector<double> &arc, double query) {
  const auto upper = std::upper_bound(arc.begin(), arc.end(), query);
  const std::size_t next = std::min<std::size_t>(
      std::distance(arc.begin(), upper), path.size() - 1);
  const std::size_t prev = next - 1;
  const double ratio = (query - arc[prev]) / (arc[next] - arc[prev]);
  return {
      path[prev].x + ratio * (path[next].x - path[prev].x),
      path[prev].y + ratio * (path[next].y - path[prev].y),
      std::atan2(path[next].y - path[prev].y,
                 path[next].x - path[prev].x)};
}

CrosswalkPose2d predictedPose(
    const IntersectionPredictedVehicle &vehicle, double time) {
  const double index = time / vehicle.time_step_s;
  const std::size_t previous = std::min<std::size_t>(
      static_cast<std::size_t>(index), vehicle.predicted_poses.size() - 1);
  const std::size_t next = std::min(previous + 1, vehicle.predicted_poses.size() - 1);
  const double ratio = next == previous ? 0.0 : index - previous;
  const auto &first = vehicle.predicted_poses[previous];
  const auto &second = vehicle.predicted_poses[next];
  const double angle = std::atan2(
      std::sin(second.yaw_rad - first.yaw_rad),
      std::cos(second.yaw_rad - first.yaw_rad));
  return {first.x + ratio * (second.x - first.x),
          first.y + ratio * (second.y - first.y),
          first.yaw_rad + ratio * angle};
}

std::vector<std::pair<double, double>> smoothEgoTimeArc(
    const Trajectory &path, const std::vector<double> &arc,
    double ego_x, double ego_y, double ego_arc,
    const IntersectionCollisionParameters &settings) {
  if (!settings.ego_velocity_mps) return {};
  const double initial_speed = std::max(0.0, *settings.ego_velocity_mps);
  const auto lateral = applyJerkLateralAccelerationLimit(
      path, initial_speed, settings.ego_acceleration_mps2, {});
  const auto steering = applyJerkSteeringRateLimit(lateral, {});
  TrajectoryPoint ego_pose;
  ego_pose.x = ego_x;
  ego_pose.y = ego_y;
  const auto nearest = std::min_element(
      path.begin(), path.end(), [&](const auto &a, const auto &b) {
        return std::hypot(a.x - ego_x, a.y - ego_y) <
               std::hypot(b.x - ego_x, b.y - ego_y);
      });
  ego_pose.orientation_x = nearest->orientation_x;
  ego_pose.orientation_y = nearest->orientation_y;
  ego_pose.orientation_z = nearest->orientation_z;
  ego_pose.orientation_w = nearest->orientation_w;
  const auto resampled = resampleJerkNodeInput(
      steering, ego_pose, 3.0, 1.046, {}, 0.1);
  if (resampled.size() < 2) return {};
  const auto closest = std::min_element(
      resampled.begin(), resampled.end(), [&](const auto &a, const auto &b) {
        return std::hypot(a.x - ego_x, a.y - ego_y) <
               std::hypot(b.x - ego_x, b.y - ego_y);
      });
  const Trajectory clipped(closest, resampled.end());
  if (clipped.size() < 2) return {};
  const auto smoothed = runJerkSmoother(
      clipped, initial_speed, settings.ego_acceleration_mps2);
  if (!smoothed.output || smoothed.output->size() < 2) return {};
  std::vector<std::pair<double, double>> time_arc{{0.0, ego_arc}};
  double time = 0.0;
  const auto &points = *smoothed.output;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const double distance = std::hypot(
        points[i + 1].x - points[i].x,
        points[i + 1].y - points[i].y);
    const double speed = std::max(settings.minimum_speed_mps,
        (static_cast<double>(points[i].longitudinal_velocity_mps) +
         points[i + 1].longitudinal_velocity_mps) / 2.0);
    time += distance / speed;
    const double position_arc = projectArc(
        path, arc, points[i + 1].x, points[i + 1].y);
    if (position_arc > time_arc.back().second + 1.0e-6) {
      time_arc.emplace_back(time, position_arc);
    }
  }
  return time_arc;
}

double arcAtTime(const std::vector<std::pair<double, double>> &profile,
                 double time, double fallback_speed, double path_end) {
  if (time >= profile.back().first) {
    return std::min(path_end, profile.back().second +
        (time - profile.back().first) * fallback_speed);
  }
  const auto upper = std::upper_bound(
      profile.begin(), profile.end(), time,
      [](double query, const auto &point) { return query < point.first; });
  if (upper == profile.begin()) return profile.front().second;
  const auto &before = *(upper - 1);
  const double fraction = (time - before.first) / (upper->first - before.first);
  return before.second + fraction * (upper->second - before.second);
}

bool safeUnderTrafficControl(
    const IntersectionPredictedVehicle &vehicle,
    const IntersectionCollisionParameters &settings) {
  if ((!settings.fully_prioritized && !settings.partially_prioritized) ||
      !vehicle.distance_to_stopline_m || !vehicle.initial_speed_mps ||
      !std::isfinite(*vehicle.distance_to_stopline_m) ||
      !std::isfinite(*vehicle.initial_speed_mps)) return false;
  const double deceleration =
      vehicle.classification == 5 || vehicle.classification == 6 ? 5.0 : 2.0;
  const double braking_distance = *vehicle.initial_speed_mps *
      *vehicle.initial_speed_mps / (2.0 * deceleration);
  if (settings.partially_prioritized &&
      *vehicle.distance_to_stopline_m > braking_distance) return true;
  if (!settings.fully_prioritized || !vehicle.stopline_to_ego_path_m ||
      !std::isfinite(*vehicle.stopline_to_ego_path_m)) return false;
  if (*vehicle.distance_to_stopline_m > braking_distance) return false;
  return *vehicle.stopline_to_ego_path_m +
      *vehicle.distance_to_stopline_m - braking_distance >
      settings.traffic_control_margin_m;
}

}  // 익명 네임스페이스

IntersectionCollisionState::IntersectionCollisionState(double hold_time_s)
    : hold_time_s_(hold_time_s) {
  if (!std::isfinite(hold_time_s) || hold_time_s < 0.0) {
    throw std::invalid_argument("교차로 충돌 상태 유지 시간이 잘못되었습니다");
  }
}

bool IntersectionCollisionState::update(bool collision, double now_s) {
  if (!std::isfinite(now_s)) {
    throw std::invalid_argument("교차로 충돌 시각에는 유한한 값이 필요합니다");
  }
  if (collision) {
    stopped_ = true;
    clear_started_s_.reset();
  } else if (stopped_) {
    if (!clear_started_s_ || now_s < *clear_started_s_) {
      clear_started_s_ = now_s;
    } else if (now_s - *clear_started_s_ > hold_time_s_ + 1.0e-9) {
      stopped_ = false;
      clear_started_s_.reset();
    }
  }
  return stopped_;
}

IntersectionCollisionResult detectIntersectionCollision(
    const Trajectory &path, double ego_x, double ego_y,
    double intersection_entry_arc_m,
    const std::vector<IntersectionPredictedVehicle> &vehicles,
    IntersectionCollisionParameters settings) {
  const double values[] = {
      ego_x, ego_y, intersection_entry_arc_m,
      settings.front_m, settings.rear_m, settings.width_m,
      settings.intersection_speed_mps, settings.minimum_speed_mps,
      settings.time_sample_s, settings.collision_start_margin_s,
      settings.collision_end_margin_s, settings.ego_acceleration_mps2,
      settings.minimum_predicted_path_confidence,
      settings.traffic_control_margin_m};
  for (double value : values) {
    if (!std::isfinite(value)) throw std::invalid_argument("교차로 충돌 입력에는 유한한 값이 필요합니다");
  }
  if (settings.front_m < 0.0 || settings.rear_m < 0.0 ||
      settings.width_m <= 0.0 || settings.intersection_speed_mps <= 0.0 ||
      settings.minimum_speed_mps <= 0.0 || settings.time_sample_s <= 0.0 ||
      settings.collision_start_margin_s < 0.0 ||
      settings.collision_end_margin_s < 0.0 ||
      settings.minimum_predicted_path_confidence < 0.0 ||
      settings.minimum_predicted_path_confidence > 1.0 ||
      settings.traffic_control_margin_m < 0.0) {
    throw std::invalid_argument("교차로 충돌 설정이 잘못되었습니다");
  }
  IntersectionCollisionResult result;
  if (path.size() < 2 || vehicles.empty()) return result;
  std::vector<IntersectionPredictedVehicle> predictions;
  for (const auto &vehicle : vehicles) {
    if (safeUnderTrafficControl(vehicle, settings)) continue;
    predictions.push_back(vehicle);
    predictions.back().additional_predicted_paths.clear();
    for (const auto &alternative : vehicle.additional_predicted_paths) {
      auto candidate = vehicle;
      candidate.time_step_s = alternative.time_step_s;
      candidate.predicted_poses = alternative.poses;
      candidate.confidence = alternative.confidence;
      candidate.additional_predicted_paths.clear();
      predictions.push_back(std::move(candidate));
    }
  }
  const bool has_candidate = std::any_of(
      predictions.begin(), predictions.end(), [&](const auto &vehicle) {
        return vehicle.target_vehicle && vehicle.classification >= 1 &&
               vehicle.classification <= 6 &&
               std::isfinite(vehicle.confidence) &&
               vehicle.confidence >= settings.minimum_predicted_path_confidence &&
               vehicle.predicted_poses.size() >= 2;
      });
  if (!has_candidate) return result;
  const auto arc = arcs(path);
  const double ego_arc = projectArc(path, arc, ego_x, ego_y);
  const double entry = std::clamp(intersection_entry_arc_m, ego_arc, arc.back());
  if (settings.ego_velocity_mps && !std::isfinite(*settings.ego_velocity_mps)) {
    throw std::invalid_argument("자차 속도에는 유한한 값이 필요합니다");
  }
  const auto ego_profile = smoothEgoTimeArc(
      path, arc, ego_x, ego_y, ego_arc, settings);
  double approach_time = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double start = std::max(arc[i], ego_arc);
    const double end = std::min(arc[i + 1], entry);
    if (end > start) {
      const double speed = std::max<double>(
          path[i].longitudinal_velocity_mps, settings.minimum_speed_mps);
      approach_time += (end - start) / speed;
    }
  }
  for (const auto &vehicle : predictions) {
    if (!vehicle.target_vehicle || vehicle.classification < 1 ||
        vehicle.classification > 6 ||
        !std::isfinite(vehicle.confidence) ||
        vehicle.confidence < settings.minimum_predicted_path_confidence ||
        vehicle.predicted_poses.size() < 2 ||
        !std::isfinite(vehicle.length_m) || !std::isfinite(vehicle.width_m) ||
        !std::isfinite(vehicle.time_step_s) ||
        vehicle.length_m <= 0.0 || vehicle.width_m <= 0.0 ||
        vehicle.time_step_s <= 0.0) continue;
    const bool valid_path = std::all_of(
        vehicle.predicted_poses.begin(), vehicle.predicted_poses.end(),
        [](const CrosswalkPose2d &pose) {
          return std::isfinite(pose.x) && std::isfinite(pose.y) &&
                 std::isfinite(pose.yaw_rad);
        });
    if (!valid_path) continue;
    const double horizon = vehicle.time_step_s *
        static_cast<double>(vehicle.predicted_poses.size() - 1);
    for (double time = 0.0; time <= horizon; time += settings.time_sample_s) {
      double raw_position_arc = ego_arc;
      if (time < approach_time) {
        double remaining = time;
        for (std::size_t i = 0; i + 1 < path.size(); ++i) {
          const double start = std::max(arc[i], ego_arc);
          const double end = std::min(arc[i + 1], entry);
          if (end <= start) continue;
          const double speed = std::max<double>(
              path[i].longitudinal_velocity_mps, settings.minimum_speed_mps);
          const double duration = (end - start) / speed;
          if (remaining <= duration) {
            raw_position_arc = start + remaining * speed;
            break;
          }
          remaining -= duration;
        }
      } else {
        raw_position_arc = std::min(
            arc.back(), entry +
                (time - approach_time) * settings.intersection_speed_mps);
      }
      const double smooth_position_arc = ego_profile.empty()
          ? raw_position_arc
          : arcAtTime(ego_profile, time,
                      settings.intersection_speed_mps, arc.back());
      const double first_time = std::max(0.0,
          time - settings.collision_start_margin_s);
      const double last_time = std::min(horizon,
          time + settings.collision_end_margin_s);
      bool overlaps = false;
      double overlap_arc = 0.0;
      for (const double ego_position_arc :
           {raw_position_arc, smooth_position_arc}) {
        if (ego_position_arc < entry) continue;
        const auto ego_pose = poseAtArc(path, arc, ego_position_arc);
        const auto ego_shape = box(ego_pose.x, ego_pose.y, ego_pose.yaw_rad,
                                   settings.front_m, settings.rear_m,
                                   settings.width_m);
        for (double car_time = first_time; car_time <= last_time;
             car_time += settings.time_sample_s) {
          const auto car_pose = predictedPose(vehicle, car_time);
          const auto car_shape = box(car_pose.x, car_pose.y, car_pose.yaw_rad,
                                     vehicle.length_m / 2.0,
                                     vehicle.length_m / 2.0, vehicle.width_m);
          if (bg::intersects(ego_shape, car_shape)) {
            overlaps = true;
            overlap_arc = ego_position_arc;
            break;
          }
        }
        if (overlaps) break;
      }
      if (overlaps) {
        if (!result.collision_time_s || time < *result.collision_time_s) {
          result.collision = true;
          result.collision_time_s = time;
          result.ego_arc_m = overlap_arc;
        }
        break;
      }
    }
  }
  return result;
}

}  // planning 네임스페이스
