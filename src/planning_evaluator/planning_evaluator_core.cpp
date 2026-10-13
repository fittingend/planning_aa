// SPDX-License-Identifier: Apache-2.0

#include "planning/planning_evaluator_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace planning {
namespace {
double distance(const TrajectoryPoint &a, const TrajectoryPoint &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double yaw(const TrajectoryPoint &point) {
  return std::atan2(
      2.0 * (point.orientation_w * point.orientation_z +
             point.orientation_x * point.orientation_y),
      1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                   point.orientation_z * point.orientation_z));
}

double angle(double value) {
  return std::remainder(value, 2.0 * std::acos(-1.0));
}

std::size_t nearestPoint(const Trajectory &trajectory,
                         const TrajectoryPoint &point) {
  std::size_t selected = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    const double candidate = distance(trajectory[i], point);
    if (candidate < best) {
      best = candidate;
      selected = i;
    }
  }
  return selected;
}

struct Projection {
  std::size_t segment = 0;
  double ratio = 0.0;
  double distance = 0.0;
  double signed_distance = 0.0;
};

Projection project(const Trajectory &path, const TrajectoryPoint &point) {
  Projection selected;
  selected.distance = std::numeric_limits<double>::infinity();
  if (path.size() < 2) return selected;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto &a = path[i - 1];
    const auto &b = path[i];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length2 = dx * dx + dy * dy;
    if (length2 < 1.0e-12) continue;
    const double ratio = std::clamp(
        ((point.x - a.x) * dx + (point.y - a.y) * dy) / length2,
        0.0, 1.0);
    const double signed_distance =
        (dx * (point.y - a.y) - dy * (point.x - a.x)) /
        std::sqrt(length2);
    const double candidate = std::hypot(
        point.x - a.x - ratio * dx,
        point.y - a.y - ratio * dy);
    if (candidate < selected.distance) {
      selected = {i - 1, ratio, candidate, signed_distance};
    }
  }
  return selected;
}

Trajectory lookahead(const Trajectory &path, const TrajectoryPoint &ego,
                     double max_distance, double max_time) {
  Trajectory result;
  if (path.empty()) return result;
  const std::size_t start = project(path, ego).segment;
  double traveled = 0.0;
  double time = 0.0;
  for (std::size_t i = start; i < path.size() &&
       traveled <= max_distance && time <= max_time; ++i) {
    result.push_back(path[i]);
    if (i == start) continue;
    const double step = distance(path[i - 1], path[i]);
    traveled += step;
    const double velocity = std::abs(path[i - 1].longitudinal_velocity_mps);
    if (velocity > 0.0) time += step / velocity;
  }
  return result;
}

double length(const Trajectory &path) {
  double result = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i)
    result += distance(path[i - 1], path[i]);
  return result;
}

double remainingLength(const Trajectory &path,
                       const TrajectoryPoint &ego) {
  if (path.size() < 2) return 0.0;
  const std::size_t start = project(path, ego).segment;
  double result = 0.0;
  for (std::size_t i = start + 1; i < path.size(); ++i)
    result += distance(path[i - 1], path[i]);
  return result;
}

Trajectory resampleAtSpacing(const Trajectory &path, double spacing) {
  if (path.size() < 2 || spacing <= 0.0) return path;
  Trajectory result;
  result.push_back(path.front());
  double traveled = 0.0;
  double next = spacing;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto &a = path[i - 1];
    const auto &b = path[i];
    const double step = distance(a, b);
    if (step < 1e-9) continue;
    while (next <= traveled + step) {
      const double ratio = (next - traveled) / step;
      auto point = a;
      point.x += ratio * (b.x - a.x);
      point.y += ratio * (b.y - a.y);
      result.push_back(point);
      next += spacing;
    }
    traveled += step;
  }
  if (distance(result.back(), path.back()) > 1e-6)
    result.push_back(path.back());
  return result;
}

PlanningMetricStat frechet(const Trajectory &a, const Trajectory &b) {
  PlanningMetricStat result;
  if (a.empty() || b.empty()) return result;
  std::vector<double> previous(b.size(), 0.0);
  std::vector<double> current(b.size(), 0.0);
  for (std::size_t i = 0; i < a.size(); ++i) {
    for (std::size_t j = 0; j < b.size(); ++j) {
      const double d = distance(a[i], b[j]);
      if (i == 0 && j == 0) current[j] = d;
      else if (i == 0) current[j] = std::max(current[j - 1], d);
      else if (j == 0) current[j] = std::max(previous[j], d);
      else current[j] = std::max(
          std::min({previous[j], previous[j - 1], current[j - 1]}), d);
    }
    std::swap(previous, current);
  }
  result.add(previous.back());
  return result;
}

void insert(std::map<std::string, PlanningMetricStat> &output,
            const std::string &name, PlanningMetricStat stat) {
  if (stat.count > 0) output.emplace(name, stat);
}
}  // namespace

void PlanningMetricStat::add(double value) {
  if (count == 0) {
    minimum = maximum = mean = value;
    count = 1;
    return;
  }
  minimum = std::min(minimum, value);
  maximum = std::max(maximum, value);
  mean += (value - mean) / static_cast<double>(++count);
}

PlanningEvaluatorCore::PlanningEvaluatorCore(
    PlanningEvaluatorParameters parameters)
    : parameters_(parameters) {
  if (!(parameters_.minimum_relative_angle_point_distance_m >= 0.0) ||
      !(parameters_.lookahead_distance_m > 0.0) ||
      !(parameters_.lookahead_time_s > 0.0) ||
      !(parameters_.evaluation_time_s > 0.0) ||
      !(parameters_.obstacle_collision_distance_m > 0.0))
    throw std::invalid_argument("invalid evaluator parameters");
}

PlanningEvaluatorResult PlanningEvaluatorCore::tick(
    const PlanningEvaluatorInput &input) {
  PlanningEvaluatorResult output;
  if (input.reference_trajectory) reference_ = *input.reference_trajectory;
  if (input.object_centers) object_centers_ = *input.object_centers;
  if (input.ego) {
    if (input.ego_lane_id)
      output.scalar_metrics["ego_lane_info/lane_id"] =
          static_cast<double>(*input.ego_lane_id);
    if (input.ego_lane_arc_m)
      output.scalar_metrics["ego_lane_info/s"] = *input.ego_lane_arc_m;
    if (input.ego_lane_lateral_m)
      output.scalar_metrics["ego_lane_info/t"] =
          *input.ego_lane_lateral_m;
    if (input.ego_acceleration_mps2) {
      output.scalar_metrics["kinematic_state/vel"] = input.ego_speed_mps;
      output.scalar_metrics["kinematic_state/acc"] =
          *input.ego_acceleration_mps2;
      double jerk = 0.0;
      if (previous_acceleration_ &&
          input.acceleration_stamp_s > previous_acceleration_->first) {
        jerk = (*input.ego_acceleration_mps2 -
                previous_acceleration_->second) /
               (input.acceleration_stamp_s - previous_acceleration_->first);
      }
      output.scalar_metrics["kinematic_state/jerk"] = jerk;
      if (!previous_acceleration_ ||
          input.acceleration_stamp_s > previous_acceleration_->first)
        previous_acceleration_ = std::make_pair(
            input.acceleration_stamp_s, *input.ego_acceleration_mps2);
    }
  }

  if (input.ego && input.trajectory) {
    const auto &path = *input.trajectory;
    PlanningMetricStat curvature, interval, relative_angle, velocity,
        acceleration, jerk, lateral, yaw_deviation, velocity_deviation,
        obstacle_distance, obstacle_ttc, stability, local_displacement,
        lookahead_displacement;
    for (std::size_t i = 0; i < path.size(); ++i) {
      const auto &point = path[i];
      velocity.add(point.longitudinal_velocity_mps);
      acceleration.add(point.acceleration_mps2);
      if (i > 0) interval.add(distance(path[i - 1], point));
      if (!reference_.empty()) {
        const auto &nearest = reference_[nearestPoint(reference_, point)];
        const double ref_yaw = yaw(nearest);
        lateral.add(-std::sin(ref_yaw) * (point.x - nearest.x) +
                    std::cos(ref_yaw) * (point.y - nearest.y));
        yaw_deviation.add(angle(yaw(point) - ref_yaw));
        velocity_deviation.add(
            point.longitudinal_velocity_mps -
            nearest.longitudinal_velocity_mps);
      }
      double nearest_obstacle = std::numeric_limits<double>::max();
      for (const auto &[x, y] : object_centers_)
        nearest_obstacle = std::min(nearest_obstacle,
                                    std::hypot(point.x - x, point.y - y));
      obstacle_distance.add(nearest_obstacle);
      if (i + 1 < path.size()) {
        const double speed = std::abs(point.longitudinal_velocity_mps);
        const double step = distance(point, path[i + 1]);
        if (speed > 0.0 && step > 0.0)
          jerk.add(path[i + 1].acceleration_mps2 -
                   point.acceleration_mps2 / (step / speed));
      }
    }
    for (std::size_t i = 0; i + 2 < path.size(); ++i) {
      const auto &a = path[i];
      const auto &b = path[i + 1];
      const auto &c = path[i + 2];
      if (a.longitudinal_velocity_mps < 0.0f ||
          b.longitudinal_velocity_mps < 0.0f ||
          c.longitudinal_velocity_mps < 0.0f)
        continue;
      const double ab = distance(a, b);
      const double ac = distance(a, c);
      if (ab < parameters_.minimum_relative_angle_point_distance_m ||
          ac < parameters_.minimum_relative_angle_point_distance_m)
        continue;
      relative_angle.add(angle(
          std::atan2(c.y - a.y, c.x - a.x) -
          std::atan2(b.y - a.y, b.x - a.x)));
    }
    for (std::size_t i = 0; i + 2 < path.size(); ++i) {
      auto nextAfter = [&](std::size_t start) {
        for (std::size_t j = start + 1; j < path.size(); ++j)
          if (distance(path[start], path[j]) >= 1.0) return j;
        return start;
      };
      const auto b = nextAfter(i);
      const auto c = nextAfter(b);
      if (b == i || c == b) break;
      const double ab = distance(path[i], path[b]);
      const double bc = distance(path[b], path[c]);
      const double ca = distance(path[c], path[i]);
      const double cross =
          (path[b].x - path[i].x) * (path[c].y - path[i].y) -
          (path[b].y - path[i].y) * (path[c].x - path[i].x);
      if (ab * bc * ca > 1.0e-9)
        curvature.add(2.0 * cross / (ab * bc * ca));
    }
    PlanningMetricStat route_length, duration;
    route_length.add(length(path));
    double time_s = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
      const double speed =
          std::abs(path[i - 1].longitudinal_velocity_mps);
      if (speed > 0.0)
        time_s += distance(path[i - 1], path[i]) / speed;
    }
    duration.add(time_s);
    double ttc_time = 0.0;
    for (std::size_t i = 0; i < path.size(); ++i) {
      if (i > 0 && path[i - 1].longitudinal_velocity_mps != 0.0f)
        ttc_time += distance(path[i - 1], path[i]) /
                    std::abs(path[i - 1].longitudinal_velocity_mps);
      if (path[i == 0 ? 0 : i - 1].longitudinal_velocity_mps == 0.0f)
        continue;
      const bool collision = std::any_of(
          object_centers_.begin(), object_centers_.end(),
          [&](const auto &object) {
            return std::hypot(path[i].x - object.first,
                              path[i].y - object.second) <=
                   parameters_.obstacle_collision_distance_m;
          });
      if (collision) {
        obstacle_ttc.add(ttc_time);
        break;
      }
    }
    if (!previous_.empty() && !path.empty()) {
      const auto old_lateral = project(previous_, *input.ego);
      const auto new_lateral = project(path, *input.ego);
      if (std::isfinite(old_lateral.distance) &&
          std::isfinite(new_lateral.distance))
        local_displacement.add(std::abs(
            old_lateral.signed_distance - new_lateral.signed_distance));
      const auto old_lookahead = lookahead(
          previous_, *input.ego, parameters_.lookahead_distance_m,
          parameters_.lookahead_time_s);
      const auto new_lookahead = lookahead(
          path, *input.ego, parameters_.lookahead_distance_m,
          parameters_.lookahead_time_s);
      if (old_lookahead.size() >= 2) {
        for (const auto &point : new_lookahead)
          stability.add(project(old_lookahead, point).distance);
      }
      insert(output.metrics, "stability_frechet",
             frechet(old_lookahead, new_lookahead));
      const double evaluation_length =
          parameters_.evaluation_time_s * std::abs(input.ego_speed_mps);
      if (remainingLength(previous_, *input.ego) >= evaluation_length &&
          remainingLength(path, *input.ego) >= evaluation_length) {
        const auto prior_lookahead = lookahead(
            previous_, *input.ego, evaluation_length,
            parameters_.evaluation_time_s);
        // Source resampleTrajectory returns its input for interval zero.
        const auto samples = evaluation_length > 0.0 ?
            resampleAtSpacing(prior_lookahead,
                              evaluation_length / 10.0) :
            prior_lookahead;
        for (const auto &point : samples) {
          const auto projection = project(path, point);
          if (std::isfinite(projection.distance))
            lookahead_displacement.add(
                std::abs(projection.signed_distance));
        }
      }
    }
    insert(output.metrics, "curvature", curvature);
    insert(output.metrics, "point_interval", interval);
    insert(output.metrics, "relative_angle", relative_angle);
    insert(output.metrics, "length", route_length);
    insert(output.metrics, "duration", duration);
    insert(output.metrics, "velocity", velocity);
    insert(output.metrics, "acceleration", acceleration);
    insert(output.metrics, "jerk", jerk);
    insert(output.metrics, "lateral_deviation", lateral);
    insert(output.metrics, "yaw_deviation", yaw_deviation);
    insert(output.metrics, "velocity_deviation", velocity_deviation);
    insert(output.metrics, "lateral_trajectory_displacement_local",
           local_displacement);
    insert(output.metrics, "lateral_trajectory_displacement_lookahead",
           lookahead_displacement);
    insert(output.metrics, "stability", stability);
    insert(output.metrics, "obstacle_distance", obstacle_distance);
    insert(output.metrics, "obstacle_ttc", obstacle_ttc);
    previous_ = path;
  }

  if (input.ego && input.modified_goal) {
    const double goal_yaw = input.modified_goal->yaw_rad;
    const double dx = input.ego->x - input.modified_goal->x;
    const double dy = input.ego->y - input.modified_goal->y;
    PlanningMetricStat longitudinal, lateral, yaw_deviation;
    longitudinal.add(std::abs(std::cos(goal_yaw) * dx +
                              std::sin(goal_yaw) * dy));
    lateral.add(std::abs(-std::sin(goal_yaw) * dx +
                         std::cos(goal_yaw) * dy));
    yaw_deviation.add(std::abs(angle(yaw(*input.ego) - goal_yaw)));
    insert(output.metrics, "modified_goal_longitudinal_deviation",
           longitudinal);
    insert(output.metrics, "modified_goal_lateral_deviation", lateral);
    insert(output.metrics, "modified_goal_yaw_deviation", yaw_deviation);
  }
  return output;
}

void PlanningEvaluatorCore::reset() {
  previous_.clear();
  reference_.clear();
  object_centers_.clear();
  previous_acceleration_.reset();
}

}  // namespace planning
