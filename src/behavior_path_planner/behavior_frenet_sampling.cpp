// SPDX-License-Identifier: Apache-2.0
// 원본 Frenet 표본기의 삼차 기준 스플라인과 오차 다항식을 계산합니다.

#include "planning/behavior_frenet_sampling.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace planning {
namespace {

struct Spline {
  std::vector<double> a, b, c, d;
  bool reset(const std::vector<double> &s, const std::vector<double> &values) {
    if (s.size() != values.size() || s.size() < 2) return false;
    a = values;
    const std::size_t n = s.size();
    std::vector<double> h(n - 1);
    for (std::size_t i = 0; i + 1 < n; ++i) {
      h[i] = s[i + 1] - s[i];
      if (!std::isfinite(h[i]) || h[i] <= 1.0e-8) return false;
    }
    c.assign(n, 0.0);
    std::vector<double> next(n, 0.0);
    for (std::size_t iteration = 0; iteration <= 100; ++iteration) {
      double change = 0.0;
      for (std::size_t i = 1; i + 1 < n; ++i) {
        const double rhs = 3.0 / h[i] * (a[i + 1] - a[i]) -
            3.0 / h[i - 1] * (a[i] - a[i - 1]);
        next[i] += 1.8 / (2.0 * (h[i - 1] + h[i])) *
            (rhs - h[i - 1] * next[i - 1] -
             2.0 * (h[i - 1] + h[i]) * c[i] - h[i] * c[i + 1]);
      }
      for (std::size_t i = 0; i < n; ++i)
        change += std::abs(next[i] - c[i]);
      c = next;
      if (change < 1.0e-5) break;
    }
    b.assign(n, 0.0);
    d.assign(n, 0.0);
    for (std::size_t i = 0; i + 1 < n; ++i) {
      d[i] = (c[i + 1] - c[i]) / (3.0 * h[i]);
      b[i] = (a[i + 1] - a[i]) / h[i] -
          h[i] * (2.0 * c[i] + c[i + 1]) / 3.0;
    }
    return true;
  }
  double value(const std::vector<double> &s, double query) const {
    const auto upper = std::upper_bound(s.begin(), s.end(), query);
    const std::size_t i = std::clamp<std::size_t>(
        upper == s.begin() ? 0 : static_cast<std::size_t>(upper - s.begin() - 1),
        0, s.size() - 2);
    const double ds = query - s[i];
    return a[i] + (b[i] + (c[i] + d[i] * ds) * ds) * ds;
  }
  double velocity(const std::vector<double> &s, double query) const {
    const auto lower = std::lower_bound(s.begin(), s.end(), query);
    const std::size_t i = std::clamp<std::size_t>(
        lower == s.begin() ? 0 : static_cast<std::size_t>(lower - s.begin() - 1),
        0, s.size() - 2);
    const double ds = query - s[i];
    return b[i] + 2.0 * c[i] * ds + 3.0 * d[i] * ds * ds;
  }
  double acceleration(const std::vector<double> &s,
                      double query) const {
    const auto lower = std::lower_bound(s.begin(), s.end(), query);
    const std::size_t i = std::clamp<std::size_t>(
        lower == s.begin() ? 0 : static_cast<std::size_t>(lower - s.begin() - 1),
        0, s.size() - 2);
    return 2.0 * c[i] + 6.0 * d[i] * (query - s[i]);
  }
};

struct Quintic {
  std::array<double, 6> coefficient{};
  bool reset(double x0, double v0, double a0,
             double x1, double v1, double a1, double duration) {
    if (!std::isfinite(duration) || duration <= 1.0e-8) return false;
    coefficient[0] = x0;
    coefficient[1] = v0;
    coefficient[2] = a0 / 2.0;
    const double t = duration;
    const double position = x1 - x0 - v0 * t - a0 * t * t / 2.0;
    const double velocity = v1 - v0 - a0 * t;
    const double acceleration = a1 - a0;
    coefficient[3] = (10.0 * position - 4.0 * velocity * t +
                      acceleration * t * t / 2.0) / (t * t * t);
    coefficient[4] = (-15.0 * position + 7.0 * velocity * t -
                      acceleration * t * t) / (t * t * t * t);
    coefficient[5] = (6.0 * position - 3.0 * velocity * t +
                      acceleration * t * t / 2.0) / (t * t * t * t * t);
    return true;
  }
  double value(double t) const {
    return coefficient[0] + t * (coefficient[1] + t *
        (coefficient[2] + t * (coefficient[3] + t *
        (coefficient[4] + t * coefficient[5]))));
  }
};

}  // 익명 네임스페이스

std::optional<BehaviorFrenetState> projectBehaviorFrenetState(
    const Trajectory &reference, const TrajectoryPoint &pose) {
  if (reference.size() < 2 || !std::isfinite(pose.x) ||
      !std::isfinite(pose.y)) return std::nullopt;
  std::vector<double> s{0.0}, x, y;
  std::size_t nearest = 0;
  double minimum = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < reference.size(); ++i) {
    const auto &point = reference[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y))
      return std::nullopt;
    x.push_back(point.x);
    y.push_back(point.y);
    if (i != 0) s.push_back(s.back() + std::hypot(
        point.x - reference[i - 1].x,
        point.y - reference[i - 1].y));
    const double distance = std::hypot(pose.x - point.x,
                                       pose.y - point.y);
    if (distance <= minimum) {
      minimum = distance;
      nearest = i;
    }
  }
  Spline sx, sy;
  if (!sx.reset(s, x) || !sy.reset(s, y)) return std::nullopt;
  const auto distance_at = [&](double arc) {
    return std::hypot(pose.x - sx.value(s, arc),
                      pose.y - sy.value(s, arc));
  };
  std::array<double, 5> arc{
      s[nearest == 0 ? 0 : nearest - 1], 0.0,
      s[nearest], 0.0,
      s[std::min(nearest + 1, s.size() - 1)]};
  std::array<double, 5> distance{
      distance_at(arc[0]), 0.0, distance_at(arc[2]),
      0.0, distance_at(arc[4])};
  double best_s = arc[2];
  for (int iteration = 0;
       iteration < 50 && arc[4] - arc[0] > 0.01;
       ++iteration) {
    arc[1] = 0.5 * (arc[0] + arc[2]);
    arc[3] = 0.5 * (arc[2] + arc[4]);
    distance[1] = distance_at(arc[1]);
    distance[3] = distance_at(arc[3]);
    std::size_t best = 0;
    for (std::size_t i = 1; i < distance.size(); ++i)
      if (distance[i] <= distance[best]) best = i;
    best_s = arc[best];
    const std::size_t low = best == 0 ? 0 : best - 1;
    const std::size_t high = std::min(best + 1, arc.size() - 1);
    arc = {arc[low], 0.0, arc[best], 0.0, arc[high]};
    distance = {distance[low], 0.0, distance[best],
                0.0, distance[high]};
  }
  const double vx = sx.velocity(s, best_s);
  const double vy = sy.velocity(s, best_s);
  const double speed_squared = vx * vx + vy * vy;
  if (speed_squared <= 1.0e-12) return std::nullopt;
  const double signed_cross = vx *
      (pose.y - sy.value(s, best_s)) - vy *
      (pose.x - sx.value(s, best_s));
  const double lateral = std::copysign(
      distance_at(best_s), signed_cross);
  const double yaw = std::atan2(vy, vx);
  const double ego_yaw = std::atan2(
      2.0 * (pose.orientation_w * pose.orientation_z +
             pose.orientation_x * pose.orientation_y),
      1.0 - 2.0 * (pose.orientation_y * pose.orientation_y +
                   pose.orientation_z * pose.orientation_z));
  const double yaw_delta = std::remainder(
      ego_yaw - yaw, 2.0 * std::acos(-1.0));
  const auto curvature_at = [&](double query) {
    const double dx = sx.velocity(s, query);
    const double dy = sy.velocity(s, query);
    const double ddx = sx.acceleration(s, query);
    const double ddy = sy.acceleration(s, query);
    const double denominator = std::pow(dx * dx + dy * dy, 1.5);
    return denominator > 1.0e-12 ?
        (ddy * dx - ddx * dy) / denominator : 0.0;
  };
  const double curvature = curvature_at(best_s);
  BehaviorFrenetState state;
  state.s_m = best_s;
  state.lateral_m = lateral;
  state.lateral_derivative =
      (1.0 - curvature * lateral) * std::tan(yaw_delta);
  const double delta_s = 0.001;
  const double derivative = (
      curvature_at(std::min(s.back(), best_s + delta_s)) -
      curvature) / delta_s;
  const double cosine = std::cos(yaw_delta);
  if (std::abs(cosine) > 1.0e-8) {
    state.lateral_second_derivative =
        -(derivative * lateral +
          curvature * state.lateral_derivative) *
            std::tan(yaw_delta) -
        ((1.0 - curvature * lateral) /
         (cosine * cosine)) * curvature;
  }
  if (!std::isfinite(state.lateral_derivative) ||
      !std::isfinite(state.lateral_second_derivative))
    return std::nullopt;
  return state;
}

std::vector<BehaviorFrenetCandidate> generateBehaviorFrenetCandidates(
    const BehaviorFrenetSamplingInput &input) {
  std::vector<BehaviorFrenetCandidate> result;
  if (input.reference.size() < 2 ||
      !std::isfinite(input.resolution_m) || input.resolution_m <= 0.0 ||
      !std::isfinite(input.initial.s_m) ||
      !std::isfinite(input.initial.lateral_m) ||
      !std::isfinite(input.initial.lateral_derivative) ||
      !std::isfinite(input.initial.lateral_second_derivative)) return result;
  std::vector<double> s{0.0}, x, y;
  x.reserve(input.reference.size());
  y.reserve(input.reference.size());
  for (std::size_t i = 0; i < input.reference.size(); ++i) {
    const auto &point = input.reference[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) return result;
    x.push_back(point.x);
    y.push_back(point.y);
    if (i != 0) s.push_back(s.back() + std::hypot(
        point.x - input.reference[i - 1].x,
        point.y - input.reference[i - 1].y));
  }
  Spline sx, sy;
  if (!sx.reset(s, x) || !sy.reset(s, y)) return result;
  for (const double target_length : input.target_lengths_m) {
    if (!std::isfinite(target_length)) continue;
    const double target_s = std::min(
        s.back(), input.initial.s_m + std::max(0.0, target_length));
    const double duration = target_s - input.initial.s_m;
    if (duration <= input.resolution_m) continue;
    for (const double lateral : input.target_lateral_positions_m) {
      for (const double derivative : input.target_lateral_derivatives) {
        for (const double second : input.target_lateral_second_derivatives) {
          if (!std::isfinite(lateral) || !std::isfinite(derivative) ||
              !std::isfinite(second)) continue;
          Quintic polynomial;
          if (!polynomial.reset(
                  input.initial.lateral_m,
                  input.initial.lateral_derivative,
                  input.initial.lateral_second_derivative,
                  lateral, derivative, second, duration)) continue;
          BehaviorFrenetCandidate candidate;
          candidate.target_s_m = target_s;
          candidate.target_lateral_m = lateral;
          for (double ds = input.resolution_m;
               ds <= duration + 1.0e-9; ds += input.resolution_m) {
            const double arc = std::min(target_s, input.initial.s_m + ds);
            const double yaw = std::atan2(
                sy.velocity(s, arc), sx.velocity(s, arc));
            const double offset = polynomial.value(ds);
            TrajectoryPoint point;
            point.x = sx.value(s, arc) - offset * std::sin(yaw);
            point.y = sy.value(s, arc) + offset * std::cos(yaw);
            point.z = input.reference.front().z;
            point.longitudinal_velocity_mps =
                input.reference.front().longitudinal_velocity_mps;
            candidate.path.push_back(point);
          }
          if (candidate.path.size() < 2) continue;
          candidate.arc_lengths_m.push_back(0.0);
          std::vector<double> yaws;
          for (std::size_t i = 0; i + 1 < candidate.path.size(); ++i) {
            const auto &a = candidate.path[i];
            const auto &b = candidate.path[i + 1];
            const double dx = b.x - a.x;
            const double dy = b.y - a.y;
            yaws.push_back(std::atan2(dy, dx));
            candidate.arc_lengths_m.push_back(
                candidate.arc_lengths_m.back() + std::hypot(dx, dy));
          }
          yaws.push_back(yaws.back());
          for (std::size_t i = 0; i < candidate.path.size(); ++i) {
            candidate.path[i].orientation_z = std::sin(yaws[i] / 2.0);
            candidate.path[i].orientation_w = std::cos(yaws[i] / 2.0);
            if (i == 0) continue;
            const double distance = candidate.arc_lengths_m[i] -
                candidate.arc_lengths_m[i - 1];
            candidate.curvatures_per_m.push_back(distance > 1.0e-8 ?
                std::remainder(yaws[i] - yaws[i - 1], 2.0 * std::acos(-1.0)) /
                    distance : 0.0);
          }
          candidate.curvatures_per_m.push_back(
              candidate.curvatures_per_m.back());
          result.push_back(std::move(candidate));
        }
      }
    }
    if (target_s >= s.back()) break;
  }
  return result;
}

}  // planning 네임스페이스
