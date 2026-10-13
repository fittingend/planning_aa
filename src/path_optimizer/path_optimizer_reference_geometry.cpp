// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 경로 최적화의 자연 3차 스플라인 형상 계산을 이관했습니다.

#include "planning/path_optimizer_reference_geometry.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {
namespace {

class NaturalSpline {
 public:
  NaturalSpline(const std::vector<double> &keys,
                const std::vector<double> &values)
      : keys_(keys) {
    if (keys.size() < 2 || keys.size() != values.size() ||
        !std::isfinite(keys.front()) || !std::isfinite(values.front())) {
      throw std::invalid_argument("스플라인 기준점 수가 유효하지 않습니다");
    }
    for (std::size_t i = 1; i < keys.size(); ++i) {
      if (!std::isfinite(keys[i]) || keys[i] <= keys[i - 1] ||
          !std::isfinite(values[i])) {
        throw std::invalid_argument("스플라인 기준점 값이 유효하지 않습니다");
      }
    }
    const Eigen::Index n = static_cast<Eigen::Index>(keys.size());
    Eigen::VectorXd second = Eigen::VectorXd::Zero(n);
    if (n > 2) {
      Eigen::VectorXd lower = Eigen::VectorXd::Zero(n - 2);
      Eigen::VectorXd diagonal = Eigen::VectorXd::Zero(n - 2);
      Eigen::VectorXd upper = Eigen::VectorXd::Zero(n - 2);
      Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n - 2);
      for (Eigen::Index i = 1; i < n - 1; ++i) {
        const double h0 = keys[static_cast<std::size_t>(i)] -
                          keys[static_cast<std::size_t>(i - 1)];
        const double h1 = keys[static_cast<std::size_t>(i + 1)] -
                          keys[static_cast<std::size_t>(i)];
        if (h0 <= 0.0 || h1 <= 0.0) {
          throw std::invalid_argument("스플라인 호길이가 증가하지 않습니다");
        }
        const Eigen::Index row = i - 1;
        diagonal(row) = 2.0 * (h0 + h1);
        rhs(row) = 6.0 * ((values[static_cast<std::size_t>(i + 1)] -
                           values[static_cast<std::size_t>(i)]) / h1 -
                          (values[static_cast<std::size_t>(i)] -
                           values[static_cast<std::size_t>(i - 1)]) / h0);
        if (row > 0) lower(row) = h0;
        if (row + 1 < n - 2) upper(row) = h1;
      }
      for (Eigen::Index i = 1; i < n - 2; ++i) {
        const double factor = lower(i) / diagonal(i - 1);
        diagonal(i) -= factor * upper(i - 1);
        rhs(i) -= factor * rhs(i - 1);
      }
      second(n - 2) = rhs(n - 3) / diagonal(n - 3);
      for (Eigen::Index i = n - 4; i >= 0; --i) {
        second(i + 1) =
            (rhs(i) - upper(i) * second(i + 2)) / diagonal(i);
      }
    }
    a_.resize(static_cast<std::size_t>(n - 1));
    b_.resize(static_cast<std::size_t>(n - 1));
    c_.resize(static_cast<std::size_t>(n - 1));
    d_.resize(static_cast<std::size_t>(n - 1));
    for (Eigen::Index i = 0; i < n - 1; ++i) {
      const double h = keys[static_cast<std::size_t>(i + 1)] -
                       keys[static_cast<std::size_t>(i)];
      a_[static_cast<std::size_t>(i)] =
          (second(i + 1) - second(i)) / (6.0 * h);
      b_[static_cast<std::size_t>(i)] = second(i) / 2.0;
      c_[static_cast<std::size_t>(i)] =
          (values[static_cast<std::size_t>(i + 1)] -
           values[static_cast<std::size_t>(i)]) / h -
          h * (2.0 * second(i) + second(i + 1)) / 6.0;
      d_[static_cast<std::size_t>(i)] = values[static_cast<std::size_t>(i)];
    }
  }

  double evaluate(double arc, int derivative = 0) const {
    arc = std::clamp(arc, keys_.front(), keys_.back());
    const auto next = std::lower_bound(keys_.begin(), keys_.end(), arc);
    const std::size_t i = std::clamp<std::size_t>(
        static_cast<std::size_t>(next - keys_.begin()) == 0 ? 0 :
        static_cast<std::size_t>(next - keys_.begin()) - 1,
        0, a_.size() - 1);
    const double h = arc - keys_[i];
    if (derivative == 2) return 6.0 * a_[i] * h + 2.0 * b_[i];
    if (derivative == 1) return 3.0 * a_[i] * h * h +
                                2.0 * b_[i] * h + c_[i];
    return ((a_[i] * h + b_[i]) * h + c_[i]) * h + d_[i];
  }

 private:
  std::vector<double> keys_;
  std::vector<double> a_, b_, c_, d_;
};

std::vector<double> arcsOf(const Trajectory &points) {
  std::vector<double> arcs(points.size(), 0.0);
  for (std::size_t i = 1; i < points.size(); ++i) {
    arcs[i] = arcs[i - 1] +
        std::hypot(points[i].x - points[i - 1].x,
                   points[i].y - points[i - 1].y);
  }
  return arcs;
}

Trajectory sanitize(const Trajectory &points) {
  Trajectory result;
  for (const auto &point : points) {
    if (!result.empty() &&
        std::abs(point.x - result.back().x) < 1.0e-6 &&
        std::abs(point.y - result.back().y) < 1.0e-6) continue;
    result.push_back(point);
  }
  return result;
}

Trajectory resample(const Trajectory &points, double interval) {
  const auto input = sanitize(points);
  const auto arcs = arcsOf(input);
  std::vector<double> x, y;
  for (const auto &point : input) {
    x.push_back(point.x);
    y.push_back(point.y);
  }
  const NaturalSpline spline_x(arcs, x);
  const NaturalSpline spline_y(arcs, y);
  std::vector<double> samples;
  for (double arc = 0.0; arc < arcs.back(); arc += interval) samples.push_back(arc);
  if (samples.empty()) samples.push_back(0.0);
  if (arcs.back() - samples.back() < 0.1) samples.back() = arcs.back();
  else samples.push_back(arcs.back());
  Trajectory result;
  result.reserve(samples.size());
  bool stopped = false;
  for (double arc : samples) {
    const auto next = std::upper_bound(arcs.begin(), arcs.end(), arc);
    const std::size_t index = std::min<std::size_t>(
        static_cast<std::size_t>(next - arcs.begin()) == 0 ? 0 :
        static_cast<std::size_t>(next - arcs.begin()) - 1,
        input.size() - 2);
    const double ratio = (arc - arcs[index]) /
                         (arcs[index + 1] - arcs[index]);
    auto point = input[index];
    point.x = spline_x.evaluate(arc);
    point.y = spline_y.evaluate(arc);
    point.z = input[index].z +
        ratio * (input[index + 1].z - input[index].z);
    if (std::abs(input[index].longitudinal_velocity_mps) < 1.0e-4) stopped = true;
    if (stopped) point.longitudinal_velocity_mps = 0.0F;
    result.push_back(point);
  }
  if (result.size() < 2) return input;
  for (std::size_t i = 0; i < result.size(); ++i) {
    const std::size_t segment = std::min(i, result.size() - 2);
    const double yaw = std::atan2(
        result[segment + 1].y - result[segment].y,
        result[segment + 1].x - result[segment].x);
    result[i].orientation_x = 0.0;
    result[i].orientation_y = 0.0;
    result[i].orientation_z = std::sin(yaw / 2.0);
    result[i].orientation_w = std::cos(yaw / 2.0);
  }
  return result;
}

std::size_t nearestSegment(const Trajectory &points, const TrajectoryPoint &ego,
                           double distance_limit, double yaw_limit) {
  const auto yaw_of = [](const TrajectoryPoint &point) {
    return std::atan2(2.0 * (point.orientation_w * point.orientation_z +
        point.orientation_x * point.orientation_y),
        1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                    point.orientation_z * point.orientation_z));
  };
  const auto search = [&](bool check_distance, bool check_yaw) {
    std::size_t index = 0;
    double best = std::numeric_limits<double>::infinity();
    bool found = false;
    for (std::size_t i = 0; i < points.size(); ++i) {
      const double distance = std::hypot(points[i].x - ego.x, points[i].y - ego.y);
      const double yaw_difference = std::abs(std::remainder(
          yaw_of(points[i]) - yaw_of(ego), 2.0 * 3.141592653589793));
      if ((check_distance && distance > distance_limit) ||
          (check_yaw && yaw_difference > yaw_limit)) {
        if (found) break;
        continue;
      }
      if (distance < best) { best = distance; index = i; found = true; }
    }
    return std::pair<bool, std::size_t>{found, index};
  };
  auto result = search(true, true);
  if (!result.first) result = search(true, false);
  if (!result.first) result = search(false, false);
  const std::size_t nearest = result.second;
  if (nearest == 0) return 0;
  if (nearest + 1 == points.size()) return points.size() - 2;
  const double dx = points[nearest + 1].x - points[nearest].x;
  const double dy = points[nearest + 1].y - points[nearest].y;
  const double dot = dx * (ego.x - points[nearest].x) +
                     dy * (ego.y - points[nearest].y);
  return dot <= 0.0 ? nearest - 1 : nearest;
}

double segmentOffset(const Trajectory &points, std::size_t i,
                     const TrajectoryPoint &ego) {
  const double dx = points[i + 1].x - points[i].x;
  const double dy = points[i + 1].y - points[i].y;
  const double length = std::hypot(dx, dy);
  return length < 1.0e-9 ? 0.0 :
      (dx * (ego.x - points[i].x) + dy * (ego.y - points[i].y)) / length;
}

Trajectory crop(const Trajectory &points, const TrajectoryPoint &ego,
                double forward, double backward,
                double distance_limit, double yaw_limit) {
  std::size_t segment = nearestSegment(points, ego, distance_limit, yaw_limit);
  double length = -segmentOffset(points, segment, ego);
  std::size_t end = points.size();
  for (std::size_t i = segment + 1; i < points.size(); ++i) {
    length += std::hypot(points[i].x - points[i - 1].x,
                         points[i].y - points[i - 1].y);
    if (forward < length) { end = i; break; }
  }
  if (end < 2) end = 2;
  Trajectory result(points.begin(), points.begin() + end);
  segment = std::min(segment, result.size() - 2);
  length = -segmentOffset(result, segment, ego);
  for (std::size_t i = segment; i > 0; --i) {
    length -= std::hypot(result[i].x - result[i - 1].x,
                         result[i].y - result[i - 1].y);
    if (length < -backward) {
      result.erase(result.begin(), result.begin() + i);
      break;
    }
  }
  return result;
}

std::vector<double> interpolateCurvature(
    const Trajectory &input, const std::vector<double> &curvature,
    const Trajectory &output) {
  const auto input_arcs = arcsOf(input);
  const auto output_arcs = arcsOf(output);
  std::vector<double> values;
  values.reserve(output.size());
  for (const double query : output_arcs) {
    const auto next = std::upper_bound(input_arcs.begin(), input_arcs.end(), query);
    const std::size_t i = std::min<std::size_t>(
        static_cast<std::size_t>(next - input_arcs.begin()) == 0 ? 0 :
        static_cast<std::size_t>(next - input_arcs.begin()) - 1,
        input.size() - 2);
    const double ratio = std::clamp(
        (query - input_arcs[i]) / (input_arcs[i + 1] - input_arcs[i]),
        0.0, 1.0);
    values.push_back(curvature[i] + ratio * (curvature[i + 1] - curvature[i]));
  }
  return values;
}

void attachPreviousReference(
    Trajectory &points, std::vector<double> &curvature,
    std::vector<std::optional<std::array<double, 2>>> &fixed_states,
    const PathOptimizerReferenceGeometry &previous, double interval,
    double distance_limit, double yaw_limit) {
  if (previous.points.size() < 2 ||
      previous.curvature.size() != previous.points.size()) {
    throw std::invalid_argument("이전 경로 최적화 기준점이 유효하지 않습니다");
  }
  const std::size_t index = nearestSegment(
      previous.points, points.front(), distance_limit, yaw_limit);
  const auto &fixed = previous.points[index];
  const auto segment = nearestSegment(points, fixed, distance_limit, yaw_limit);
  double longitude = segmentOffset(points, segment, fixed);
  for (std::size_t i = 0; i < segment; ++i) {
    longitude += std::hypot(points[i + 1].x - points[i].x,
                            points[i + 1].y - points[i].y);
  }
  if (longitude > 0.0) return;
  const double distance = std::hypot(fixed.x - points.front().x,
                                     fixed.y - points.front().y);
  if (distance < interval) {
    points.front() = fixed;
    curvature.front() = previous.curvature[index];
  } else {
    points.insert(points.begin(), fixed);
    curvature.insert(curvature.begin(), previous.curvature[index]);
  }
  const auto front = points.front();
  const double front_curvature = curvature.front();
  if (index > 0) {
    points.insert(points.begin(), previous.points[index - 1]);
    curvature.insert(curvature.begin(), previous.curvature[index - 1]);
  }
  const auto original = points;
  const auto original_curvature = curvature;
  points = resample(original, interval);
  curvature = interpolateCurvature(original, original_curvature, points);
  if (index > 0) {
    points[0] = previous.points[index - 1];
    points[1] = previous.points[index];
    curvature[0] = previous.curvature[index - 1];
    curvature[1] = previous.curvature[index];
    if (previous.optimized_states.size() == previous.points.size()) {
      fixed_states.resize(points.size());
      fixed_states[0] = previous.optimized_states[index - 1];
      fixed_states[1] = previous.optimized_states[index];
    }
  } else {
    points[0] = front;
    curvature[0] = front_curvature;
    if (previous.optimized_states.size() == previous.points.size()) {
      fixed_states.resize(points.size());
      fixed_states[0] = previous.optimized_states[index];
    }
  }
}

}  // 익명 네임스페이스

std::vector<std::vector<PathOptimizerSplinePose>> samplePathOptimizerSplineOffsets(
    const Trajectory &points, const std::vector<double> &offsets) {
  if (points.size() < 2) {
    throw std::invalid_argument("경로 최적화 스플라인 기준점이 부족합니다");
  }
  const auto arcs = arcsOf(points);
  std::vector<double> x, y;
  x.reserve(points.size());
  y.reserve(points.size());
  for (const auto &point : points) {
    x.push_back(point.x);
    y.push_back(point.y);
  }
  const NaturalSpline spline_x(arcs, x);
  const NaturalSpline spline_y(arcs, y);
  std::vector<std::vector<PathOptimizerSplinePose>> samples(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    for (const double offset : offsets) {
      if (!std::isfinite(offset)) {
        throw std::invalid_argument("경로 최적화 원 위치가 유효하지 않습니다");
      }
      const double query = std::clamp(arcs[i] + offset, arcs.front(), arcs.back());
      samples[i].push_back({spline_x.evaluate(query), spline_y.evaluate(query),
          std::atan2(spline_y.evaluate(query, 1), spline_x.evaluate(query, 1))});
    }
  }
  return samples;
}

PathOptimizerReferenceGeometry preparePathOptimizerReferenceGeometry(
    const Trajectory &input, const TrajectoryPoint &ego,
    const PathOptimizerReferenceGeometryParameters &p,
    const PathOptimizerReferenceGeometry *previous) {
  if (input.size() < 2 || p.maximum_points < 2 ||
      !std::isfinite(p.interval) || p.interval <= 0.0 ||
      !std::isfinite(p.forward_length) || p.forward_length <= 0.0 ||
      !std::isfinite(p.backward_length) || p.backward_length < 0.0 ||
      !std::isfinite(p.margin) || p.margin < 0.0 ||
      !std::isfinite(p.nearest_distance) || p.nearest_distance < 0.0 ||
      !std::isfinite(p.nearest_yaw) || p.nearest_yaw < 0.0) {
    throw std::invalid_argument("경로 최적화 기준 경로 입력이 유효하지 않습니다");
  }
  auto points = resample(input, p.interval);
  points = sanitize(crop(points, ego, p.forward_length + p.margin,
                         p.backward_length + p.margin,
                         p.nearest_distance, p.nearest_yaw));
  const auto arcs = arcsOf(points);
  std::vector<double> x, y;
  for (const auto &point : points) {
    x.push_back(point.x);
    y.push_back(point.y);
  }
  const NaturalSpline spline_x(arcs, x);
  const NaturalSpline spline_y(arcs, y);
  std::vector<double> curvature;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double dx = spline_x.evaluate(arcs[i], 1);
    const double dy = spline_y.evaluate(arcs[i], 1);
    const double ddx = spline_x.evaluate(arcs[i], 2);
    const double ddy = spline_y.evaluate(arcs[i], 2);
    const double yaw = std::atan2(dy, dx);
    points[i].orientation_x = 0.0;
    points[i].orientation_y = 0.0;
    points[i].orientation_z = std::sin(yaw / 2.0);
    points[i].orientation_w = std::cos(yaw / 2.0);
    curvature.push_back((dx * ddy - ddx * dy) /
                        std::pow(dx * dx + dy * dy, 1.5));
  }
  auto cropped = crop(points, ego, p.forward_length + p.margin,
                      p.backward_length, p.nearest_distance, p.nearest_yaw);
  // 두 번째 자르기는 앞쪽 점만 제거하므로 좌표로 대응 곡률을 찾습니다.
  std::size_t first = 0;
  while (first < points.size() &&
         (std::abs(points[first].x - cropped.front().x) > 1.0e-9 ||
          std::abs(points[first].y - cropped.front().y) > 1.0e-9)) ++first;
  if (first == points.size()) throw std::runtime_error("기준점 자르기 대응 실패");
  curvature.erase(curvature.begin(), curvature.begin() + first);
  std::vector<std::optional<std::array<double, 2>>> fixed_states;
  if (previous) attachPreviousReference(
      cropped, curvature, fixed_states, *previous, p.interval,
      p.nearest_distance, p.nearest_yaw);
  if (cropped.size() > p.maximum_points) cropped.resize(p.maximum_points);
  curvature.resize(cropped.size());
  fixed_states.resize(cropped.size());
  PathOptimizerReferenceGeometry result;
  result.points = std::move(cropped);
  result.curvature = std::move(curvature);
  result.fixed_states = std::move(fixed_states);
  result.arc_length_to_next.resize(result.points.size(), 0.0);
  for (std::size_t i = 0; i + 1 < result.points.size(); ++i) {
    result.arc_length_to_next[i] = std::hypot(
        result.points[i + 1].x - result.points[i].x,
        result.points[i + 1].y - result.points[i].y);
  }
  return result;
}

}  // planning 네임스페이스
