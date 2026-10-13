// SPDX-License-Identifier: Apache-2.0
// Analytical 스무더의 횡가속도 상한과 전방 저크 피드백입니다.

#include "planning/analytical_velocity_smoother.hpp"
#include "planning/jerk_resample.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <vector>

namespace planning {
namespace {

double distance(const TrajectoryPoint &first, const TrajectoryPoint &last) {
  return std::hypot(last.x - first.x, last.y - first.y);
}

double curvature(const TrajectoryPoint &first,
                 const TrajectoryPoint &middle,
                 const TrajectoryPoint &last) {
  const double ab = distance(first, middle);
  const double bc = distance(middle, last);
  const double ca = distance(last, first);
  if (ab * bc * ca < 1.0e-9) return 0.0;
  return 2.0 * ((middle.x - first.x) * (last.y - first.y) -
                (middle.y - first.y) * (last.x - first.x)) /
         (ab * bc * ca);
}

}  // 익명 네임스페이스

Trajectory applyAnalyticalLateralAccelerationLimit(
    const Trajectory &input,
    double sample_interval_m,
    double maximum_lateral_acceleration_mps2,
    double minimum_curve_velocity_mps,
    double before_curve_m,
    double after_curve_m) {
  if (!std::isfinite(sample_interval_m) || sample_interval_m <= 0.0 ||
      !std::isfinite(maximum_lateral_acceleration_mps2) ||
      maximum_lateral_acceleration_mps2 <= 0.0 ||
      !std::isfinite(minimum_curve_velocity_mps) ||
      minimum_curve_velocity_mps < 0.0 ||
      !std::isfinite(before_curve_m) || before_curve_m < 0.0 ||
      !std::isfinite(after_curve_m) || after_curve_m < 0.0) {
    throw std::invalid_argument("Analytical 횡가속도 설정이 유효하지 않습니다");
  }
  if (input.size() < 3) return input;
  auto output = resampleJerkUniform(input, sample_interval_m);
  if (output.size() < 3) return output;
  const auto index_distance = static_cast<std::size_t>(
      std::max(1, static_cast<int>(5.0 / sample_interval_m)));
  std::vector<double> bends(output.size(), 0.0);
  for (std::size_t index = 1; index + 1 < output.size(); ++index) {
    const auto back = index - std::min(index, index_distance);
    const auto front = index +
        std::min(output.size() - 1 - index, index_distance);
    bends[index] = curvature(output[back], output[index], output[front]);
  }
  bends.front() = bends[1];
  bends.back() = bends[bends.size() - 2];
  const auto before = static_cast<std::size_t>(
      std::round(before_curve_m / sample_interval_m));
  const auto after = static_cast<std::size_t>(
      std::round(after_curve_m / sample_interval_m));
  for (std::size_t index = 0; index < output.size(); ++index) {
    const auto first = index > after ? index - after : 0;
    const auto last = std::min(output.size(), index + before + 1);
    double bend = 0.0;
    for (std::size_t other = first; other < last; ++other) {
      bend = std::max(bend, std::fabs(bends[other]));
    }
    const double limit = std::max(
        std::sqrt(maximum_lateral_acceleration_mps2 /
                  std::max(bend, 1.0e-5)),
        minimum_curve_velocity_mps);
    output[index].longitudinal_velocity_mps = static_cast<float>(
        std::min(static_cast<double>(output[index].longitudinal_velocity_mps),
                 limit));
  }
  return output;
}

Trajectory applyAnalyticalForwardJerkFilter(
    const Trajectory &reference, std::size_t start_index,
    double initial_velocity_mps, double initial_acceleration_mps2,
    const AnalyticalForwardParameters &parameters) {
  if (reference.empty() || start_index >= reference.size() ||
      !std::isfinite(initial_velocity_mps) ||
      !std::isfinite(initial_acceleration_mps2) ||
      !std::isfinite(parameters.feedback_gain) ||
      parameters.feedback_gain < 0.0 ||
      parameters.min_acceleration_mps2 > parameters.max_acceleration_mps2 ||
      parameters.min_jerk_mps3 > parameters.max_jerk_mps3) {
    throw std::invalid_argument("Analytical 전방 속도 입력이 유효하지 않습니다");
  }
  Trajectory output = reference;
  output[start_index].longitudinal_velocity_mps =
      static_cast<float>(initial_velocity_mps);
  output[start_index].acceleration_mps2 =
      static_cast<float>(initial_acceleration_mps2);
  for (std::size_t index = start_index + 1;
       index < reference.size(); ++index) {
    const double velocity = output[index - 1].longitudinal_velocity_mps;
    const double ds = distance(reference[index - 1], reference[index]);
    const double dt = ds / std::max(velocity, 1.0);
    if (dt <= 1.0e-9) {
      output[index].longitudinal_velocity_mps =
          output[index - 1].longitudinal_velocity_mps;
      output[index].acceleration_mps2 = output[index - 1].acceleration_mps2;
      continue;
    }
    const double acceleration = output[index - 1].acceleration_mps2;
    const double next_velocity = std::max(velocity + acceleration * dt, 0.0);
    const double error =
        reference[index].longitudinal_velocity_mps - next_velocity;
    const double target_acceleration = std::clamp(
        parameters.feedback_gain * error,
        parameters.min_acceleration_mps2,
        parameters.max_acceleration_mps2);
    const double jerk = std::clamp(
        (target_acceleration - acceleration) / dt,
        parameters.min_jerk_mps3,
        parameters.max_jerk_mps3);
    output[index].longitudinal_velocity_mps =
        static_cast<float>(next_velocity);
    output[index].acceleration_mps2 =
        static_cast<float>(acceleration + jerk * dt);
  }
  return output;
}

namespace {

struct StopPlan {
  int type = 0;
  std::vector<double> times;
  double distance_m = 0.0;
};

double integratePosition(double x, double v, double a,
                         double jerk, double time) {
  return x + v * time + 0.5 * a * time * time +
         jerk * time * time * time / 6.0;
}

double integrateVelocity(double v, double a, double jerk, double time) {
  return v + a * time + 0.5 * jerk * time * time;
}

std::optional<StopPlan> calculateStopPlan(
    double velocity, double acceleration, double jerk,
    double minimum_acceleration, double target_velocity) {
  const double rising_jerk = -jerk;
  if (jerk >= 0.0 || rising_jerk <= 0.0 || minimum_acceleration >= 0.0)
    return std::nullopt;
  StopPlan plan;
  const double minimum_duration =
      (target_velocity - velocity -
       0.5 * (0.0 - acceleration) / jerk * acceleration -
       0.5 * minimum_acceleration / jerk * minimum_acceleration -
       0.5 * (0.0 - minimum_acceleration) / rising_jerk *
           minimum_acceleration) / minimum_acceleration;
  double end_velocity = 0.0;
  double end_acceleration = 0.0;
  if (minimum_duration > 0.0) {
    const double t1 = std::max(0.0,
        (minimum_acceleration - acceleration) / jerk < 0.01 ?
        0.0 : (minimum_acceleration - acceleration) / jerk);
    const double a1 = acceleration + jerk * t1;
    const double v1 = integrateVelocity(velocity, acceleration, jerk, t1);
    const double x1 = integratePosition(0.0, velocity, acceleration, jerk, t1);
    const double t2 = minimum_duration < 0.01 ? 0.0 : minimum_duration;
    const double a2 = a1;
    const double v2 = integrateVelocity(v1, a1, 0.0, t2);
    const double x2 = integratePosition(x1, v1, a1, 0.0, t2);
    const double t3 =
        (0.0 - minimum_acceleration) / rising_jerk < 0.01 ? 0.0 :
        (0.0 - minimum_acceleration) / rising_jerk;
    end_acceleration = a2 + rising_jerk * t3;
    end_velocity = integrateVelocity(v2, a2, rising_jerk, t3);
    plan.distance_m = integratePosition(x2, v2, a2, rising_jerk, t3);
    plan.type = 1;
    plan.times = {t1, t2, t3};
  } else {
    const double deceleration_needed =
        0.5 * (0.0 - acceleration) / rising_jerk * acceleration -
        (target_velocity - velocity);
    if (deceleration_needed > 0.0 || acceleration > 0.0) {
      const double squared =
          (target_velocity - velocity -
           0.5 * (0.0 - acceleration) / jerk * acceleration) *
          (2.0 * rising_jerk * jerk / (rising_jerk - jerk));
      if (squared < 0.0) return std::nullopt;
      const double a1_target = -std::sqrt(squared);
      const double t1_raw = (a1_target - acceleration) / jerk;
      const double t1 = t1_raw < 0.01 ? 0.0 : t1_raw;
      const double a1 = acceleration + jerk * t1;
      const double v1 = integrateVelocity(velocity, acceleration, jerk, t1);
      const double x1 = integratePosition(0.0, velocity, acceleration, jerk, t1);
      const double t2_raw = (0.0 - a1_target) / rising_jerk;
      const double t2 = t2_raw < 0.01 ? 0.0 : t2_raw;
      end_acceleration = a1 + rising_jerk * t2;
      end_velocity = integrateVelocity(v1, a1, rising_jerk, t2);
      plan.distance_m = integratePosition(x1, v1, a1, rising_jerk, t2);
      plan.type = 2;
      plan.times = {t1, t2};
    } else {
      const double t1_raw = -acceleration / rising_jerk;
      if (t1_raw < 0.0) return std::nullopt;
      const double t1 = t1_raw < 0.01 ? 0.0 : t1_raw;
      end_acceleration = acceleration + rising_jerk * t1;
      end_velocity = integrateVelocity(velocity, acceleration, rising_jerk, t1);
      plan.distance_m = integratePosition(
          0.0, velocity, acceleration, rising_jerk, t1);
      plan.type = 3;
      plan.times = {t1};
    }
  }
  if (!std::isfinite(plan.distance_m) ||
      std::fabs(end_velocity - target_velocity) > 0.3 ||
      std::fabs(end_acceleration) > 0.1) return std::nullopt;
  return plan;
}

struct StopState {
  double x = 0.0;
  double v = 0.0;
  double a = 0.0;
};

StopState stopStateAt(
    double velocity, double acceleration, double jerk,
    const StopPlan &plan, double time) {
  StopState state{0.0, velocity, acceleration};
  double elapsed = 0.0;
  for (std::size_t phase = 0; phase < plan.times.size(); ++phase) {
    const double phase_jerk =
        plan.type == 3 ? -jerk :
        phase == 0 ? jerk :
        plan.type == 1 && phase == 1 ? 0.0 : -jerk;
    const double dt = std::min(plan.times[phase],
                               std::max(0.0, time - elapsed));
    state.x = integratePosition(
        state.x, state.v, state.a, phase_jerk, dt);
    state.v = integrateVelocity(state.v, state.a, phase_jerk, dt);
    state.a += phase_jerk * dt;
    elapsed += plan.times[phase];
    if (time <= elapsed) break;
  }
  return state;
}

bool applyStopPlan(Trajectory &output, std::size_t start,
                   double target_velocity, double jerk,
                   double minimum_acceleration, const StopPlan &plan) {
  const double initial_velocity = output[start].longitudinal_velocity_mps;
  const double initial_acceleration = output[start].acceleration_mps2;
  const double duration = std::accumulate(
      plan.times.begin(), plan.times.end(), 0.0);
  std::vector<double> distance_samples;
  std::vector<double> velocity_samples;
  std::vector<double> acceleration_samples;
  for (double time = 0.0; time < duration; time += 0.1) {
    const auto state = stopStateAt(
        initial_velocity, initial_acceleration, jerk, plan, time);
    if (state.v > 0.0 &&
        (distance_samples.empty() || state.x > distance_samples.back())) {
      distance_samples.push_back(state.x);
      velocity_samples.push_back(state.v);
      acceleration_samples.push_back(
          std::max(state.a, minimum_acceleration));
    }
  }
  const auto final = stopStateAt(
      initial_velocity, initial_acceleration, jerk, plan, duration);
  if (final.v > 0.0 &&
      (distance_samples.empty() || final.x > distance_samples.back())) {
    distance_samples.push_back(final.x);
    velocity_samples.push_back(final.v);
    acceleration_samples.push_back(
        std::max(final.a, minimum_acceleration));
  }
  if (std::fabs(final.v - target_velocity) > 0.3 ||
      std::fabs(final.a) > 0.1) return false;
  if (distance_samples.size() < 2) {
    for (std::size_t index = start; index < output.size(); ++index) {
      output[index].longitudinal_velocity_mps =
          static_cast<float>(target_velocity);
      output[index].acceleration_mps2 = 0.0F;
    }
    return true;
  }
  double distance_from_start = 0.0;
  for (std::size_t index = start; index < output.size(); ++index) {
    if (index > start) {
      distance_from_start += distance(output[index - 1], output[index]);
    }
    if (distance_from_start > distance_samples.back()) {
      output[index].longitudinal_velocity_mps =
          static_cast<float>(target_velocity);
      output[index].acceleration_mps2 = 0.0F;
      continue;
    }
    const auto upper = std::upper_bound(
        distance_samples.begin(), distance_samples.end(),
        distance_from_start);
    const std::size_t last = std::min<std::size_t>(
        distance_samples.size() - 1,
        static_cast<std::size_t>(upper - distance_samples.begin()));
    const std::size_t first = last == 0 ? 0 : last - 1;
    const double span = distance_samples[last] - distance_samples[first];
    const double ratio = span > 1.0e-9 ?
        (distance_from_start - distance_samples[first]) / span : 0.0;
    output[index].longitudinal_velocity_mps = static_cast<float>(
        velocity_samples[first] +
        ratio * (velocity_samples[last] - velocity_samples[first]));
    output[index].acceleration_mps2 = static_cast<float>(
        acceleration_samples[first] +
        ratio * (acceleration_samples[last] - acceleration_samples[first]));
  }
  return true;
}

bool applyBackwardFilter(
    Trajectory &output, const std::vector<std::size_t> &start_indices,
    std::size_t target_index, double target_velocity,
    const AnalyticalBackwardParameters &parameters) {
  double selected_jerk = -100.0;
  std::size_t selected_start = 0;
  StopPlan selected_plan;
  std::vector<double> selected_distances;
  for (auto start : start_indices) {
    for (std::size_t index = start; index < target_index; ++index) {
      if (output[index].longitudinal_velocity_mps >= target_velocity) {
        start = index;
        break;
      }
    }
    std::vector<double> distances(output.size(), 0.0);
    for (std::size_t index = target_index; index > start; --index) {
      distances[index - 1] = distances[index] +
          distance(output[index - 1], output[index]);
    }
    for (double jerk = parameters.start_jerk_mps3;
         jerk > parameters.minimum_jerk_mps3 - 0.001;
         jerk += parameters.jerk_step_mps3) {
      const double minimum_acceleration =
          jerk < parameters.minimum_mild_jerk_mps3 ?
          parameters.minimum_acceleration_mps2 :
          parameters.minimum_mild_acceleration_mps2;
      const auto plan = calculateStopPlan(
          output[start].longitudinal_velocity_mps,
          output[start].acceleration_mps2, jerk,
          minimum_acceleration, target_velocity);
      if (!plan || plan->distance_m < 0.0 ||
          plan->distance_m > distances[start]) continue;
      if (jerk >= selected_jerk) {
        selected_jerk = jerk;
        selected_start = start;
        selected_plan = *plan;
        selected_distances = std::move(distances);
      }
      break;
    }
  }
  if (selected_jerk == -100.0) return false;
  std::size_t deceleration_start = selected_start;
  if (std::fabs(selected_jerk - parameters.start_jerk_mps3) < 1.0e-9) {
    for (std::size_t index = target_index; index-- > selected_start;) {
      const double minimum_acceleration =
          selected_jerk < parameters.minimum_mild_jerk_mps3 ?
          parameters.minimum_acceleration_mps2 :
          parameters.minimum_mild_acceleration_mps2;
      const auto plan = calculateStopPlan(
          output[index].longitudinal_velocity_mps,
          output[index].acceleration_mps2, selected_jerk,
          minimum_acceleration, target_velocity);
      if (plan && plan->distance_m >= 0.0 &&
          plan->distance_m <= selected_distances[index]) {
        deceleration_start = index;
        selected_plan = *plan;
        break;
      }
    }
  }
  const double minimum_acceleration =
      selected_jerk < parameters.minimum_mild_jerk_mps3 ?
      parameters.minimum_acceleration_mps2 :
      parameters.minimum_mild_acceleration_mps2;
  return applyStopPlan(output, deceleration_start, target_velocity,
                       selected_jerk, minimum_acceleration, selected_plan);
}

}  // 익명 네임스페이스

Trajectory smoothAnalyticalVelocity(
    const Trajectory &reference, double initial_velocity_mps,
    double initial_acceleration_mps2,
    const AnalyticalForwardParameters &forward,
    const AnalyticalBackwardParameters &backward) {
  if (reference.empty() || backward.start_jerk_mps3 >= 0.0 ||
      backward.minimum_jerk_mps3 >= 0.0 ||
      backward.jerk_step_mps3 >= 0.0) {
    throw std::invalid_argument("Analytical 감속 입력이 유효하지 않습니다");
  }
  if (reference.size() == 1) {
    auto result = reference;
    result.front().longitudinal_velocity_mps =
        static_cast<float>(initial_velocity_mps);
    result.front().acceleration_mps2 =
        static_cast<float>(initial_acceleration_mps2);
    return result;
  }
  std::vector<std::pair<std::size_t, double>> candidates;
  for (std::size_t index = 1; index + 1 < reference.size(); ++index) {
    const double before = reference[index].longitudinal_velocity_mps -
                          reference[index - 1].longitudinal_velocity_mps;
    const double after = reference[index + 1].longitudinal_velocity_mps -
                         reference[index].longitudinal_velocity_mps;
    if (before < -0.00001 && after > -0.00001) {
      candidates.emplace_back(
          index, reference[index].longitudinal_velocity_mps);
    }
  }
  if (reference.back().longitudinal_velocity_mps -
          reference[reference.size() - 2].longitudinal_velocity_mps <
      -0.00001) {
    candidates.emplace_back(
        reference.size() - 1,
        reference.back().longitudinal_velocity_mps);
  }
  std::vector<std::pair<std::size_t, double>> targets;
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    if (index + 1 < candidates.size() &&
        candidates[index + 1].first - candidates[index].first < 10 &&
        candidates[index + 1].second < candidates[index].second) {
      continue;
    }
    targets.push_back(candidates[index]);
  }
  Trajectory filtered = reference;
  for (std::size_t index = 0; index < targets.size(); ++index) {
    const std::size_t forward_start = index == 0 ? 0 :
        targets[index - 1].first;
    const double velocity = index == 0 ? initial_velocity_mps :
        filtered[forward_start].longitudinal_velocity_mps;
    const double acceleration = index == 0 ? initial_acceleration_mps2 :
        filtered[forward_start].acceleration_mps2;
    const auto advanced = applyAnalyticalForwardJerkFilter(
        reference, forward_start, velocity, acceleration, forward);
    std::copy(advanced.begin() + forward_start, advanced.end(),
              filtered.begin() + forward_start);
    std::size_t backward_start = 0;
    for (std::size_t previous = index; previous > 0; --previous) {
      if (targets[previous - 1].second < targets[previous].second) {
        backward_start = targets[previous - 1].first;
        break;
      }
    }
    std::vector<std::size_t> starts{backward_start};
    if (backward_start != forward_start) starts.push_back(forward_start);
    if (!applyBackwardFilter(filtered, starts,
                             targets[index].first,
                             targets[index].second, backward)) {
      if (std::fabs(targets[index].second) < 0.001) {
        for (std::size_t point = backward_start;
             point < filtered.size(); ++point) {
          filtered[point].longitudinal_velocity_mps = 0.0F;
          filtered[point].acceleration_mps2 = 0.0F;
        }
        return filtered;
      }
      auto capped = reference;
      for (std::size_t point = backward_start;
           point <= targets[index].first; ++point) {
        capped[point].longitudinal_velocity_mps = static_cast<float>(
            std::min(static_cast<double>(
                         capped[point].longitudinal_velocity_mps),
                     targets[index].second));
      }
      const auto fallback = applyAnalyticalForwardJerkFilter(
          capped, backward_start,
          backward_start == 0 ? initial_velocity_mps :
              filtered[backward_start].longitudinal_velocity_mps,
          backward_start == 0 ? initial_acceleration_mps2 :
              filtered[backward_start].acceleration_mps2,
          forward);
      std::copy(fallback.begin() + backward_start, fallback.end(),
                filtered.begin() + backward_start);
    }
  }
  const std::size_t start = targets.empty() ? 0 : targets.back().first;
  const auto final = applyAnalyticalForwardJerkFilter(
      reference, start,
      targets.empty() ? initial_velocity_mps :
          filtered[start].longitudinal_velocity_mps,
      targets.empty() ? initial_acceleration_mps2 :
          filtered[start].acceleration_mps2,
      forward);
  std::copy(final.begin() + start, final.end(), filtered.begin() + start);
  return filtered;
}

}  // planning 네임스페이스
