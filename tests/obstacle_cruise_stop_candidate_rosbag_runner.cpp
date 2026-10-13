// SPDX-License-Identifier: Apache-2.0
// 실제 예측 객체의 현재 외형과 순항 정지·감속 후보 앞단을 재생합니다.

#include "planning/obstacle_cruise_candidate.hpp"
#include "planning/obstacle_cruise_collision.hpp"
#include "planning/obstacle_cruise_stop_core.hpp"
#include "planning/obstacle_cruise_slowdown_core.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace {
double arcAt(const planning::Trajectory &path, double x, double y) {
  double best = std::numeric_limits<double>::infinity();
  double result = 0.0, accumulated = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i + 1].x - path[i].x;
    const double dy = path[i + 1].y - path[i].y;
    const double length = std::hypot(dx, dy);
    if (length < 1e-9) continue;
    const double ratio = std::clamp(
        ((x - path[i].x) * dx + (y - path[i].y) * dy) /
            (length * length), 0.0, 1.0);
    const double distance = std::hypot(
        x - path[i].x - ratio * dx, y - path[i].y - ratio * dy);
    if (distance < best) {
      best = distance;
      result = accumulated + ratio * length;
    }
    accumulated += length;
  }
  return result;
}

planning::CruisePredictedPose futurePose(
    const std::vector<planning::CruiseTimedPose> &prediction,
    double time_s) {
  if (prediction.empty()) return {};
  if (time_s <= prediction.front().time_from_object_stamp_s)
    return prediction.front().pose;
  for (std::size_t i = 1; i < prediction.size(); ++i) {
    if (time_s > prediction[i].time_from_object_stamp_s) continue;
    const double duration = prediction[i].time_from_object_stamp_s -
                            prediction[i - 1].time_from_object_stamp_s;
    const double ratio = duration < 1e-9 ? 0.0 :
        (time_s - prediction[i - 1].time_from_object_stamp_s) / duration;
    const auto &a = prediction[i - 1].pose;
    const auto &b = prediction[i].pose;
    return {a.x + ratio * (b.x - a.x),
            a.y + ratio * (b.y - a.y),
            a.yaw_rad + ratio * std::remainder(
                b.yaw_rad - a.yaw_rad, 2.0 * std::acos(-1.0))};
  }
  return prediction.back().pose;
}

bool selectSlowdown(
    const std::string &id, const double lateral_distance,
    const bool has_collision,
    const std::unordered_set<std::string> &previous,
    std::unordered_map<std::string, int> &counter,
    std::unordered_set<std::string> &current) {
  const bool was_selected = previous.count(id) != 0;
  const bool close = was_selected ? lateral_distance <= 1.2 :
                                    lateral_distance < 1.0;
  bool selected = was_selected;
  if (was_selected && !close) {
    const int count = counter[id] = std::min(-1, counter[id] - 1);
    if (count <= -5) {
      counter[id] = 0;
      selected = false;
    }
  } else if (!was_selected && close) {
    const int count = counter[id] = std::max(1, counter[id] + 1);
    if (count >= 5) {
      counter[id] = 0;
      selected = true;
    }
  }
  if (selected && has_collision) current.insert(id);
  return selected && has_collision;
}
}  // 익명 네임스페이스

int main(int argc, char **argv) {
  if (argc != 3 && argc != 4 && argc != 5 && argc != 7 && argc != 8) return 2;
  const bool sample_shape = argc == 7 || argc == 8;
  const double front_m = sample_shape ? std::stod(argv[4]) : 3.74;
  const double rear_m = sample_shape ? std::stod(argv[5]) : 1.03;
  const double width_m = sample_shape ? std::stod(argv[6]) : 1.83;
  std::ifstream input(argv[1]);
  std::ofstream output(argv[2]);
  std::ifstream cruise_ids;
  std::ifstream oracle_activity;
  if (argc >= 4) cruise_ids.open(argv[3]);
  if (argc == 5 || argc == 8) oracle_activity.open(argv[argc - 1]);
  if (!input || !output || (argc >= 4 && !cruise_ids) ||
      ((argc == 5 || argc == 8) && !oracle_activity)) return 2;
  std::size_t frame_count = 0;
  input >> frame_count;
  std::unordered_map<std::string, int> slowdown_counter;
  std::unordered_set<std::string> previous_slowdown;
  std::unordered_map<std::string, int> strict_slowdown_counter;
  std::unordered_set<std::string> strict_previous_slowdown;
  planning::CruiseStopConsistency strict_stop_consistency;
  planning::ObstacleCruiseStopParameters stop_parameters;
  stop_parameters.driving_acceleration_limit = 1.0;
  stop_parameters.braking_acceleration_limit = -2.5;
  stop_parameters.suppress_sudden_stop = true;
  planning::ObstacleCruiseStopCore stop_core(stop_parameters);
  planning::ObstacleCruiseSlowdownCore slowdown_core;
  planning::ObstacleCruiseSlowdownCore oracle_slowdown_core;
  for (std::size_t frame = 0; frame < frame_count; ++frame) {
    std::unordered_set<std::string> baseline_cruise, strict_cruise;
    if (cruise_ids) {
      std::size_t count = 0;
      std::string id;
      cruise_ids >> count;
      for (std::size_t i = 0; i < count; ++i) {
        cruise_ids >> id;
        baseline_cruise.insert(id);
      }
      cruise_ids >> count;
      for (std::size_t i = 0; i < count; ++i) {
        cruise_ids >> id;
        strict_cruise.insert(id);
      }
      if (!cruise_ids)
        throw std::runtime_error("순항 후보 객체 ID 입력이 잘못되었습니다");
    }
    std::size_t point_count = 0, object_count = 0, ego_index = 0;
    double ego_speed = 0.0, frame_time_s = 0.0, ego_acceleration = 0.0;
    planning::CruisePredictedPose ego_pose;
    input >> point_count >> object_count >> ego_index >> ego_speed >>
        ego_pose.x >> ego_pose.y >> ego_pose.yaw_rad >> frame_time_s >>
        ego_acceleration;
    planning::Trajectory path(point_count);
    for (auto &point : path)
      input >> point.x >> point.y >> point.orientation_x >>
          point.orientation_y >> point.orientation_z >>
          point.orientation_w >> point.longitudinal_velocity_mps;
    if (!input || point_count < 2 || ego_index >= point_count)
      throw std::runtime_error("순항 정지 후보 경로가 유효하지 않습니다");
    std::size_t ego_segment =
        ego_index + 1 == point_count ? ego_index - 1 : ego_index;
    if (ego_index > 0 && ego_index + 1 < point_count) {
      const double projection =
          (ego_pose.x - path[ego_index].x) *
              (path[ego_index + 1].x - path[ego_index].x) +
          (ego_pose.y - path[ego_index].y) *
              (path[ego_index + 1].y - path[ego_index].y);
      if (projection <= 0.0) ego_segment = ego_index - 1;
    }
    const auto trimmed = planning::resampleCruiseCollisionTrajectory(
        path, ego_segment, 2.0, 6.0);
    std::size_t front = 0, rough = 0, current_collision = 0;
    std::size_t candidate = 0, pedestrian_candidate = 0;
    std::size_t baseline_priority_stop = 0, strict_priority_stop = 0;
    std::size_t slowdown_geometry = 0, slowdown_candidate = 0;
    std::size_t transient_rejected = 0;
    std::size_t strict_priority_slowdown = 0;
    std::size_t strict_current_priority_slowdown = 0;
    std::unordered_set<std::string> seen, current_slowdown;
    std::unordered_set<std::string> strict_seen, strict_current_slowdown;
    std::vector<planning::ObstacleCruiseSlowdownCandidate>
        current_priority_slowdowns;
    std::vector<planning::CruiseStopObservation> observations;
    std::vector<planning::CruiseStopTrackedCandidate> strict_stop_objects;
    double object_stamp_s = frame_time_s;
    double closest_distance = std::numeric_limits<double>::infinity();
    double strict_closest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < object_count; ++i) {
      planning::CruiseObjectKinematics object;
      std::string id;
      double radius = 0.0, elapsed = 0.0;
      int shape_type = 0, label = 0;
      double length = 0.0, width = 0.0;
      std::size_t predicted_count = 0, footprint_count = 0;
      input >> id >> object.x >> object.y >> object.yaw_rad >>
          object.local_velocity_x >> object.local_velocity_y >> radius >>
          elapsed >> predicted_count >> shape_type >> length >> width >>
          footprint_count >> label;
      if (i == 0) object_stamp_s = frame_time_s - elapsed;
      observations.push_back({id, object.local_velocity_x});
      planning::CruiseObjectShape shape{shape_type, length, width, {}};
      shape.footprint.resize(footprint_count);
      for (auto &vertex : shape.footprint)
        input >> vertex.x >> vertex.y;
      std::vector<planning::CruisePredictedPath> predictions(predicted_count);
      for (auto &prediction : predictions) {
        std::size_t pose_count = 0;
        input >> prediction.time_step_s >> prediction.confidence >> pose_count;
        prediction.poses.resize(pose_count);
        for (auto &pose : prediction.poses)
          input >> pose.x >> pose.y >> pose.yaw_rad;
      }
      const auto current = planning::currentCruiseObjectPose(
          {object.x, object.y, object.yaw_rad}, predictions, elapsed);
      object.x = current.x; object.y = current.y;
      object.yaw_rad = current.yaw_rad;
      const auto projection = planning::projectCruiseObjectToTrajectory(
          path, ego_index, object);
      if (!projection) continue;
      seen.insert(id);
      ++front;
      if (!planning::passesObstacleCruiseRoughLateralFilter(
              *projection, radius, width_m)) continue;
      ++rough;
      const double precise = planning::cruiseObjectLateralDistance(
          trimmed, current, shape, front_m, rear_m, width_m, ego_pose);
      std::optional<planning::CruiseSlowdownCollisionInterval>
          slowdown_interval;
      bool raw_slowdown_selected = false;
      if (label >= 1 && label <= 7) {
        slowdown_interval = planning::findCruiseSlowdownCollisionInterval(
            trimmed, current, shape, front_m, rear_m, width_m,
            1.3, ego_pose);
        if (slowdown_interval) ++slowdown_geometry;
        const bool was_slowdown = previous_slowdown.count(id) != 0;
        const bool close = was_slowdown ? precise <= 1.2 : precise < 1.0;
        bool selected = was_slowdown;
        if (was_slowdown && !close) {
          const int count = slowdown_counter[id] =
              std::min(-1, slowdown_counter[id] - 1);
          if (count <= -5) {
            slowdown_counter[id] = 0;
            selected = false;
          }
        } else if (!was_slowdown && close) {
          const int count = slowdown_counter[id] =
              std::max(1, slowdown_counter[id] + 1);
          if (count >= 5) {
            slowdown_counter[id] = 0;
            selected = true;
          }
        }
        if (selected && slowdown_interval) {
          ++slowdown_candidate;
          current_slowdown.insert(id);
          raw_slowdown_selected = true;
        }
      }
      const auto select_strict_slowdown = [&]() {
        if (label < 1 || label > 7 || strict_cruise.count(id) != 0)
          return;
        strict_seen.insert(id);
        if (selectSlowdown(id, precise, slowdown_interval.has_value(),
                           strict_previous_slowdown,
                           strict_slowdown_counter,
                           strict_current_slowdown))
          ++strict_priority_slowdown;
      };
      const auto record_current_priority_slowdown = [&]() {
        if (!raw_slowdown_selected || strict_cruise.count(id) != 0 ||
            !slowdown_interval) return;
        ++strict_current_priority_slowdown;
        current_priority_slowdowns.push_back({
            id,
            arcAt(path, slowdown_interval->front_point.x,
                  slowdown_interval->front_point.y),
            arcAt(path, slowdown_interval->back_point.x,
                  slowdown_interval->back_point.y),
            precise,
            projection->longitudinal_velocity_mps,
            projection->approach_velocity_mps});
      };
      if (precise > 0.3) {
        record_current_priority_slowdown();
        select_strict_slowdown();
        continue;
      }
      const auto collision = planning::findCruiseCurrentStopCollision(
          trimmed, current, shape, front_m, rear_m, width_m,
          0.3, true, ego_pose);
      if (collision) ++current_collision;
      const double time_to_reach = collision ?
          planning::cruiseStopTimeToReachCollision(
              arcAt(trimmed, collision->x, collision->y) -
                  arcAt(trimmed, ego_pose.x, ego_pose.y),
              3.0 + front_m, ego_speed) : 0.0;
      bool checked_future = false, future_collision = false;
      if (collision && time_to_reach > 1.0 && !predictions.empty()) {
        const auto best = std::max_element(
            predictions.begin(), predictions.end(),
            [](const auto &a, const auto &b) {
              return a.confidence < b.confidence;
            });
        const auto samples = planning::resampleCruisePredictedPath(
            *best, 0.1, 10.0);
        if (!samples.empty()) {
          checked_future = true;
          future_collision = planning::findCruiseCurrentStopCollision(
              trimmed, futurePose(samples, time_to_reach - 1.0),
              shape, front_m, rear_m, width_m, 0.3, true,
              ego_pose).has_value();
        }
      }
      planning::CruiseStopBehaviorInput decision;
      decision.supported_class = label >= 0 && label <= 7;
      decision.inside_class = decision.supported_class;
      decision.precise_lateral_distance_m = precise;
      decision.approach_velocity_mps = projection->approach_velocity_mps;
      decision.has_collision_point = collision.has_value();
      decision.time_to_reach_stop_point_s = time_to_reach;
      decision.checked_future_collision = checked_future;
      decision.has_future_collision = future_collision;
      const auto stop_rejection =
          planning::classifyCruiseStopBehavior(decision);
      if (stop_rejection != planning::CruiseStopRejection::None) {
        transient_rejected +=
            stop_rejection == planning::CruiseStopRejection::TransientInsideObject;
        record_current_priority_slowdown();
        select_strict_slowdown();
        continue;
      }
      ++candidate;
      if (baseline_cruise.count(id) == 0) ++baseline_priority_stop;
      if (strict_cruise.count(id) == 0) {
        ++strict_priority_stop;
        strict_closest_distance = std::min(
            strict_closest_distance, collision->distance_from_start_m);
        strict_stop_objects.push_back({
            id, label, frame_time_s - elapsed,
            collision->distance_from_start_m});
      }
      if (label == 7) ++pedestrian_candidate;
      closest_distance = std::min(
          closest_distance, collision->distance_from_start_m);
    }
    if (!input) throw std::runtime_error("순항 정지 후보 객체 입력이 잘못되었습니다");
    const auto held_stop_objects = strict_stop_consistency.update(
        object_stamp_s, observations, strict_stop_objects);
    double held_closest_distance = std::numeric_limits<double>::infinity();
    for (const auto &item : held_stop_objects)
      held_closest_distance = std::min(
          held_closest_distance, item.collision_distance_from_start_m);
    double start_arc = 0.0, full_length = 0.0;
    std::optional<double> behavior_stop_arc;
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
      if (i == ego_segment) start_arc = full_length;
      full_length += std::hypot(path[i + 1].x - path[i].x,
                                path[i + 1].y - path[i].y);
      if (!behavior_stop_arc && i + 1 > ego_segment &&
          path[i + 1].longitudinal_velocity_mps <= 0.0)
        behavior_stop_arc = full_length;
    }
    const double selected_margin = held_stop_objects.empty() ? -1.0 :
        planning::chooseCruiseStopMargin(
            start_arc + held_closest_distance, full_length,
            behavior_stop_arc, 6.0, 3.0, 3.0);
    const double selected_stop_arc = held_stop_objects.empty() ? -1.0 :
        std::max(0.0, start_arc + held_closest_distance - selected_margin);
    std::vector<planning::ObstacleCruiseStopCandidate> stop_candidates;
    for (const auto &item : held_stop_objects) {
      const double collision_arc =
          start_arc + item.collision_distance_from_start_m;
      planning::ObstacleCruiseStopCandidate stop_candidate;
      stop_candidate.classification = item.classification;
      stop_candidate.collision_distance_from_start = collision_arc;
      stop_candidate.desired_margin = planning::chooseCruiseStopMargin(
          collision_arc, full_length, behavior_stop_arc, 6.0, 3.0, 3.0);
      stop_candidate.type_specific_parameters = item.classification == 0;
      stop_candidate.sudden_acceleration_limit = -1.2;
      stop_candidate.sudden_object_acceleration_threshold = -2.5;
      stop_candidates.push_back(stop_candidate);
    }
    const auto stop_plan = stop_core.plan(
        path, arcAt(path, ego_pose.x, ego_pose.y), ego_speed,
        stop_candidates);
    for (auto &item : current_priority_slowdowns) {
      item.front_collision_arc += start_arc;
      item.back_collision_arc += start_arc;
    }
    const auto slowdown_plan = slowdown_core.plan(
        path, {ego_pose.x, ego_pose.y}, ego_speed, ego_acceleration,
        current_priority_slowdowns);
    planning::ObstacleCruiseSlowdownCore stateless_slowdown_core;
    const auto stateless_slowdown_plan = stateless_slowdown_core.plan(
        path, {ego_pose.x, ego_pose.y}, ego_speed, ego_acceleration,
        current_priority_slowdowns);
    int original_slowdown_active = 0;
    if (oracle_activity) oracle_activity >> original_slowdown_active;
    if (oracle_activity.is_open() && !oracle_activity)
      throw std::runtime_error("원본 감속 활성 입력이 부족합니다");
    const auto oracle_slowdown_plan = oracle_slowdown_core.plan(
        path, {ego_pose.x, ego_pose.y}, ego_speed, ego_acceleration,
        original_slowdown_active ? current_priority_slowdowns :
            std::vector<planning::ObstacleCruiseSlowdownCandidate>{});
    previous_slowdown = std::move(current_slowdown);
    strict_previous_slowdown = std::move(strict_current_slowdown);
    for (auto it = slowdown_counter.begin(); it != slowdown_counter.end();) {
      if (seen.count(it->first) == 0) it = slowdown_counter.erase(it);
      else ++it;
    }
    for (auto it = strict_slowdown_counter.begin();
         it != strict_slowdown_counter.end();) {
      if (strict_seen.count(it->first) == 0)
        it = strict_slowdown_counter.erase(it);
      else ++it;
    }
    output << front << ' ' << rough << ' ' << current_collision << ' '
           << candidate << ' ' << pedestrian_candidate << ' '
           << (std::isfinite(closest_distance) ? closest_distance : -1.0)
           << ' ' << slowdown_geometry << ' ' << slowdown_candidate
           << ' ' << baseline_priority_stop << ' ' << strict_priority_stop
           << ' ' << (std::isfinite(strict_closest_distance) ?
                          strict_closest_distance : -1.0)
           << ' ' << strict_priority_slowdown
           << ' ' << strict_current_priority_slowdown
           << ' ' << held_stop_objects.size()
           << ' ' << (std::isfinite(held_closest_distance) ?
                          held_closest_distance : -1.0)
           << ' ' << selected_margin
           << ' ' << selected_stop_arc
           << ' ' << slowdown_plan.intervals.slowdowns.size()
           << ' ' << (slowdown_plan.intervals.slowdowns.empty() ? -1.0 :
                          slowdown_plan.intervals.slowdowns.front().velocity)
           << ' ' << (current_priority_slowdowns.empty() ? -1.0 :
                          current_priority_slowdowns.front().lateral_distance)
           << ' ' << (stateless_slowdown_plan.intervals.slowdowns.empty() ?
                          -1.0 :
                          stateless_slowdown_plan.intervals.slowdowns.front().velocity)
           << ' ' << stop_plan.stop_distance_from_start.value_or(-1.0)
           << ' ' << (slowdown_plan.intervals.slowdowns.empty() ? -1.0 :
                         arcAt(path,
                             slowdown_plan.intervals.slowdowns.front().from.x,
                             slowdown_plan.intervals.slowdowns.front().from.y))
           << ' ' << (slowdown_plan.intervals.slowdowns.empty() ? -1.0 :
                         arcAt(path,
                             slowdown_plan.intervals.slowdowns.front().to.x,
                             slowdown_plan.intervals.slowdowns.front().to.y))
           << ' ' << oracle_slowdown_plan.intervals.slowdowns.size()
           << ' ' << (oracle_slowdown_plan.intervals.slowdowns.empty() ? -1.0 :
                         oracle_slowdown_plan.intervals.slowdowns.front().velocity)
           << ' ' << transient_rejected
           << '\n';
  }
}
