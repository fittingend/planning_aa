// SPDX-License-Identifier: Apache-2.0
// 기록된 예측 객체의 순항 앞단 투영·넓은 횡거리 필터를 계산합니다.

#include "planning/obstacle_cruise_candidate.hpp"
#include "planning/obstacle_cruise_collision.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

int main(int argc, char **argv) {
  if (argc != 3 && argc != 4 && argc != 5 && argc != 6) return 2;
  if (argc >= 5 && std::string(argv[3]) != "--write-ids") return 2;
  if (argc == 6 && std::string(argv[5]) != "--sample-vehicle") return 2;
  const bool resample = argc != 4;
  const bool sample_vehicle = argc == 6;
  const double front_m = sample_vehicle ? 3.79 : 3.74;
  const double rear_m = sample_vehicle ? 1.10 : 1.03;
  const double width_m = sample_vehicle ? 1.896 : 1.83;
  std::ifstream input(argv[1]);
  std::ofstream output(argv[2]);
  std::ofstream id_output;
  if (argc >= 5) id_output.open(argv[4]);
  if (!input || !output || (argc >= 5 && !id_output)) return 2;
  std::size_t frames = 0;
  input >> frames;
  planning::CruiseInsideHysteresis baseline_history;
  planning::CruiseInsideHysteresis strict_history(3.5, 3.0);
  for (std::size_t frame = 0; frame < frames; ++frame) {
    std::size_t point_count = 0, object_count = 0, ego_index = 0;
    double ego_speed = 0.0, frame_time_s = 0.0, ego_acceleration = 0.0;
    planning::CruisePredictedPose ego_pose;
    input >> point_count >> object_count >> ego_index >> ego_speed >>
        ego_pose.x >> ego_pose.y >> ego_pose.yaw_rad >> frame_time_s >>
        ego_acceleration;
    (void)frame_time_s;
    (void)ego_acceleration;
    planning::Trajectory path(point_count);
    for (auto &point : path)
      input >> point.x >> point.y >> point.orientation_x >>
          point.orientation_y >> point.orientation_z >> point.orientation_w >>
          point.longitudinal_velocity_mps;
    std::size_t ego_segment = ego_index;
    if (ego_index + 1 == path.size())
      ego_segment = ego_index - 1;
    else if (ego_index > 0) {
      const double projection =
          (ego_pose.x - path[ego_index].x) *
              (path[ego_index + 1].x - path[ego_index].x) +
          (ego_pose.y - path[ego_index].y) *
              (path[ego_index + 1].y - path[ego_index].y);
      if (projection <= 0.0) ego_segment = ego_index - 1;
    }
    std::size_t cruise_types = 0, front = 0, rough = 0;
    std::size_t forward = 0, collision = 0;
    std::size_t accepted = 0, inside = 0, outside = 0;
    std::size_t strict_accepted = 0, strict_inside = 0, strict_outside = 0;
    std::vector<std::string> accepted_ids, strict_accepted_ids;
    std::vector<planning::CruiseYieldObject> yield_objects;
    for (std::size_t i = 0; i < object_count; ++i) {
      planning::CruiseObjectKinematics object;
      std::string object_id;
      double radius = 0.0, elapsed = 0.0;
      int shape_type = 0, label = 0;
      double length = 0.0, width = 0.0;
      std::size_t predicted_count = 0, footprint_count = 0;
      input >> object_id >> object.x >> object.y >> object.yaw_rad >>
          object.local_velocity_x >> object.local_velocity_y >> radius >>
          elapsed >> predicted_count >> shape_type >> length >> width >>
          footprint_count >> label;
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
      if (label == 7) continue;
      ++cruise_types;
      const auto current = planning::currentCruiseObjectPose(
          {object.x, object.y, object.yaw_rad}, predictions, elapsed);
      object.x = current.x;
      object.y = current.y;
      object.yaw_rad = current.yaw_rad;
      const auto projected = planning::projectCruiseObjectToTrajectory(
          path, ego_index, object);
      if (!projected) continue;
      ++front;
      if (planning::passesObstacleCruiseRoughLateralFilter(
              *projected, radius, width_m)) {
        ++rough;
        const double heading_difference = std::abs(std::remainder(
            projected->heading_difference_rad, 2.0 * std::acos(-1.0)));
        const bool crossing = heading_difference > 0.523599 &&
            std::acos(-1.0) - heading_difference > 0.523599;
        yield_objects.push_back({
            object_id, label >= 1 && label <= 5, crossing,
            object.x, object.y, object.local_velocity_x,
            object.local_velocity_y,
            projected->longitudinal_velocity_mps,
            projected->signed_lateral_offset_m,
            projected->distance_from_ego_m,
            projected->heading_difference_rad});
        if (projected->longitudinal_velocity_mps >= 0.0) {
          ++forward;
          if (!predictions.empty()) {
            if (ego_segment + 1 >= path.size()) continue;
            const planning::Trajectory trimmed = resample ?
                planning::resampleCruiseCollisionTrajectory(
                    path, ego_segment, 2.0, 6.0) :
                planning::Trajectory(path.begin() + ego_segment, path.end());
            const double precise = planning::cruiseObjectLateralDistance(
                trimmed, current, shape, front_m, rear_m, width_m,
                resample ? std::optional<planning::CruisePredictedPose>(ego_pose)
                         : std::nullopt);
            std::vector<const planning::CruisePredictedPath *> ranked;
            for (const auto &prediction : predictions)
              ranked.push_back(&prediction);
            std::sort(ranked.begin(), ranked.end(),
                      [](const auto *a, const auto *b) {
                        return a->confidence > b->confidence;
                      });
            std::vector<planning::CruiseCollisionPoint> points;
            const std::size_t path_limit =
                precise < 1e-6 ? 1 : std::min<std::size_t>(3, ranked.size());
            for (std::size_t index = 0; index < path_limit; ++index) {
              const auto samples = planning::resampleCruisePredictedPath(
                  *ranked[index], 0.1, 10.0);
              points = planning::findCruiseCollisionPoints(
                  trimmed, samples, shape, front_m, rear_m, width_m,
                  elapsed, 10.0, radius + 2.0 +
                      std::hypot(front_m + rear_m,
                                 width_m * 0.5 + 1.0), true,
                  resample ? std::optional<planning::CruisePredictedPose>(ego_pose)
                           : std::nullopt);
              if (!points.empty()) break;
            }
            if (!points.empty()) ++collision;
            std::size_t object_index = 0;
            double closest = std::numeric_limits<double>::infinity();
            for (std::size_t index = 0; index < trimmed.size(); ++index) {
              const double distance = std::hypot(
                  trimmed[index].x - current.x,
                  trimmed[index].y - current.y);
              if (distance < closest) {
                closest = distance;
                object_index = index;
              }
            }
            double arc_to_collision = 0.0;
            if (!points.empty()) {
              const auto first_index = points.front().trajectory_index;
              const auto lo = std::min(object_index, first_index);
              const auto hi = std::max(object_index, first_index);
              for (auto index = lo; index < hi; ++index)
                arc_to_collision += std::hypot(
                    trimmed[index + 1].x - trimmed[index].x,
                    trimmed[index + 1].y - trimmed[index].y);
              if (first_index < object_index) arc_to_collision = -arc_to_collision;
            }
            planning::CruiseCandidateInput candidate;
            candidate.supported_class = true;
            candidate.outside_class = label >= 1 && label <= 5;
            candidate.ego_speed_mps = ego_speed;
            candidate.longitudinal_speed_mps =
                projected->longitudinal_velocity_mps;
            candidate.approach_speed_mps =
                projected->approach_velocity_mps;
            candidate.lateral_distance_m = precise;
            candidate.heading_difference_rad =
                projected->heading_difference_rad;
            candidate.object_speed_mps = std::hypot(
                object.local_velocity_x, object.local_velocity_y);
            candidate.has_collision_points = !points.empty();
            candidate.collision_overlap_s = points.empty() ? 0.0 :
                points.back().time_from_object_stamp_s -
                points.front().time_from_object_stamp_s;
            candidate.first_collision_in_front =
                arc_to_collision > -radius;
            if (planning::classifyCruiseCandidate(candidate) ==
                    planning::CruiseCandidateRejection::None &&
                (precise >= 1e-6 ||
                 baseline_history.permits(
                     object_id, projected->longitudinal_velocity_mps))) {
              ++accepted;
              accepted_ids.push_back(object_id);
              if (precise < 1e-6) ++inside;
              else ++outside;
            }
            // 별도 배포 설정의 임계값을 적용해 기록의 설정 민감도를 확인합니다.
            planning::CruiseCandidateParameters strict_parameters;
            strict_parameters.crossing_angle_threshold_rad = 1.22;
            strict_parameters.minimum_outside_speed_mps = 3.5;
            strict_parameters.minimum_outside_overlap_s = 2.0;
            if (planning::classifyCruiseCandidate(
                    candidate, strict_parameters) ==
                    planning::CruiseCandidateRejection::None &&
                (precise >= 1e-6 ||
                 strict_history.permits(
                     object_id, projected->longitudinal_velocity_mps))) {
              ++strict_accepted;
              strict_accepted_ids.push_back(object_id);
              if (precise < 1e-6) ++strict_inside;
              else ++strict_outside;
            }
          }
        }
      }
    }
    if (!input || point_count < 2)
      throw std::runtime_error("순항 객체 재생 입력이 잘못되었습니다");
    const auto yield_pairs = planning::findCruiseYieldPairs(yield_objects);
    const auto baseline_object_ids = accepted_ids;
    const auto strict_object_ids = strict_accepted_ids;
    auto strict_yield_objects = yield_objects;
    for (auto &candidate : strict_yield_objects) {
      const double angle = std::abs(std::remainder(
          candidate.heading_difference_rad, 2.0 * std::acos(-1.0)));
      candidate.crossing_trajectory =
          angle > 1.22 && std::acos(-1.0) - angle > 1.22;
    }
    planning::CruiseYieldParameters strict_yield_parameters;
    strict_yield_parameters.outside_minimum_speed_mps = 3.5;
    const auto strict_yield_pairs = planning::findCruiseYieldPairs(
        strict_yield_objects, strict_yield_parameters);
    for (const auto &pair : yield_pairs)
      accepted_ids.push_back(yield_objects[pair.moving_index].id);
    for (const auto &pair : strict_yield_pairs)
      strict_accepted_ids.push_back(
          strict_yield_objects[pair.moving_index].id);
    baseline_history.replacePreviousCruiseIds(accepted_ids);
    strict_history.replacePreviousCruiseIds(strict_accepted_ids);
    if (id_output) {
      id_output << baseline_object_ids.size();
      for (const auto &id : baseline_object_ids) id_output << ' ' << id;
      id_output << ' ' << strict_object_ids.size();
      for (const auto &id : strict_object_ids) id_output << ' ' << id;
      id_output << '\n';
    }
    output << cruise_types << ' ' << front << ' ' << rough << ' '
           << forward << ' ' << collision << ' ' << accepted << ' '
           << inside << ' ' << outside << ' ' << strict_accepted << ' '
           << strict_inside << ' ' << strict_outside << ' '
           << yield_pairs.size() << '\n';
  }
}
