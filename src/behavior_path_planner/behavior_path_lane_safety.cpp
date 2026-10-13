// SPDX-License-Identifier: Apache-2.0
// 차로 다각형 합집합에 차량 전체 외곽이 들어가는지 검사합니다.

#include "planning/behavior_path_lane_safety.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace planning {
namespace {
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
using MultiPolygon = bg::model::multi_polygon<Polygon>;

MultiPolygon fuseLanes(
    const std::vector<std::vector<BehaviorPathPoint2d>> &lanes) {
  MultiPolygon fused;
  for (const auto &lane : lanes) {
    if (lane.size() < 3) continue;
    Polygon polygon;
    for (const auto &point : lane) {
      polygon.outer().emplace_back(point.x, point.y);
    }
    bg::correct(polygon);
    if (bg::area(polygon) <= 0.0) continue;
    if (fused.empty()) {
      fused.push_back(std::move(polygon));
    } else {
      MultiPolygon updated;
      bg::union_(fused, polygon, updated);
      fused = std::move(updated);
    }
  }
  return fused;
}

bool footprintInside(
    double x, double y, double yaw,
    const MultiPolygon &lanes,
    const BehaviorPathVehicleFootprint &vehicle) {
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  Polygon footprint;
  for (const auto &[along, across] : {
           std::pair{-vehicle.back_m, -vehicle.half_width_m},
           std::pair{vehicle.front_m, -vehicle.half_width_m},
           std::pair{vehicle.front_m, vehicle.half_width_m},
           std::pair{-vehicle.back_m, vehicle.half_width_m}}) {
    footprint.outer().emplace_back(
        x + along * cosine - across * sine,
        y + along * sine + across * cosine);
  }
  bg::correct(footprint);
  return bg::within(footprint, lanes);
}

double yaw(const TrajectoryPoint &point) {
  const double x = point.orientation_x;
  const double y = point.orientation_y;
  const double z = point.orientation_z;
  const double w = point.orientation_w;
  return std::atan2(2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z));
}

}  // 익명 네임스페이스

bool behaviorPathPointsInsideLanes(
    const Trajectory &path,
    const std::vector<std::vector<BehaviorPathPoint2d>> &lane_polygons) {
  if (path.empty() || lane_polygons.empty()) return false;
  const auto fused = fuseLanes(lane_polygons);
  if (fused.empty()) return false;
  return std::all_of(path.begin(), path.end(), [&](const auto &point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           bg::covered_by(Point{point.x, point.y}, fused);
  });
}

std::optional<BehaviorPathLaneDeparture> firstBehaviorPathLaneDeparture(
    const Trajectory &path,
    const std::vector<std::vector<BehaviorPathPoint2d>> &lane_polygons,
    const BehaviorPathVehicleFootprint &vehicle) {
  if (!std::isfinite(vehicle.front_m) || !std::isfinite(vehicle.back_m) ||
      !std::isfinite(vehicle.half_width_m) ||
      !std::isfinite(vehicle.sample_spacing_m) ||
      vehicle.front_m < 0.0 || vehicle.back_m < 0.0 ||
      vehicle.half_width_m <= 0.0 || vehicle.sample_spacing_m <= 0.0) {
    throw std::invalid_argument("차로 이탈 차량 외형 입력이 유효하지 않습니다");
  }
  if (path.empty()) return std::nullopt;
  if (lane_polygons.empty()) return BehaviorPathLaneDeparture{};
  for (const auto &point : path) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(yaw(point))) {
      throw std::invalid_argument("차로 이탈 경로 점이 유효하지 않습니다");
    }
  }
  for (const auto &polygon : lane_polygons) {
    for (const auto &point : polygon) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        throw std::invalid_argument("차로 다각형 점이 유효하지 않습니다");
      }
    }
  }
  const auto fused_lanes = fuseLanes(lane_polygons);
  if (fused_lanes.empty()) return BehaviorPathLaneDeparture{};
  for (std::size_t index = 0; index < path.size(); ++index) {
    const auto &first = path[index];
    if (index + 1 == path.size()) {
      if (!footprintInside(first.x, first.y, yaw(first),
                           fused_lanes, vehicle)) {
        return BehaviorPathLaneDeparture{index, 0.0};
      }
      continue;
    }
    const auto &last = path[index + 1];
    const double distance = std::hypot(last.x - first.x, last.y - first.y);
    if (distance / vehicle.sample_spacing_m > 1.0e6) {
      throw std::invalid_argument("차로 이탈 검사 표본 수가 과도합니다");
    }
    const std::size_t samples = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::ceil(
               distance / vehicle.sample_spacing_m)));
    const double first_yaw = yaw(first);
    const double delta_yaw = std::remainder(
        yaw(last) - first_yaw, 2.0 * 3.14159265358979323846);
    for (std::size_t sample = 0; sample < samples; ++sample) {
      const double fraction = static_cast<double>(sample) / samples;
      if (!footprintInside(
              first.x + fraction * (last.x - first.x),
              first.y + fraction * (last.y - first.y),
              first_yaw + fraction * delta_yaw,
              fused_lanes, vehicle)) {
        return BehaviorPathLaneDeparture{index, fraction};
      }
    }
  }
  return std::nullopt;
}

std::vector<StartPullOutCandidate> selectLaneSafeStartPullOutCandidates(
    const std::vector<StartPullOutCandidate> &candidates,
    const std::vector<std::vector<BehaviorPathPoint2d>> &lane_polygons,
    const BehaviorPathVehicleFootprint &vehicle) {
  std::vector<StartPullOutCandidate> selected;
  for (const auto &candidate : candidates) {
    if (!firstBehaviorPathLaneDeparture(
            candidate.path, lane_polygons, vehicle)) {
      selected.push_back(candidate);
    }
  }
  return selected;
}

std::vector<std::optional<BehaviorSafetyPose>> sampleStartPullOutBackwardPoses(
    const std::vector<BehaviorPathPoint2d> &centerline,
    const BehaviorSafetyPose &start_pose,
    const std::vector<double> &backward_distances_m) {
  if (centerline.size() < 2 ||
      !std::isfinite(start_pose.x) || !std::isfinite(start_pose.y)) {
    throw std::invalid_argument("출발 후방 중심선 입력이 유효하지 않습니다");
  }
  for (const auto distance : backward_distances_m) {
    if (!std::isfinite(distance) || distance < 0.0) {
      throw std::invalid_argument("출발 후방 표본 거리가 유효하지 않습니다");
    }
  }
  const auto cumulative = [](const std::vector<BehaviorPathPoint2d> &path) {
    std::vector<double> arcs(path.size(), 0.0);
    for (std::size_t index = 0; index < path.size(); ++index) {
      if (!std::isfinite(path[index].x) || !std::isfinite(path[index].y)) {
        throw std::invalid_argument("출발 후방 중심선 좌표가 유효하지 않습니다");
      }
      if (index == 0) continue;
      const double segment = std::hypot(
          path[index].x - path[index - 1].x,
          path[index].y - path[index - 1].y);
      if (segment <= 1.0e-6) {
        throw std::invalid_argument("출발 후방 중심선 길이가 유효하지 않습니다");
      }
      arcs[index] = arcs[index - 1] + segment;
    }
    return arcs;
  };
  const auto project = [](const std::vector<BehaviorPathPoint2d> &path,
                          const std::vector<double> &arcs,
                          double x, double y) {
    double best_error = std::numeric_limits<double>::infinity();
    double best_arc = 0.0;
    std::size_t best_index = 0;
    double best_x = 0.0;
    double best_y = 0.0;
    for (std::size_t index = 0; index + 1 < path.size(); ++index) {
      const double dx = path[index + 1].x - path[index].x;
      const double dy = path[index + 1].y - path[index].y;
      const double length = arcs[index + 1] - arcs[index];
      const double fraction = std::clamp(
          ((x - path[index].x) * dx + (y - path[index].y) * dy) /
              (length * length), 0.0, 1.0);
      const double projected_x = path[index].x + fraction * dx;
      const double projected_y = path[index].y + fraction * dy;
      const double error = std::hypot(x - projected_x, y - projected_y);
      if (error < best_error) {
        best_error = error;
        best_arc = arcs[index] + fraction * length;
        best_index = index;
        best_x = projected_x;
        best_y = projected_y;
      }
    }
    return std::tuple<double, std::size_t, double, double>{
        best_arc, best_index, best_x, best_y};
  };
  const auto center_arcs = cumulative(centerline);
  const auto [center_arc, center_index, center_x, center_y] = project(
      centerline, center_arcs, start_pose.x, start_pose.y);
  (void)center_arc;
  const double center_dx = centerline[center_index + 1].x -
                           centerline[center_index].x;
  const double center_dy = centerline[center_index + 1].y -
                           centerline[center_index].y;
  const double center_length = center_arcs[center_index + 1] -
                               center_arcs[center_index];
  const double lateral =
      ((start_pose.y - center_y) * center_dx -
       (start_pose.x - center_x) * center_dy) / center_length;
  std::vector<BehaviorPathPoint2d> shifted(centerline.size());
  std::vector<double> headings(centerline.size());
  for (std::size_t index = 0; index < centerline.size(); ++index) {
    const std::size_t segment = std::min(index, centerline.size() - 2);
    const double dx = centerline[segment + 1].x - centerline[segment].x;
    const double dy = centerline[segment + 1].y - centerline[segment].y;
    const double heading = std::atan2(dy, dx);
    headings[index] = heading;
    shifted[index] = {
        centerline[index].x - lateral * std::sin(heading),
        centerline[index].y + lateral * std::cos(heading)};
  }
  const auto shifted_arcs = cumulative(shifted);
  const auto [start_arc, ignored_index, ignored_x, ignored_y] = project(
      shifted, shifted_arcs, start_pose.x, start_pose.y);
  (void)ignored_index;
  (void)ignored_x;
  (void)ignored_y;
  std::vector<std::optional<BehaviorSafetyPose>> samples;
  samples.reserve(backward_distances_m.size());
  for (const auto distance : backward_distances_m) {
    const double target = start_arc - distance;
    if (target < 0.0 || target > shifted_arcs.back()) {
      samples.push_back(std::nullopt);
      continue;
    }
    const auto upper = std::lower_bound(
        shifted_arcs.begin(), shifted_arcs.end(), target);
    const std::size_t after = upper == shifted_arcs.begin() ?
        1 : static_cast<std::size_t>(upper - shifted_arcs.begin());
    const std::size_t before = after - 1;
    const double fraction = std::clamp(
        (target - shifted_arcs[before]) /
            (shifted_arcs[after] - shifted_arcs[before]), 0.0, 1.0);
    const double sample_yaw = fraction > 1.0 - 1.0e-6 ?
        headings[after] : std::atan2(
            shifted[after].y - shifted[before].y,
            shifted[after].x - shifted[before].x);
    samples.push_back(BehaviorSafetyPose{
        0.0,
        shifted[before].x + fraction *
            (shifted[after].x - shifted[before].x),
        shifted[before].y + fraction *
            (shifted[after].y - shifted[before].y),
        sample_yaw, 0.0});
  }
  return samples;
}

std::vector<BehaviorSafetyPose> searchStartPullOutPoseCandidates(
    const StartPullOutSearchInput &input) {
  const double values[] = {
      input.start_lane_arc_m, input.lane_length_m,
      input.base_link_to_rear_m, input.maximum_backward_distance_m,
      input.ignore_distance_from_lane_end_m,
      input.front_collision_margin_m,
      input.backward_collision_margin_m};
  for (const double value : values) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("출발 시작 자세 검색 값이 유효하지 않습니다");
    }
  }
  if (input.start_lane_arc_m > input.lane_length_m) {
    throw std::invalid_argument("출발 차로 호길이가 유효하지 않습니다");
  }
  const double allowed_backward_distance = std::clamp(
      input.start_lane_arc_m - input.base_link_to_rear_m,
      0.0, input.maximum_backward_distance_m);
  std::vector<BehaviorSafetyPose> selected;
  double previous_distance = -1.0;
  for (const auto &sample : input.samples) {
    if (!std::isfinite(sample.backward_distance_m) ||
        !std::isfinite(sample.lane_arc_m) ||
        sample.backward_distance_m < 0.0 ||
        sample.lane_arc_m < 0.0 ||
        sample.lane_arc_m > input.lane_length_m ||
        sample.backward_distance_m <= previous_distance) {
      throw std::invalid_argument("출발 시작 자세 표본이 유효하지 않습니다");
    }
    previous_distance = sample.backward_distance_m;
    if (sample.backward_distance_m > allowed_backward_distance) break;
    if (behaviorFootprintCollidesWithObjects(
            sample.pose, input.vehicle,
            input.front_stop_objects,
            input.front_collision_margin_m)) {
      continue;
    }
    if (input.lane_length_m - sample.lane_arc_m <
        input.ignore_distance_from_lane_end_m) {
      continue;
    }
    if (behaviorFootprintCollidesWithObjects(
            sample.pose, input.vehicle,
            input.all_stop_objects,
            input.backward_collision_margin_m)) {
      break;
    }
    selected.push_back(sample.pose);
  }
  return selected;
}

std::vector<BehaviorSafetyPolygon> filterStartPullOutStopObjects(
    const std::vector<StartPullOutStopObject> &objects,
    const std::vector<std::vector<BehaviorPathPoint2d>> &lane_polygons,
    const std::vector<BehaviorPathPoint2d> &reference_path,
    double current_x, double current_y,
    double maximum_velocity_mps,
    double forward_distance_m, double backward_distance_m) {
  const double values[] = {
      current_x, current_y, maximum_velocity_mps,
      forward_distance_m, backward_distance_m};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("출발 정지 객체 검색 값이 유효하지 않습니다");
    }
  }
  if (maximum_velocity_mps < 0.0 || forward_distance_m < 0.0 ||
      backward_distance_m < 0.0) {
    throw std::invalid_argument("출발 정지 객체 검색 거리가 유효하지 않습니다");
  }
  if (reference_path.size() < 2 || lane_polygons.empty()) return {};
  for (const auto &lane : lane_polygons) {
    for (const auto &point : lane) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        throw std::invalid_argument("출발 정지 객체 차로 좌표가 유효하지 않습니다");
      }
    }
  }
  std::vector<double> arcs(reference_path.size(), 0.0);
  for (std::size_t index = 0; index < reference_path.size(); ++index) {
    if (!std::isfinite(reference_path[index].x) ||
        !std::isfinite(reference_path[index].y)) {
      throw std::invalid_argument("출발 정지 객체 기준 경로가 유효하지 않습니다");
    }
    if (index > 0) {
      arcs[index] = arcs[index - 1] + std::hypot(
          reference_path[index].x - reference_path[index - 1].x,
          reference_path[index].y - reference_path[index - 1].y);
    }
  }
  const auto projected_arc = [&](double x, double y) {
    double best_distance = std::numeric_limits<double>::infinity();
    double best_arc = 0.0;
    double best_heading = 0.0;
    for (std::size_t index = 0; index + 1 < reference_path.size(); ++index) {
      const double dx = reference_path[index + 1].x - reference_path[index].x;
      const double dy = reference_path[index + 1].y - reference_path[index].y;
      const double length_squared = dx * dx + dy * dy;
      if (length_squared < 1.0e-12) continue;
      const double fraction = std::clamp(
          ((x - reference_path[index].x) * dx +
           (y - reference_path[index].y) * dy) / length_squared,
          0.0, 1.0);
      const double distance = std::hypot(
          x - reference_path[index].x - fraction * dx,
          y - reference_path[index].y - fraction * dy);
      if (distance < best_distance) {
        best_distance = distance;
        best_arc = arcs[index] + fraction * std::sqrt(length_squared);
        best_heading = std::atan2(dy, dx);
      }
    }
    if (!std::isfinite(best_distance)) {
      throw std::invalid_argument("출발 정지 객체 기준 경로 길이가 유효하지 않습니다");
    }
    return std::pair<double, double>{best_arc, best_heading};
  };
  const double current_arc = projected_arc(current_x, current_y).first;
  const auto lanes = fuseLanes(lane_polygons);
  std::vector<BehaviorSafetyPolygon> selected;
  for (const auto &object : objects) {
    if (!std::isfinite(object.velocity_x_mps) ||
        !std::isfinite(object.velocity_y_mps) ||
        !std::isfinite(object.pose.x) || !std::isfinite(object.pose.y) ||
        !std::isfinite(object.pose.yaw_rad)) {
      throw std::invalid_argument("출발 정지 객체 값이 유효하지 않습니다");
    }
    if (std::hypot(object.velocity_x_mps, object.velocity_y_mps) >=
        maximum_velocity_mps) {
      continue;
    }
    const auto world = behaviorSafetyWorldPolygon(object.pose, object.shape);
    Polygon polygon;
    for (const auto &[x, y] : world) polygon.outer().emplace_back(x, y);
    bg::correct(polygon);
    if (!bg::intersects(polygon, lanes)) continue;
    const auto [object_arc, path_heading] = projected_arc(
        object.pose.x, object.pose.y);
    // 원본의 기본 차로 방향 허용각은 π이며, 경계값 자체는 제외합니다.
    if (std::abs(std::remainder(
            object.pose.yaw_rad - path_heading,
            2.0 * std::acos(-1.0))) >= std::acos(-1.0)) {
      continue;
    }
    const double signed_arc = object_arc - current_arc;
    if (signed_arc <= -backward_distance_m ||
        signed_arc >= forward_distance_m || !object.allowed_class) {
      continue;
    }
    selected.push_back(world);
  }
  return selected;
}

std::vector<BehaviorSafetyPose> planStartPullOutPoseCandidates(
    const StartPullOutCandidatePlanInput &input) {
  const double values[] = {
      input.start_lane_arc_m, input.lane_length_m,
      input.base_link_to_rear_m, input.maximum_backward_distance_m,
      input.backward_search_resolution_m, input.backward_path_length_m,
      input.ignore_distance_from_lane_end_m,
      input.moving_object_threshold_mps,
      input.first_collision_margin_m,
      input.second_collision_margin_m};
  for (const double value : values) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("출발 후보 연결 입력이 유효하지 않습니다");
    }
  }
  if (input.backward_search_resolution_m <= 0.0 ||
      input.start_lane_arc_m > input.lane_length_m) {
    throw std::invalid_argument("출발 후보 연결 거리 설정이 유효하지 않습니다");
  }
  const double allowed_backward_distance = std::clamp(
      input.start_lane_arc_m - input.base_link_to_rear_m,
      0.0, input.maximum_backward_distance_m);
  if (allowed_backward_distance / input.backward_search_resolution_m >
      100000.0) {
    throw std::invalid_argument("출발 후보 표본 수가 과도합니다");
  }
  std::vector<double> distances;
  for (double distance = 0.0; distance <= allowed_backward_distance;
       distance += input.backward_search_resolution_m) {
    distances.push_back(distance);
  }
  const auto poses = sampleStartPullOutBackwardPoses(
      input.centerline, input.original_start_pose, distances);
  const auto projected_arc = [&](double x, double y) {
    double best_distance = std::numeric_limits<double>::infinity();
    double best_arc = 0.0;
    double arc = 0.0;
    for (std::size_t index = 0; index + 1 < input.centerline.size(); ++index) {
      const double dx = input.centerline[index + 1].x -
                        input.centerline[index].x;
      const double dy = input.centerline[index + 1].y -
                        input.centerline[index].y;
      const double length_squared = dx * dx + dy * dy;
      if (length_squared < 1.0e-12) continue;
      const double fraction = std::clamp(
          ((x - input.centerline[index].x) * dx +
           (y - input.centerline[index].y) * dy) / length_squared,
          0.0, 1.0);
      const double error = std::hypot(
          x - input.centerline[index].x - fraction * dx,
          y - input.centerline[index].y - fraction * dy);
      if (error < best_distance) {
        best_distance = error;
        best_arc = arc + fraction * std::sqrt(length_squared);
      }
      arc += std::sqrt(length_squared);
    }
    return best_arc;
  };
  const double projected_start = projected_arc(
      input.original_start_pose.x, input.original_start_pose.y);
  StartPullOutSearchInput search;
  search.start_lane_arc_m = input.start_lane_arc_m;
  search.lane_length_m = input.lane_length_m;
  search.base_link_to_rear_m = input.base_link_to_rear_m;
  search.maximum_backward_distance_m = input.maximum_backward_distance_m;
  search.ignore_distance_from_lane_end_m =
      input.ignore_distance_from_lane_end_m;
  search.front_collision_margin_m = input.first_collision_margin_m;
  search.backward_collision_margin_m = input.second_collision_margin_m;
  search.vehicle = input.vehicle;
  for (std::size_t index = 0; index < poses.size(); ++index) {
    if (!poses[index]) continue;
    const double lane_arc = input.start_lane_arc_m +
                            projected_arc(poses[index]->x, poses[index]->y) -
                            projected_start;
    search.samples.push_back({*poses[index], distances[index],
        std::clamp(lane_arc, 0.0, input.lane_length_m)});
  }
  search.front_stop_objects = filterStartPullOutStopObjects(
      input.objects, input.lane_polygons, input.centerline,
      input.original_start_pose.x, input.original_start_pose.y,
      input.moving_object_threshold_mps, 0.0,
      std::numeric_limits<double>::max());
  search.all_stop_objects = filterStartPullOutStopObjects(
      input.objects, input.lane_polygons, input.centerline,
      input.original_start_pose.x, input.original_start_pose.y,
      input.moving_object_threshold_mps,
      input.backward_path_length_m + input.maximum_backward_distance_m,
      std::numeric_limits<double>::max());
  return searchStartPullOutPoseCandidates(search);
}

}  // planning 네임스페이스
