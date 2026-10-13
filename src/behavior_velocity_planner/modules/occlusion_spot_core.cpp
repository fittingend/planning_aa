// SPDX-License-Identifier: Apache-2.0
// 가림 후보를 경로 호길이에 투영하고 원본의 예측 제동식을 적용합니다.
#include "planning/occlusion_spot_core.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <stdexcept>

namespace planning {
namespace {
struct Projection {
  double arc = 0.0;
  double lateral = 0.0;
  CrosswalkPoint foot;
};

double cross(CrosswalkPoint a, CrosswalkPoint b, CrosswalkPoint c) {
  return (b.x - a.x) * (c.y - a.y) -
         (b.y - a.y) * (c.x - a.x);
}

bool intersects(CrosswalkPoint a, CrosswalkPoint b,
                CrosswalkPoint c, CrosswalkPoint d) {
  const double first = cross(a, b, c);
  const double second = cross(a, b, d);
  const double third = cross(c, d, a);
  const double fourth = cross(c, d, b);
  return first * second <= 0.0 && third * fourth <= 0.0 &&
      std::max(std::min(a.x, b.x), std::min(c.x, d.x)) <=
          std::min(std::max(a.x, b.x), std::max(c.x, d.x)) &&
      std::max(std::min(a.y, b.y), std::min(c.y, d.y)) <=
          std::min(std::max(a.y, b.y), std::max(c.y, d.y));
}

Projection project(const Trajectory &path, CrosswalkPoint point) {
  Projection best;
  double distance = std::numeric_limits<double>::infinity();
  double traveled = 0.0;
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const double dx = path[index + 1].x - path[index].x;
    const double dy = path[index + 1].y - path[index].y;
    const double length = std::hypot(dx, dy);
    if (length < 1.0e-9) continue;
    const double ratio = std::clamp(
        ((point.x - path[index].x) * dx +
         (point.y - path[index].y) * dy) / (length * length),
        0.0, 1.0);
    const double lateral = (dx * (point.y - path[index].y) -
                            dy * (point.x - path[index].x)) / length;
    const double error = std::hypot(
        point.x - path[index].x - ratio * dx,
        point.y - path[index].y - ratio * dy);
    if (error < distance) {
      distance = error;
      best = {traveled + ratio * length, lateral,
              {path[index].x + ratio * dx,
               path[index].y + ratio * dy}};
    }
    traveled += length;
  }
  return best;
}

bool insideVehicle(CrosswalkPoint point,
                   const CrosswalkOcclusionMaskObject &vehicle) {
  const double dx = point.x - vehicle.pose.x;
  const double dy = point.y - vehicle.pose.y;
  const double c = std::cos(vehicle.pose.yaw_rad);
  const double s = std::sin(vehicle.pose.yaw_rad);
  return std::abs(c * dx + s * dy) <= vehicle.length_m / 2.0 &&
         std::abs(-s * dx + c * dy) <= vehicle.width_m / 2.0;
}

bool rayBlockedByVehicle(CrosswalkPoint source, CrosswalkPoint target,
                         const CrosswalkOcclusionMaskObject &vehicle) {
  const double c = std::cos(vehicle.pose.yaw_rad);
  const double s = std::sin(vehicle.pose.yaw_rad);
  const auto local = [&](CrosswalkPoint point) {
    const double dx = point.x - vehicle.pose.x;
    const double dy = point.y - vehicle.pose.y;
    return CrosswalkPoint{c * dx + s * dy, -s * dx + c * dy};
  };
  const auto a = local(source);
  const auto b = local(target);
  double entry = 0.0;
  double exit = 1.0;
  const auto clip = [&](double start, double delta,
                        double minimum, double maximum) {
    if (std::abs(delta) < 1.0e-12)
      return start >= minimum && start <= maximum;
    double first = (minimum - start) / delta;
    double last = (maximum - start) / delta;
    if (first > last) std::swap(first, last);
    entry = std::max(entry, first);
    exit = std::min(exit, last);
    return entry <= exit;
  };
  return clip(a.x, b.x - a.x, -vehicle.length_m / 2.0,
              vehicle.length_m / 2.0) &&
         clip(a.y, b.y - a.y, -vehicle.width_m / 2.0,
              vehicle.width_m / 2.0) && entry < 1.0 && exit > 0.0;
}

CrosswalkPoint cellPoint(const CrosswalkOccupancyGrid &grid,
                         std::size_t x, std::size_t y) {
  const double local_x = (x + 0.5) * grid.resolution_m;
  const double local_y = (y + 0.5) * grid.resolution_m;
  const double c = std::cos(grid.origin_yaw_rad);
  const double s = std::sin(grid.origin_yaw_rad);
  return {grid.origin_x + c * local_x - s * local_y,
          grid.origin_y + s * local_x + c * local_y};
}

bool occupiedCrossesPedestrianPath(const CrosswalkOccupancyGrid &grid,
                                   CrosswalkPoint from, CrosswalkPoint to,
                                   double radius_m, int occupied_min) {
  const double c = std::cos(grid.origin_yaw_rad);
  const double s = std::sin(grid.origin_yaw_rad);
  const auto local = [&](CrosswalkPoint point) {
    const double x = point.x - grid.origin_x;
    const double y = point.y - grid.origin_y;
    return CrosswalkPoint{c * x + s * y, -s * x + c * y};
  };
  const auto first = local(from);
  const auto last = local(to);
  const auto lower = [&](double value) {
    return std::max(0.0, std::floor((value - radius_m) /
                                    grid.resolution_m - 0.5));
  };
  const auto upper = [&](double value, std::size_t limit) {
    return std::clamp(std::ceil((value + radius_m) /
                                grid.resolution_m + 0.5),
                      0.0, static_cast<double>(limit));
  };
  const std::size_t min_x = static_cast<std::size_t>(
      lower(std::min(first.x, last.x)));
  const std::size_t min_y = static_cast<std::size_t>(
      lower(std::min(first.y, last.y)));
  const std::size_t max_x = static_cast<std::size_t>(
      upper(std::max(first.x, last.x), grid.width));
  const std::size_t max_y = static_cast<std::size_t>(
      upper(std::max(first.y, last.y), grid.height));
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double squared_length = dx * dx + dy * dy;
  for (std::size_t y = min_y; y < max_y; ++y) {
    for (std::size_t x = min_x; x < max_x; ++x) {
      if (grid.values[y * grid.width + x] < occupied_min) continue;
      const auto point = cellPoint(grid, x, y);
      const double ratio = squared_length > 1.0e-12 ?
          std::clamp(((point.x - from.x) * dx +
                      (point.y - from.y) * dy) / squared_length,
                     0.0, 1.0) : 0.0;
      if (std::hypot(point.x - from.x - ratio * dx,
                     point.y - from.y - ratio * dy) <= radius_m)
        return true;
    }
  }
  return false;
}

struct SafeMotion {
  double velocity = 0.0;
  double stop_distance = 0.0;
};

SafeMotion safeStoppingMotion(const OcclusionSpotParameters &p,
                              double time_to_vehicle_s) {
  const double jerk = p.safety_ratio * p.maximum_stop_jerk_mps3;
  const double acceleration = p.safety_ratio *
                              p.maximum_stop_acceleration_mps2;
  const double delay = p.response_delay_s;
  const double jerk_time = acceleration / jerk;
  SafeMotion result;
  if (time_to_vehicle_s > delay) {
    const double active = time_to_vehicle_s - delay;
    if (active <= jerk_time) {
      const double safe_velocity = -0.5 * jerk * active * active;
      result.velocity = safe_velocity;
      result.stop_distance = safe_velocity * delay -
                             jerk * active * active * active / 6.0;
    } else {
      const double constant_time = active - jerk_time;
      const double jerk_velocity = -0.5 * jerk * jerk_time * jerk_time;
      const double safe_velocity = jerk_velocity - acceleration * constant_time;
      result.velocity = safe_velocity;
      result.stop_distance = safe_velocity * delay -
                             jerk * jerk_time * jerk_time * jerk_time / 6.0 +
                             jerk_velocity * constant_time -
                             acceleration * constant_time * constant_time / 2.0;
    }
  }
  result.stop_distance += p.safe_margin_m;
  return result;
}

double velocityAtArc(const Trajectory &path, double target_arc) {
  double traveled = 0.0;
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const double length = std::hypot(
        path[index + 1].x - path[index].x,
        path[index + 1].y - path[index].y);
    if (traveled + length >= target_arc)
      return path[index].longitudinal_velocity_mps;
    traveled += length;
  }
  return path.back().longitudinal_velocity_mps;
}
}  // 익명 네임스페이스

OcclusionSpotResult planOcclusionSpot(
    const Trajectory &path, CrosswalkPoint ego,
    double ego_speed_mps, double ego_acceleration_mps2,
    const std::optional<CrosswalkOccupancyGrid> &grid,
    const std::vector<CrosswalkOcclusionMaskObject> &vehicles,
    const OcclusionSpotParameters &p,
    const std::vector<VelocityLimiterSegment> &partitions) {
  OcclusionSpotResult result{path, {}};
  if (path.size() < 2) return result;
  const double values[] = {
      p.detection_length_m, p.maximum_lateral_distance_m,
      p.minimum_occlusion_size_m, p.pedestrian_speed_mps,
      p.pedestrian_radius_m,
      p.vehicle_front_m, p.vehicle_half_width_m,
      p.stuck_vehicle_speed_mps, p.safety_ratio,
      p.maximum_stop_jerk_mps3, p.maximum_stop_acceleration_mps2,
      p.maximum_slow_jerk_mps3, p.maximum_slow_acceleration_mps2,
      p.minimum_velocity_mps, p.safe_margin_m, p.response_delay_s};
  if (!std::all_of(std::begin(values), std::end(values),
                   [](double value) { return std::isfinite(value); }) ||
      p.detection_length_m <= 0.0 ||
      p.maximum_lateral_distance_m <= p.vehicle_half_width_m ||
      p.minimum_occlusion_size_m <= 0.0 || p.pedestrian_speed_mps <= 0.0 ||
      p.pedestrian_radius_m < 0.0 ||
      p.safety_ratio <= 0.0 || p.vehicle_front_m < 0.0 ||
      p.vehicle_half_width_m < 0.0 || p.response_delay_s < 0.0 ||
      p.minimum_velocity_mps < 0.0 || p.safe_margin_m < 0.0 ||
      p.maximum_stop_jerk_mps3 >= 0.0 ||
      p.maximum_stop_acceleration_mps2 >= 0.0 ||
      p.maximum_slow_jerk_mps3 >= 0.0 ||
      p.maximum_slow_acceleration_mps2 >= 0.0 ||
      !std::isfinite(ego_speed_mps) || !std::isfinite(ego_acceleration_mps2))
    throw std::invalid_argument("가림 영역 계획 입력이 유효하지 않습니다");
  const double detection_length =
      p.method == OcclusionSpotDetectionMethod::OccupancyGrid ?
          std::min(35.0, p.detection_length_m) : p.detection_length_m;
  const auto detection_path = resampleOcclusionDetectionPath(
      path, detection_length);
  if (detection_path.size() < 2) return result;
  std::vector<CrosswalkPoint> candidates;
  if (p.method == OcclusionSpotDetectionMethod::OccupancyGrid && grid) {
    if (grid->width == 0 || grid->height == 0 ||
        grid->width > grid->values.size() / grid->height ||
        grid->width * grid->height != grid->values.size() ||
        !std::isfinite(grid->resolution_m) || grid->resolution_m <= 0.0 ||
        !std::isfinite(grid->origin_x) || !std::isfinite(grid->origin_y) ||
        !std::isfinite(grid->origin_yaw_rad))
      throw std::invalid_argument("가림 영역 격자가 유효하지 않습니다");
    const std::size_t stride = grid->width + 1;
    std::vector<std::size_t> prefix(stride * (grid->height + 1), 0);
    const auto scan_origin = cellPoint(*grid, grid->width / 2,
                                      grid->height / 2);
    for (std::size_t y = 0; y < grid->height; ++y) {
      for (std::size_t x = 0; x < grid->width; ++x) {
        const int value = grid->values[y * grid->width + x];
        const std::size_t unknown =
            value > p.free_space_max && value < p.occupied_min;
        prefix[(y + 1) * stride + x + 1] = unknown +
            prefix[y * stride + x + 1] +
            prefix[(y + 1) * stride + x] - prefix[y * stride + x];
      }
    }
    // 원본의 2x2 침식은 기본 앵커 (1,1)를 사용합니다.
    const double iteration_ratio =
        p.minimum_occlusion_size_m / grid->resolution_m - 1.0;
    const std::size_t iterations = iteration_ratio > 0.0 ?
        static_cast<std::size_t>(iteration_ratio) : 0;
    for (std::size_t y = 0; y < grid->height; ++y) {
      const std::size_t begin_y = y > iterations ? y - iterations : 0;
      for (std::size_t x = 0; x < grid->width; ++x) {
        const std::size_t begin_x = x > iterations ? x - iterations : 0;
        const std::size_t count =
            prefix[(y + 1) * stride + x + 1] -
            prefix[begin_y * stride + x + 1] -
            prefix[(y + 1) * stride + begin_x] +
            prefix[begin_y * stride + begin_x];
        if (count != (x - begin_x + 1) * (y - begin_y + 1))
          continue;
        const auto point = cellPoint(*grid, x, y);
        const bool masked = std::any_of(vehicles.begin(), vehicles.end(),
            [&](const auto &vehicle) {
              auto mask = vehicle;
              const bool moving = vehicle.longitudinal_speed_mps >
                                  p.stuck_vehicle_speed_mps;
              const double scale = moving ? 1.5 : 0.8;
              mask.length_m *= scale;
              mask.width_m *= scale;
              if (moving)
                return p.use_moving_vehicle_ray_mask &&
                       (insideVehicle(point, mask) ||
                        rayBlockedByVehicle(scan_origin, point, mask));
              return insideVehicle(point, mask);
            });
        if (!masked) candidates.push_back(point);
      }
    }
  } else if (p.method == OcclusionSpotDetectionMethod::StoppedObjects) {
    for (const auto &vehicle : vehicles) {
      if (vehicle.longitudinal_speed_mps > p.stuck_vehicle_speed_mps ||
          vehicle.length_m <= 0.0 || vehicle.width_m <= 0.0) continue;
      const double c = std::cos(vehicle.pose.yaw_rad);
      const double s = std::sin(vehicle.pose.yaw_rad);
      std::vector<std::pair<CrosswalkPoint, Projection>> corners;
      for (const double forward : {-vehicle.length_m / 2.0,
                                   vehicle.length_m / 2.0}) {
        for (const double lateral : {-vehicle.width_m / 2.0,
                                     vehicle.width_m / 2.0}) {
          const CrosswalkPoint point{
              vehicle.pose.x + c * forward - s * lateral,
              vehicle.pose.y + s * forward + c * lateral};
          corners.emplace_back(point, project(detection_path, point));
        }
      }
      std::sort(corners.begin(), corners.end(),
                [](const auto &first, const auto &second) {
                  return first.second.arc < second.second.arc;
                });
      const auto selected = std::min_element(
          corners.begin() + 2, corners.end(),
          [](const auto &first, const auto &second) {
            return std::abs(first.second.lateral) <
                   std::abs(second.second.lateral);
          });
      candidates.push_back(selected->first);
    }
  }
  const double ego_arc = project(detection_path, ego).arc;
  std::vector<std::pair<double, double>> restrictions;
  std::map<int, std::pair<CrosswalkPoint, double>> nearest_by_slice;
  for (const auto &candidate : candidates) {
    const auto projection = project(detection_path, candidate);
    if (p.method == OcclusionSpotDetectionMethod::OccupancyGrid && grid &&
        occupiedCrossesPedestrianPath(*grid, candidate, projection.foot,
                                      p.pedestrian_radius_m, p.occupied_min))
      continue;
    if (p.method == OcclusionSpotDetectionMethod::OccupancyGrid &&
        std::any_of(vehicles.begin(), vehicles.end(),
                    [&](const auto &vehicle) {
          const bool moving = vehicle.longitudinal_speed_mps >
                              p.stuck_vehicle_speed_mps;
          if (moving && !p.use_moving_vehicle_ray_mask) return false;
          auto mask = vehicle;
          const double scale = moving ? 1.5 : 0.8;
          mask.length_m = scale * mask.length_m +
                          2.0 * p.pedestrian_radius_m;
          mask.width_m = scale * mask.width_m +
                         2.0 * p.pedestrian_radius_m;
          return rayBlockedByVehicle(candidate, projection.foot, mask);
        }))
      continue;
    if (std::any_of(partitions.begin(), partitions.end(),
                    [&](const auto &partition) {
                      return intersects(candidate, projection.foot,
                                        {partition.from.x, partition.from.y},
                                        {partition.to.x, partition.to.y});
                    })) continue;
    const double distance = projection.arc - ego_arc - p.vehicle_front_m;
    const double lateral = std::abs(projection.lateral) -
                           p.vehicle_half_width_m;
    if (distance <= 0.0 || distance > detection_length ||
        lateral <= 0.0 || lateral > p.maximum_lateral_distance_m) continue;
    const int slice = static_cast<int>(distance / 10.0);
    auto found = nearest_by_slice.find(slice);
    if (found == nearest_by_slice.end() || lateral < found->second.second)
      nearest_by_slice[slice] = {candidate, lateral};
  }
  for (const auto &[slice, selected] : nearest_by_slice) {
    (void)slice;
    const auto projection = project(detection_path, selected.first);
    const double lateral = selected.second;
    const double ttv = lateral / p.pedestrian_speed_mps;
    const auto safe_motion = safeStoppingMotion(p, ttv);
    const double collision_arc = std::max(0.1, projection.arc -
        p.vehicle_front_m - safe_motion.stop_distance);
    const double smooth_velocity = crosswalkFeasibleSlowVelocity(
        p.maximum_slow_jerk_mps3, p.maximum_slow_acceleration_mps2,
        ego_acceleration_mps2, std::max(0.0, ego_speed_mps),
        collision_arc - ego_arc);
    const double velocity = std::max(p.minimum_velocity_mps,
                                     smooth_velocity + 0.05);
    const double original_velocity = velocityAtArc(
        detection_path, collision_arc);
    if (original_velocity < safe_motion.velocity ||
        original_velocity < smooth_velocity) continue;
    restrictions.emplace_back(collision_arc, velocity);
  }
  std::sort(restrictions.begin(), restrictions.end());
  for (const auto &[collision_arc, velocity] : restrictions) {
    double traveled = 0.0;
    for (std::size_t index = 0; index + 1 < result.trajectory.size(); ++index) {
      const double segment = std::hypot(
          result.trajectory[index + 1].x - result.trajectory[index].x,
          result.trajectory[index + 1].y - result.trajectory[index].y);
      if (traveled + segment + 1.0e-9 < collision_arc) {
        traveled += segment;
        continue;
      }
      const double original = result.trajectory[index].longitudinal_velocity_mps;
      if (original <= velocity) break;
      const double ratio = segment > 1.0e-9 ?
          std::clamp((collision_arc - traveled) / segment, 0.0, 1.0) : 0.0;
      if (ratio > 1.0e-6 && ratio < 1.0 - 1.0e-6) {
        auto inserted = result.trajectory[index];
        const auto &next = result.trajectory[index + 1];
        inserted.x += ratio * (next.x - inserted.x);
        inserted.y += ratio * (next.y - inserted.y);
        inserted.z += ratio * (next.z - inserted.z);
        result.trajectory.insert(result.trajectory.begin() + index + 1,
                                 inserted);
        ++index;
      } else if (ratio >= 1.0 - 1.0e-6) {
        ++index;
      }
      for (std::size_t later = index; later < result.trajectory.size(); ++later)
        result.trajectory[later].longitudinal_velocity_mps = std::min(
            result.trajectory[later].longitudinal_velocity_mps,
            static_cast<float>(velocity));
      result.collision_arcs_m.push_back(collision_arc);
      break;
    }
  }
  return result;
}

}  // planning 네임스페이스
