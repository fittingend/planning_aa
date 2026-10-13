// SPDX-License-Identifier: Apache-2.0

#include "planning/costmap_generator_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace planning {
namespace {
bool inside(double x, double y, const BehaviorSafetyPolygon &polygon) {
  bool result = false;
  if (polygon.size() < 3) return false;
  for (std::size_t i = 0, j = polygon.size() - 1;
       i < polygon.size(); j = i++) {
    const auto &[xi, yi] = polygon[i];
    const auto &[xj, yj] = polygon[j];
    if ((yi > y) != (yj > y) &&
        x < (xj - xi) * (y - yi) / (yj - yi) + xi)
      result = !result;
  }
  return result;
}

double segmentDistance(double x, double y, double ax, double ay,
                       double bx, double by) {
  const double dx = bx - ax;
  const double dy = by - ay;
  const double squared = dx * dx + dy * dy;
  const double ratio = squared > 1.0e-12 ?
      std::clamp(((x - ax) * dx + (y - ay) * dy) / squared, 0.0, 1.0) :
      0.0;
  return std::hypot(x - ax - ratio * dx, y - ay - ratio * dy);
}

bool insideExpanded(double x, double y,
                    const BehaviorSafetyPolygon &polygon, double margin) {
  if (inside(x, y, polygon)) return true;
  if (margin <= 0.0 || polygon.size() < 2) return false;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto &a = polygon[i];
    const auto &b = polygon[(i + 1) % polygon.size()];
    if (segmentDistance(x, y, a.first, a.second, b.first, b.second) <=
        margin) return true;
  }
  return false;
}

BehaviorSafetyPolygon expandedObjectPolygon(
    const CostmapObject &object, double expansion) {
  if ((object.shape_type == 0 || object.shape_type == 1) &&
      object.length_m > 0.0 && object.width_m > 0.0) {
    // Source expands the full box dimensions, so each side grows by half.
    const double half_length = (object.length_m + expansion) * 0.5;
    const double half_width = (object.width_m + expansion) * 0.5;
    const double cosine = std::cos(object.yaw_rad);
    const double sine = std::sin(object.yaw_rad);
    BehaviorSafetyPolygon polygon;
    for (const auto &[along, across] :
         std::vector<std::pair<double, double>>{
             {half_length, half_width},
             {half_length, -half_width},
             {-half_length, -half_width},
             {-half_length, half_width}})
      polygon.emplace_back(object.center_x + along * cosine -
                               across * sine,
                           object.center_y + along * sine +
                               across * cosine);
    return polygon;
  }
  if (object.shape_type == 2 && object.outline.size() >= 3) {
    BehaviorSafetyPolygon polygon;
    polygon.reserve(object.outline.size());
    for (const auto &[x, y] : object.outline) {
      const double local_x = x - object.center_x;
      const double local_y = y - object.center_y;
      if (expansion == 0.0) {
        // Match makeExpandedPoint's zero-expansion branch.
        polygon.emplace_back(local_x, local_y);
      } else {
        // The source computes the angle from a local footprint corner to
        // the world-frame centroid, then adds the local corner back to it.
        const double theta = std::atan2(
            local_y - object.center_y,
            local_x - object.center_x);
        polygon.emplace_back(x + expansion * std::cos(theta),
                             y + expansion * std::sin(theta));
      }
    }
    return polygon;
  }
  return object.outline;
}

template <typename Fill>
void rasterize(const BehaviorSafetyPolygon &polygon, double margin,
               const CrosswalkOccupancyGrid &grid, Fill fill) {
  if (polygon.size() < 3) return;
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = min_x;
  double max_x = -min_x;
  double max_y = -min_x;
  for (const auto &[x, y] : polygon) {
    min_x = std::min(min_x, x);
    min_y = std::min(min_y, y);
    max_x = std::max(max_x, x);
    max_y = std::max(max_y, y);
  }
  const auto clampIndex = [](long value, std::size_t maximum) {
    return static_cast<std::size_t>(std::clamp(
        value, 0L, static_cast<long>(maximum)));
  };
  const std::size_t first_x = clampIndex(
      static_cast<long>(std::floor(
          (min_x - margin - grid.origin_x) / grid.resolution_m)),
      grid.width);
  const std::size_t last_x = clampIndex(
      static_cast<long>(std::ceil(
          (max_x + margin - grid.origin_x) / grid.resolution_m)),
      grid.width);
  const std::size_t first_y = clampIndex(
      static_cast<long>(std::floor(
          (min_y - margin - grid.origin_y) / grid.resolution_m)),
      grid.height);
  const std::size_t last_y = clampIndex(
      static_cast<long>(std::ceil(
          (max_y + margin - grid.origin_y) / grid.resolution_m)),
      grid.height);
  for (std::size_t y = first_y; y < last_y; ++y) {
    for (std::size_t x = first_x; x < last_x; ++x) {
      const double px = grid.origin_x + (x + 0.5) * grid.resolution_m;
      const double py = grid.origin_y + (y + 0.5) * grid.resolution_m;
      if (insideExpanded(px, py, polygon, margin))
        fill(y * grid.width + x);
    }
  }
}
}  // namespace

CostmapGeneratorCore::CostmapGeneratorCore(
    CostmapGeneratorParameters parameters)
    : parameters_(parameters) {
  if (!(parameters_.length_x_m > 0.0) ||
      !(parameters_.length_y_m > 0.0) ||
      !(parameters_.resolution_m > 0.0) ||
      parameters_.object_expansion_m < 0.0 ||
      parameters_.object_blur_kernel_cells < 1 ||
      parameters_.object_blur_kernel_cells % 2 == 0)
    throw std::invalid_argument("invalid costmap geometry or kernel");
}

void CostmapGeneratorCore::onMapAreas(
    std::vector<BehaviorSafetyPolygon> road,
    std::vector<BehaviorSafetyPolygon> parking) {
  road_ = std::move(road);
  parking_ = std::move(parking);
  map_received_ = true;
}

std::optional<CostmapGeneratorResult> CostmapGeneratorCore::tick(
    const CostmapGeneratorInput &input) {
  if (!map_received_ ||
      !(parameters_.activate_by_scenario ? input.parking_activated :
                                          input.ego_in_parking_lot))
    return std::nullopt;
  if (!std::isfinite(input.ego_x) || !std::isfinite(input.ego_y) ||
      !std::isfinite(input.ego_z)) return std::nullopt;
  const double resolution = parameters_.resolution_m;
  center_x_ += std::round((input.ego_x - center_x_) / resolution) * resolution;
  center_y_ += std::round((input.ego_y - center_y_) / resolution) * resolution;
  CostmapGeneratorResult output;
  output.stamp_s = input.stamp_s;
  output.center_x = center_x_;
  output.center_y = center_y_;
  auto &grid = output.occupancy;
  grid.width = static_cast<std::size_t>(
      std::round(parameters_.length_x_m / resolution));
  grid.height = static_cast<std::size_t>(
      std::round(parameters_.length_y_m / resolution));
  grid.resolution_m = resolution;
  grid.origin_x = center_x_ - grid.width * resolution * 0.5;
  grid.origin_y = center_y_ - grid.height * resolution * 0.5;
  const std::size_t size = grid.width * grid.height;
  output.primitives.assign(size, 1.0f);
  output.points.assign(size, 0.0f);
  output.objects.assign(size, 0.0f);
  output.combined.assign(size, 0.0f);
  if (parameters_.use_wayarea) {
    for (const auto &area : road_)
      rasterize(area, 0.0, grid, [&](std::size_t index) {
        output.primitives[index] = 0.0f;
      });
  }
  if (parameters_.use_parkinglot) {
    for (const auto &area : parking_)
      rasterize(area, 0.0, grid, [&](std::size_t index) {
        output.primitives[index] = 0.0f;
      });
  }
  if (!parameters_.use_wayarea && !parameters_.use_parkinglot) {
    std::fill(output.primitives.begin(), output.primitives.end(), 0.0f);
  }
  if (parameters_.use_points) {
    for (const auto &point : input.points_map_frame) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z) ||
          point.z < input.ego_z + parameters_.minimum_point_height_m ||
          point.z > input.ego_z + parameters_.maximum_point_height_m)
        continue;
      // Source PointsToCostmap uses ceil from the upper grid-map corner;
      // reversing the matrix for OccupancyGrid leaves the point one cell
      // below the usual floor-based coordinate in this recorded case.
      const long x = static_cast<long>(grid.width) - 1 -
          static_cast<long>(std::ceil(
              (grid.origin_x + grid.width * resolution - point.x) /
              resolution));
      const long y = static_cast<long>(grid.height) - 1 -
          static_cast<long>(std::ceil(
              (grid.origin_y + grid.height * resolution - point.y) /
              resolution));
      if (x >= 0 && y >= 0 &&
          x < static_cast<long>(grid.width) &&
          y < static_cast<long>(grid.height))
        output.points[static_cast<std::size_t>(y) * grid.width +
                      static_cast<std::size_t>(x)] = 1.0f;
    }
  }
  if (parameters_.use_objects) {
    for (const auto &object : input.objects_map_frame) {
      const float probability = static_cast<float>(std::clamp(
          object.classification_probability, 0.0, 1.0));
      const auto polygon = expandedObjectPolygon(
          object, parameters_.object_expansion_m);
      const double generic_margin = object.shape_type < 0 ?
          parameters_.object_expansion_m : 0.0;
      rasterize(polygon, generic_margin, grid,
                [&](std::size_t index) {
                  output.objects[index] = std::max(
                      output.objects[index], probability);
                });
    }
    const auto original = output.objects;
    const int radius = parameters_.object_blur_kernel_cells / 2;
    for (std::size_t y = 0; y < grid.height; ++y) {
      for (std::size_t x = 0; x < grid.width; ++x) {
        float sum = 0.0f;
        std::size_t count = 0;
        for (int dy = -radius; dy <= radius; ++dy) {
          const long yy = static_cast<long>(y) + dy;
          if (yy < 0 || yy >= static_cast<long>(grid.height)) continue;
          for (int dx = -radius; dx <= radius; ++dx) {
            const long xx = static_cast<long>(x) + dx;
            if (xx < 0 || xx >= static_cast<long>(grid.width)) continue;
            sum += original[static_cast<std::size_t>(yy) * grid.width +
                            static_cast<std::size_t>(xx)];
            ++count;
          }
        }
        const std::size_t index = y * grid.width + x;
        output.objects[index] = std::max(
            output.objects[index], sum / static_cast<float>(count));
      }
    }
  }
  grid.values.resize(size);
  for (std::size_t i = 0; i < size; ++i) {
    output.combined[i] = std::max({output.primitives[i],
                                   output.points[i], output.objects[i]});
    // grid_map_ros의 OccupancyGrid 변환은 정수 변환으로 소수부를 버린다.
    grid.values[i] = static_cast<std::int8_t>(
        output.combined[i] * 100.0f);
  }
  return output;
}

}  // namespace planning
