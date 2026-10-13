// 경로의 차로 ID와 이진 지도에서 적용할 계획 모듈을 선택합니다.

#include "planning/behavior_velocity_map_planner.hpp"

#include "planning/behavior_path_postprocess.hpp"
#include "planning/crosswalk_core.hpp"
#include "planning/intersection_occlusion_core.hpp"
#include "planning/jerk_resample.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <stdexcept>

namespace planning {
namespace {
struct PluginName {
  std::string_view class_name;
  std::string_view module_name;
  BehaviorVelocityModule module;
};

constexpr std::array<PluginName, 13> plugin_names{{
    {"CrosswalkModulePlugin", "crosswalk", BehaviorVelocityModule::Crosswalk},
    {"WalkwayModulePlugin", "walkway", BehaviorVelocityModule::Walkway},
    {"TrafficLightModulePlugin", "traffic_light", BehaviorVelocityModule::TrafficLight},
    {"StopLineModulePlugin", "stop_line", BehaviorVelocityModule::StopSign},
    {"BlindSpotModulePlugin", "blind_spot", BehaviorVelocityModule::BlindSpot},
    {"VirtualTrafficLightModulePlugin", "virtual_traffic_light", BehaviorVelocityModule::VirtualTrafficLight},
    {"SpeedBumpModulePlugin", "speed_bump", BehaviorVelocityModule::SpeedBump},
    {"RunOutModulePlugin", "run_out", BehaviorVelocityModule::RunOut},
    {"OcclusionSpotModulePlugin", "occlusion_spot", BehaviorVelocityModule::OcclusionSpot},
    {"NoStoppingAreaModulePlugin", "no_stopping_area", BehaviorVelocityModule::NoStoppingArea},
    {"NoDrivableLaneModulePlugin", "no_drivable_lane", BehaviorVelocityModule::NoDrivableLane},
    {"DetectionAreaModulePlugin", "detection_area", BehaviorVelocityModule::DetectionArea},
    {"MergeFromPrivateModulePlugin", "merge_from_private", BehaviorVelocityModule::MergeFromPrivate},
}};
constexpr std::string_view plugin_namespace =
    "autoware::behavior_velocity_planner::";
}  // namespace

std::optional<BehaviorVelocityModule> behaviorVelocityLoadClass(
    std::string_view class_name) {
  if (class_name.substr(0, plugin_namespace.size()) != plugin_namespace)
    return std::nullopt;
  const auto short_class = class_name.substr(plugin_namespace.size());
  const auto found = std::find_if(
      plugin_names.begin(), plugin_names.end(),
      [&](const auto &plugin) { return plugin.class_name == short_class; });
  if (found == plugin_names.end()) return std::nullopt;
  return found->module;
}

std::optional<BehaviorVelocityModule> behaviorVelocityUnloadName(
    std::string_view module_name) {
  const auto found = std::find_if(
      plugin_names.begin(), plugin_names.end(),
      [&](const auto &plugin) { return plugin.module_name == module_name; });
  if (found == plugin_names.end()) return std::nullopt;
  return found->module;
}

namespace {

constexpr std::uint8_t motorcycle_classification = 5;
constexpr std::uint8_t bicycle_classification = 6;
constexpr std::uint8_t pedestrian_classification = 7;

std::optional<CrosswalkPoint> noStoppingOverlapPoint(
    const std::vector<CrosswalkPoint> &area,
    const NoStoppingVehicle &vehicle, const Trajectory &path) {
  namespace bg = boost::geometry;
  using Point = bg::model::d2::point_xy<double>;
  using Polygon = bg::model::polygon<Point>;
  Polygon area_polygon;
  for (const auto &point : area) {
    bg::append(area_polygon.outer(), Point(point.x, point.y));
  }
  bg::correct(area_polygon);
  Polygon footprint;
  const double half_length_m = vehicle.length_m / 2.0;
  const double half_width_m = vehicle.width_m / 2.0;
  const double cosine = std::cos(vehicle.yaw_rad);
  const double sine = std::sin(vehicle.yaw_rad);
  for (const auto &offset : {
           std::pair<double, double>{-half_length_m, -half_width_m},
           {half_length_m, -half_width_m},
           {half_length_m, half_width_m},
           {-half_length_m, half_width_m}}) {
    bg::append(footprint.outer(), Point(
        vehicle.position.x + cosine * offset.first - sine * offset.second,
        vehicle.position.y + sine * offset.first + cosine * offset.second));
  }
  bg::correct(footprint);
  bg::model::linestring<Point> route_line;
  for (const auto &point : path) {
    bg::append(route_line, Point(point.x, point.y));
  }
  std::vector<Polygon> overlap;
  bg::intersection(area_polygon, footprint, overlap);
  for (const auto &piece : overlap) {
    if (std::abs(bg::area(piece)) <= 1.0e-9) continue;
    if (route_line.size() < 2 ||
        bg::distance(piece, route_line) > 0.948) continue;
    Point center(0.0, 0.0);
    bg::centroid(piece, center);
    return CrosswalkPoint{center.x(), center.y()};
  }
  return std::nullopt;
}

bool noStoppingPathStopInArea(
    const Trajectory &path, const std::vector<CrosswalkPoint> &area,
    double ego_x, double ego_y) {
  namespace bg = boost::geometry;
  using Point = bg::model::d2::point_xy<double>;
  using Polygon = bg::model::polygon<Point>;
  using Line = bg::model::linestring<Point>;
  if (path.size() < 3 || area.size() < 3) return false;
  const bool has_stop = std::any_of(
      path.begin(), path.end() - 1, [](const auto &point) {
        return point.longitudinal_velocity_mps <=
            std::numeric_limits<float>::min();
      });
  if (!has_stop) return false;
  Polygon area_polygon;
  for (const auto &point : area) {
    bg::append(area_polygon.outer(), Point(point.x, point.y));
  }
  bg::correct(area_polygon);
  const auto sampled = resampleJerkUniform(path, 0.5);
  std::size_t nearest = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < sampled.size(); ++index) {
    const double distance = std::hypot(
        sampled[index].x - ego_x, sampled[index].y - ego_y);
    if (distance < nearest_distance) {
      nearest = index;
      nearest_distance = distance;
    }
  }
  std::size_t start = nearest + 1;
  double total_distance = 0.0;
  bool found = false;
  for (; start + 1 < sampled.size(); ++start) {
    total_distance += std::hypot(
        sampled[start].x - sampled[start - 1].x,
        sampled[start].y - sampled[start - 1].y);
    if (bg::within(Point(sampled[start].x, sampled[start].y),
                   area_polygon)) {
      found = true;
      break;
    }
    if (total_distance > 200.0) return false;
  }
  if (!found) return false;
  if (start > 1) --start;
  std::size_t end = start;
  double outside_distance = 0.0;
  for (std::size_t index = start; index + 1 < sampled.size(); ++index) {
    total_distance += std::hypot(
        sampled[index].x - sampled[index - 1].x,
        sampled[index].y - sampled[index - 1].y);
    if (!bg::within(Point(sampled[index].x, sampled[index].y),
                    area_polygon)) {
      outside_distance += std::hypot(
          sampled[index].x - sampled[index - 1].x,
          sampled[index].y - sampled[index - 1].y);
    }
    if (total_distance > 200.0 || outside_distance > 2.1) break;
    end = index;
  }
  if (end <= start) return false;
  Polygon corridor;
  constexpr double half_width = 0.948;
  for (std::size_t index = start; index <= end; ++index) {
    const double yaw = 2.0 * std::atan2(
        sampled[index].orientation_z, sampled[index].orientation_w);
    bg::append(corridor.outer(), Point(
        sampled[index].x + half_width * std::sin(yaw),
        sampled[index].y - half_width * std::cos(yaw)));
  }
  for (std::size_t index = end + 1; index-- > start;) {
    const double yaw = 2.0 * std::atan2(
        sampled[index].orientation_z, sampled[index].orientation_w);
    bg::append(corridor.outer(), Point(
        sampled[index].x - half_width * std::sin(yaw),
        sampled[index].y + half_width * std::cos(yaw)));
  }
  bg::correct(corridor);
  double distance_to_end = 0.0;
  for (std::size_t index = path.size() - 1; index > 0; --index) {
    distance_to_end += std::hypot(
        path[index].x - path[index - 1].x,
        path[index].y - path[index - 1].y);
    if (distance_to_end < 1.0) continue;
    if (path[index - 1].longitudinal_velocity_mps >
            std::numeric_limits<float>::min() &&
        path[index].longitudinal_velocity_mps >
            std::numeric_limits<float>::min()) continue;
    const bool stop_after_area =
        (path[index - 1].longitudinal_velocity_mps <=
             std::numeric_limits<float>::min() &&
         !bg::within(Point(path[index - 1].x, path[index - 1].y),
                     area_polygon)) ||
        (path[index].longitudinal_velocity_mps <=
             std::numeric_limits<float>::min() &&
         !bg::within(Point(path[index].x, path[index].y), area_polygon));
    if (!stop_after_area) continue;
    Line segment;
    bg::append(segment, Point(path[index - 1].x, path[index - 1].y));
    bg::append(segment, Point(path[index].x, path[index].y));
    if (bg::intersects(corridor, segment)) return true;
  }
  return false;
}

std::optional<StopLineSegment> noStoppingGeneratedStopLine(
    const Trajectory &path,
    const std::vector<std::vector<CrosswalkPoint>> &areas) {
  namespace bg = boost::geometry;
  using Point = bg::model::d2::point_xy<double>;
  using Polygon = bg::model::polygon<Point>;
  using Line = bg::model::linestring<Point>;
  for (const auto &area : areas) {
    Polygon shape;
    for (const auto &point : area) {
      bg::append(shape.outer(), Point(point.x, point.y));
    }
    bg::correct(shape);
    for (std::size_t index = 0; index + 1 < path.size(); ++index) {
      Line segment;
      bg::append(segment, Point(path[index].x, path[index].y));
      bg::append(segment, Point(path[index + 1].x, path[index + 1].y));
      std::vector<Point> collision;
      bg::intersection(shape, segment, collision);
      if (collision.empty()) continue;
      const double yaw = std::atan2(
          path[index + 1].y - path[index].y,
          path[index + 1].x - path[index].x);
      constexpr double width_m = 1.896;
      // 원본 자동 정지선은 여유 거리를 X 좌표에만 적용합니다.
      constexpr double margin_m = 1.0;
      return StopLineSegment{
          collision.front().x() - margin_m * std::cos(yaw) -
              width_m * std::sin(yaw),
          collision.front().y() +
              width_m * std::cos(yaw),
          collision.front().x() - margin_m * std::cos(yaw) +
              width_m * std::sin(yaw),
          collision.front().y() -
              width_m * std::cos(yaw)};
    }
  }
  return std::nullopt;
}

double noStoppingBrakeDistance(double velocity, double acceleration) {
  if (velocity <= 0.0) return 0.0;
  constexpr double max_acceleration = -2.8;
  constexpr double max_jerk = -5.0;
  constexpr double response_time = 0.5;
  const double first_distance = velocity * response_time;
  const double second_velocity = velocity +
      (max_acceleration * max_acceleration - acceleration * acceleration) /
          (2.0 * max_jerk);
  if (second_velocity <= 0.0) {
    const double duration = -(max_acceleration +
        std::sqrt(acceleration * acceleration -
                  2.0 * max_jerk * velocity)) / max_jerk;
    return std::max(0.0, first_distance + velocity * duration +
        acceleration * duration * duration / 2.0 +
        max_jerk * duration * duration * duration / 6.0);
  }
  const double duration = (max_acceleration - acceleration) / max_jerk;
  return std::max(0.0, first_distance + velocity * duration +
      acceleration * duration * duration / 2.0 +
      max_jerk * duration * duration * duration / 6.0 -
      second_velocity * second_velocity / (2.0 * max_acceleration));
}

std::vector<std::int64_t> laneIdsAhead(const BehaviorVelocityMapInput &input) {
  if (input.lane_ids.size() != input.trajectory.size()) {
    throw std::invalid_argument("경로 점과 차로 ID 목록의 길이가 다릅니다");
  }
  if (input.trajectory.empty()) return {};
  std::size_t nearest = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < input.trajectory.size(); ++index) {
    const auto &point = input.trajectory[index];
    const double distance = std::hypot(point.x - input.ego_x, point.y - input.ego_y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = index;
    }
  }
  std::vector<std::int64_t> ids;
  for (std::size_t index = nearest; index < input.lane_ids.size(); ++index) {
    for (const auto lane_id : input.lane_ids[index]) {
      if (std::find(ids.begin(), ids.end(), lane_id) == ids.end()) {
        ids.push_back(lane_id);
      }
    }
  }
  return ids;
}

std::vector<CrosswalkPoint> polygon(CrosswalkBounds bounds) {
  std::reverse(bounds.right.begin(), bounds.right.end());
  bounds.left.insert(bounds.left.end(), bounds.right.begin(), bounds.right.end());
  return bounds.left;
}

CrosswalkBounds lanePrefix(CrosswalkBounds bounds, double distance_m) {
  const auto crop = [distance_m](std::vector<CrosswalkPoint> &points) {
    if (points.size() < 2) return;
    std::vector<CrosswalkPoint> selected{points.front()};
    double travelled = 0.0;
    for (std::size_t index = 1; index < points.size(); ++index) {
      const double step = std::hypot(points[index].x - points[index - 1].x,
                                     points[index].y - points[index - 1].y);
      if (travelled + step >= distance_m && step > 1.0e-9) {
        const double ratio = (distance_m - travelled) / step;
        selected.push_back({
            points[index - 1].x + ratio *
                (points[index].x - points[index - 1].x),
            points[index - 1].y + ratio *
                (points[index].y - points[index - 1].y)});
        break;
      }
      selected.push_back(points[index]);
      travelled += step;
    }
    points = std::move(selected);
  };
  crop(bounds.left);
  crop(bounds.right);
  return bounds;
}

bool lanePathIntersectsConflict(
    const Trajectory &path, std::size_t first, std::size_t last,
    const std::vector<CrosswalkBounds> &conflicts) {
  namespace bg = boost::geometry;
  using Point = bg::model::d2::point_xy<double>;
  using Polygon = bg::model::polygon<Point>;
  using Line = bg::model::linestring<Point>;
  Line route;
  for (std::size_t index = first; index <= last; ++index) {
    bg::append(route, Point(path[index].x, path[index].y));
  }
  if (route.size() < 2) return false;
  for (const auto &bounds : conflicts) {
    Polygon shape;
    for (const auto &point : polygon(bounds)) {
      bg::append(shape.outer(), Point(point.x, point.y));
    }
    bg::correct(shape);
    std::vector<Line> overlap;
    bg::intersection(route, shape, overlap);
    double inside_length = 0.0;
    for (const auto &segment : overlap) inside_length += bg::length(segment);
    // 경계에 스치기만 하는 경로는 충돌 구간으로 세지 않습니다.
    if (inside_length > 0.5) return true;
  }
  return false;
}

void appendUnique(std::vector<std::int64_t> &ids, std::int64_t id) {
  if (std::find(ids.begin(), ids.end(), id) == ids.end()) ids.push_back(id);
}

struct PathProjection {
  double ego_arc_m = 0.0;
  CrosswalkPoint entry;
};

PathProjection projectCrosswalkPath(
    const Trajectory &path, CrosswalkPoint ego, double entry_arc_m) {
  PathProjection projection;
  double nearest = std::numeric_limits<double>::infinity();
  double arc = 0.0;
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const double dx = path[index + 1].x - path[index].x;
    const double dy = path[index + 1].y - path[index].y;
    const double step = std::hypot(dx, dy);
    if (step <= 1.0e-9) continue;
    const double ratio = std::clamp(
        ((ego.x - path[index].x) * dx + (ego.y - path[index].y) * dy) /
            (step * step), 0.0, 1.0);
    const double distance = std::hypot(
        ego.x - path[index].x - ratio * dx,
        ego.y - path[index].y - ratio * dy);
    if (distance < nearest) {
      nearest = distance;
      projection.ego_arc_m = arc + ratio * step;
    }
    if (arc <= entry_arc_m && entry_arc_m <= arc + step) {
      const double entry_ratio = (entry_arc_m - arc) / step;
      projection.entry = {
          path[index].x + entry_ratio * dx,
          path[index].y + entry_ratio * dy};
    }
    arc += step;
  }
  return projection;
}

double distanceToSegment(CrosswalkPoint point, CrosswalkPoint start,
                         CrosswalkPoint end) {
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double length2 = dx * dx + dy * dy;
  if (length2 <= 1.0e-12) return std::hypot(point.x - start.x, point.y - start.y);
  const double ratio = std::clamp(
      ((point.x - start.x) * dx + (point.y - start.y) * dy) / length2,
      0.0, 1.0);
  return std::hypot(point.x - start.x - ratio * dx,
                    point.y - start.y - ratio * dy);
}

double distanceToPolygon(CrosswalkPoint point,
                         const std::vector<CrosswalkPoint> &shape) {
  bool inside = false;
  double nearest = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < shape.size(); ++index) {
    const auto first = shape[index];
    const auto second = shape[(index + 1) % shape.size()];
    nearest = std::min(nearest, distanceToSegment(point, first, second));
    if ((first.y > point.y) != (second.y > point.y)) {
      const double crossing_x = first.x +
          (point.y - first.y) * (second.x - first.x) /
              (second.y - first.y);
      if (point.x < crossing_x) inside = !inside;
    }
  }
  return inside ? 0.0 : nearest;
}

double distanceToPath(CrosswalkPoint point, const Trajectory &path) {
  double nearest = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    nearest = std::min(nearest, distanceToSegment(
        point, {path[index].x, path[index].y},
        {path[index + 1].x, path[index + 1].y}));
  }
  return nearest;
}

double projectArc(const Trajectory &path, CrosswalkPoint point) {
  double best = std::numeric_limits<double>::infinity();
  double selected = 0.0;
  double arc = 0.0;
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const double dx = path[index + 1].x - path[index].x;
    const double dy = path[index + 1].y - path[index].y;
    const double length2 = dx * dx + dy * dy;
    if (length2 <= 1.0e-12) continue;
    const double ratio = std::clamp(
        ((point.x - path[index].x) * dx +
         (point.y - path[index].y) * dy) / length2, 0.0, 1.0);
    const double distance = std::hypot(
        point.x - path[index].x - ratio * dx,
        point.y - path[index].y - ratio * dy);
    if (distance < best) {
      best = distance;
      selected = arc + ratio * std::sqrt(length2);
    }
    arc += std::sqrt(length2);
  }
  return selected;
}

// The original blind-spot module inserts its interpolated stop-line poses
// even when the safety decision is Safe.
void insertBlindSpotControlPoint(Trajectory &path, double target_arc_m) {
  if (path.size() < 2 || target_arc_m <= 0.0) return;
  double arc = 0.0;
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const double step = std::hypot(path[index + 1].x - path[index].x,
                                   path[index + 1].y - path[index].y);
    if (std::abs(target_arc_m - arc) < 0.001 ||
        std::abs(target_arc_m - arc - step) < 0.001) return;
    if (target_arc_m < arc + step) {
      auto point = resampleJerkAtArcs(path, {0.0, target_arc_m})[1];
      point.longitudinal_velocity_mps =
          path[index].longitudinal_velocity_mps;
      path.insert(path.begin() + index + 1, point);
      return;
    }
    arc += step;
  }
}

RunOutStopPoint runOutStopPointAtArc(
    const Trajectory &path, double stop_arc_m) {
  double traveled = 0.0;
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const auto &first = path[index];
    const auto &second = path[index + 1];
    const double length = std::hypot(
        second.x - first.x, second.y - first.y);
    if (length <= 1.0e-12) continue;
    if (traveled + length >= stop_arc_m) {
      const double ratio = std::clamp(
          (stop_arc_m - traveled) / length, 0.0, 1.0);
      return {first.x + ratio * (second.x - first.x),
              first.y + ratio * (second.y - first.y)};
    }
    traveled += length;
  }
  return {path.back().x, path.back().y};
}

}  // 익명 네임스페이스

BehaviorVelocityMapPlanner::BehaviorVelocityMapPlanner(
    const std::vector<std::uint8_t> &map_data,
    std::vector<BehaviorVelocityModule> modules,
    RunOutTemporalParameters run_out_temporal_parameters)
    : map_(map_data),
      run_out_temporal_(run_out_temporal_parameters),
      run_out_temporal_parameters_(run_out_temporal_parameters) {
  for (const auto module : modules) loadModule(module);
}

bool BehaviorVelocityMapPlanner::loadModule(BehaviorVelocityModule module) {
  switch (module) {
    case BehaviorVelocityModule::StopSign:
    case BehaviorVelocityModule::TrafficLight:
    case BehaviorVelocityModule::CrosswalkDefault:
    case BehaviorVelocityModule::Walkway:
    case BehaviorVelocityModule::IntersectionStuck:
    case BehaviorVelocityModule::IntersectionCollision:
    case BehaviorVelocityModule::DetectionArea:
    case BehaviorVelocityModule::SpeedBump:
    case BehaviorVelocityModule::NoDrivableLane:
    case BehaviorVelocityModule::NoStoppingArea:
    case BehaviorVelocityModule::MergeFromPrivate:
    case BehaviorVelocityModule::VirtualTrafficLight:
    case BehaviorVelocityModule::RunOut:
    case BehaviorVelocityModule::BlindSpot:
    case BehaviorVelocityModule::OcclusionSpot:
      break;
    default:
      return false;
  }
  if (std::find(modules_.begin(), modules_.end(), module) != modules_.end())
    return false;
  modules_.push_back(module);
  if (module == BehaviorVelocityModule::RunOut) {
    for (const auto &segment : map_.partitionSegments())
      run_out_map_partitions_.push_back({
          segment.from.x, segment.from.y,
          segment.to.x, segment.to.y});
  }
  if (module == BehaviorVelocityModule::OcclusionSpot)
    occlusion_spot_partitions_ = map_.partitionSegments();
  if (module == BehaviorVelocityModule::CrosswalkDefault) {
    for (const auto id : map_.crosswalkLaneletIds()) {
      auto bounds = map_.crosswalkBounds(id);
      auto shape = polygon(bounds);
      crosswalks_.push_back({id, std::move(bounds), std::move(shape),
                             map_.crosswalkTrafficLightIds(id)});
    }
  }
  if (module == BehaviorVelocityModule::Walkway) {
    for (const auto id : map_.walkwayLaneletIds()) {
      walkways_.try_emplace(id, polygon(map_.walkwayBounds(id)),
                            map_.walkwayStopLines(id));
    }
  }
  return true;
}

bool BehaviorVelocityMapPlanner::unloadModule(BehaviorVelocityModule module) {
  const auto found = std::find(modules_.begin(), modules_.end(), module);
  if (found == modules_.end()) return false;
  modules_.erase(found);
  switch (module) {
    case BehaviorVelocityModule::StopSign:
      stop_signs_.clear(); break;
    case BehaviorVelocityModule::TrafficLight:
      traffic_lights_.clear(); break;
    case BehaviorVelocityModule::CrosswalkDefault:
      crosswalks_.clear();
      crosswalk_object_states_.clear();
      crosswalk_occlusion_states_.clear();
      crosswalk_ego_stopped_since_s_.clear();
      break;
    case BehaviorVelocityModule::Walkway:
      walkways_.clear(); break;
    case BehaviorVelocityModule::IntersectionStuck:
      break;
    case BehaviorVelocityModule::IntersectionCollision:
      intersection_conflicts_.clear();
      intersection_collision_states_.clear();
      intersection_occlusion_states_.clear();
      intersection_occlusion_release_states_.clear();
      intersection_ego_stopped_since_s_.reset();
      break;
    case BehaviorVelocityModule::DetectionArea:
      detection_areas_.clear(); break;
    case BehaviorVelocityModule::SpeedBump:
      break;
    case BehaviorVelocityModule::NoDrivableLane:
      no_drivable_lanes_.clear(); break;
    case BehaviorVelocityModule::NoStoppingArea:
      no_stopping_areas_.clear();
      no_stopping_unstoppable_.clear();
      break;
    case BehaviorVelocityModule::MergeFromPrivate:
      merge_from_private_.clear(); break;
    case BehaviorVelocityModule::VirtualTrafficLight:
      virtual_traffic_lights_.clear(); break;
    case BehaviorVelocityModule::RunOut:
      run_out_temporal_ = RunOutTemporalCore(run_out_temporal_parameters_);
      run_out_object_filter_.reset();
      run_out_map_partitions_.clear();
      break;
    case BehaviorVelocityModule::BlindSpot:
      blind_spots_.clear(); break;
    case BehaviorVelocityModule::OcclusionSpot:
      occlusion_spot_partitions_.clear(); break;
  }
  return true;
}

const std::vector<CrosswalkBounds> &BehaviorVelocityMapPlanner::conflictsFor(
    std::int64_t lanelet_id) {
  auto found = intersection_conflicts_.find(lanelet_id);
  if (found == intersection_conflicts_.end()) {
    std::vector<CrosswalkBounds> shapes;
    for (const auto id : map_.intersectionConflictingLaneletIds(lanelet_id)) {
      shapes.push_back(map_.laneBounds(id));
    }
    found = intersection_conflicts_.emplace(lanelet_id, std::move(shapes)).first;
  }
  return found->second;
}

BehaviorVelocityMapResult BehaviorVelocityMapPlanner::apply(
    const BehaviorVelocityMapInput &input) {
  if (std::abs(input.ego_velocity_mps) < 1.0e-3) {
    if (!intersection_ego_stopped_since_s_ ||
        input.now_s < *intersection_ego_stopped_since_s_) {
      intersection_ego_stopped_since_s_ = input.now_s;
    }
  } else {
    intersection_ego_stopped_since_s_.reset();
  }
  const auto lane_ids = laneIdsAhead(input);
  for (auto it = merge_from_private_.begin();
       it != merge_from_private_.end();) {
    if (std::find(lane_ids.begin(), lane_ids.end(), it->first) ==
        lane_ids.end()) {
      it = merge_from_private_.erase(it);
    } else {
      ++it;
    }
  }
  bool road_has_turn_direction = false;
  bool road_has_traffic_light = false;
  if (!input.crosswalk_stuck_vehicles.empty()) {
    for (const auto lane_id : lane_ids) {
      road_has_turn_direction |= map_.laneTurnDirection(lane_id) !=
          LaneTurnDirection::Else;
      road_has_traffic_light |=
          !map_.trafficLightStopLinesForLanelet(lane_id).empty();
    }
  }
  BehaviorVelocityMapResult result;
  result.trajectory = input.trajectory;
  std::set<std::pair<std::int64_t, std::size_t>> seen_stop_signs;
  std::set<std::pair<std::int64_t, std::size_t>> seen_traffic_lights;
  std::set<std::int64_t> seen_crosswalks;
  std::set<std::int64_t> seen_intersections;
  std::set<std::int64_t> seen_detection_areas;
  std::set<std::int64_t> seen_no_stopping_areas;
  std::set<std::int64_t> seen_speed_bumps;
  std::set<std::int64_t> seen_no_drivable_lanes;
  std::set<std::int64_t> seen_virtual_traffic_lights;
  std::set<std::int64_t> seen_blind_spots;
  for (const auto module : modules_) {
    if (module == BehaviorVelocityModule::OcclusionSpot) {
      const auto planned = planOcclusionSpot(
          result.trajectory, {input.ego_x, input.ego_y},
          input.ego_velocity_mps, input.ego_acceleration_mps2,
          input.occlusion_spot_occupancy_grid,
          input.occlusion_spot_vehicles,
          input.occlusion_spot_parameters,
          occlusion_spot_partitions_);
      result.trajectory = planned.trajectory;
      result.occlusion_spot_collision_arcs_m =
          planned.collision_arcs_m;
      continue;
    }
    if (module == BehaviorVelocityModule::RunOut) {
      auto collision_arc = input.run_out_collision_arc_m;
      if (!collision_arc &&
          (!input.run_out_objects.empty() || !input.run_out_points.empty()) &&
          !result.trajectory.empty()) {
        const auto detection_path = prepareRunOutCollisionPath(
            result.trajectory, input.ego_x, input.ego_y,
            input.run_out_collision_parameters.vehicle_front_m,
            input.run_out_collision_parameters.detection_distance_m);
        const auto nearest = std::min_element(result.trajectory.begin(),
            result.trajectory.end(), [&](const auto &a, const auto &b) {
              return std::hypot(a.x - input.ego_x, a.y - input.ego_y) <
                     std::hypot(b.x - input.ego_x, b.y - input.ego_y);
            });
        const double ego_yaw = std::atan2(
            2.0 * (nearest->orientation_w * nearest->orientation_z +
                   nearest->orientation_x * nearest->orientation_y),
            1.0 - 2.0 * (nearest->orientation_y * nearest->orientation_y +
                         nearest->orientation_z * nearest->orientation_z));
        auto filter_parameters = input.run_out_object_filter_parameters;
        if (input.run_out_use_map_partitions &&
            filter_parameters.partition_segments.empty())
          filter_parameters.partition_segments = run_out_map_partitions_;
        std::vector<RunOutCollisionObject> candidates;
        if (input.run_out_detection_mode == RunOutDetectionMode::Object)
          candidates = input.run_out_objects;
        else if (input.run_out_detection_mode ==
                 RunOutDetectionMode::ObjectWithoutPath)
          candidates = createRunOutObjectsWithoutPath(
              result.trajectory, input.run_out_objects,
              input.run_out_creator_parameters);
        else
          candidates = createRunOutObjectsFromPoints(
              result.trajectory, input.run_out_points,
              input.run_out_detection_polygons,
              input.run_out_creator_parameters);
        const auto filtered = run_out_object_filter_.filter(
            detection_path, {input.ego_x, input.ego_y, ego_yaw},
            input.now_s, candidates,
            filter_parameters);
        result.run_out_collision = findRunOutCollision(
            detection_path, filtered,
            input.run_out_collision_parameters);
        if (result.run_out_collision)
          collision_arc = projectArc(result.trajectory,
              {result.run_out_collision->collision_x,
               result.run_out_collision->collision_y});
      }
      const bool detected = collision_arc && result.trajectory.size() >= 2;
      if (run_out_temporal_.acceptCollision(input.now_s, detected)) {
        RunOutBrakingInput brake_input;
        brake_input.collision_arc_m = *collision_arc;
        brake_input.ego_arc_m = projectArc(
            result.trajectory, {input.ego_x, input.ego_y});
        brake_input.velocity_mps = std::abs(input.ego_velocity_mps);
        brake_input.acceleration_mps2 = input.ego_acceleration_mps2;
        brake_input.object_detection = input.run_out_object_detection;
        const auto braking = planRunOutBraking(
            result.trajectory, brake_input,
            input.run_out_braking_parameters);
        result.trajectory = braking.trajectory;
        result.run_out_stop_arc_m = braking.stop_arc_m;
        result.run_out_deceleration_required =
            braking.deceleration_required;
        if (braking.stop_arc_m) {
          run_out_temporal_.recordStop(
              input.now_s, runOutStopPointAtArc(
                  result.trajectory, *braking.stop_arc_m));
        }
      }
      if (result.trajectory.size() >= 2) {
        const auto held = run_out_temporal_.maintainedStop(
            input.now_s, result.run_out_stop_arc_m.has_value());
        if (held) {
          const auto stopped = insertZeroVelocityAtArc(
              result.trajectory,
              projectArc(result.trajectory, {held->x, held->y}));
          result.trajectory = stopped.trajectory;
          result.run_out_stop_arc_m = stopped.stop_arc_m;
        }
      }
    } else if (module == BehaviorVelocityModule::BlindSpot) {
      for (const auto lane_id : lane_ids) {
        const auto frame = input.blind_spot_inputs.find(lane_id);
        if (frame == input.blind_spot_inputs.end() &&
            input.blind_spot_map_objects.empty() &&
            !input.blind_spot_auto_enabled) continue;
        if (!seen_blind_spots.insert(lane_id).second) continue;
        BlindSpotInput state_input;
        if (frame != input.blind_spot_inputs.end()) {
          state_input = frame->second;
        } else {
          std::vector<CrosswalkPoint> multi_lane_points;
          for (std::size_t index = 0;
               index < input.lane_ids.size() &&
               index < input.trajectory.size(); ++index) {
            if (input.lane_ids[index].size() > 1)
              multi_lane_points.push_back(
                  {input.trajectory[index].x, input.trajectory[index].y});
          }
          const auto stop_arcs = map_.blindSpotStopArcs(
              lane_id, result.trajectory, 3.79, 1.1, 0.948, 1.0,
              multi_lane_points);
          if (!stop_arcs || stop_arcs->default_stop_arc_m <= 0.0) continue;
          state_input.default_stop_arc_m = stop_arcs->default_stop_arc_m;
          state_input.critical_stop_arc_m = stop_arcs->critical_stop_arc_m;
          state_input.vehicle_front_m = 3.79;
          insertBlindSpotControlPoint(
              result.trajectory, stop_arcs->default_stop_arc_m);
          insertBlindSpotControlPoint(
              result.trajectory, stop_arcs->critical_stop_arc_m);
        }
        auto [core, inserted] = blind_spots_.try_emplace(
            lane_id, input.blind_spot_parameters);
        (void)inserted;
        state_input.now_s = input.now_s;
        state_input.ego_velocity_mps = input.ego_velocity_mps;
        state_input.ego_path_arc_m = projectArc(
            result.trajectory, {input.ego_x, input.ego_y});
        if (!input.blind_spot_map_objects.empty() &&
            state_input.objects.empty()) {
          std::vector<std::int64_t> preceding_lane_ids;
          bool reached_turn = false;
          for (const auto &point_lane_ids : input.lane_ids) {
            for (const auto point_lane_id : point_lane_ids) {
              if (point_lane_id == lane_id) {
                reached_turn = true;
                break;
              }
              if (std::find(preceding_lane_ids.begin(),
                            preceding_lane_ids.end(), point_lane_id) ==
                  preceding_lane_ids.end()) {
                preceding_lane_ids.push_back(point_lane_id);
              }
            }
            if (reached_turn) break;
          }
          const auto stop_point = runOutStopPointAtArc(
              result.trajectory, state_input.critical_stop_arc_m);
          const auto mapped = map_.mapBlindSpotObjects(
              preceding_lane_ids, map_.laneTurnDirection(lane_id),
              {input.ego_x, input.ego_y}, input.blind_spot_map_objects,
              100.0, CrosswalkPoint{stop_point.x, stop_point.y});
          if (mapped.stop_arc_m > 0.0) {
            state_input.ego_detection_lane_arc_m = mapped.ego_arc_m;
            state_input.stop_detection_lane_arc_m = mapped.stop_arc_m;
            state_input.objects = mapped.objects;
          }
        }
        const auto decision = core->second.evaluate(state_input);
        result.blind_spot_decisions.emplace(lane_id, decision);
        result.trajectory = applyBlindSpotDecision(
            result.trajectory, decision).trajectory;
      }
    } else if (module == BehaviorVelocityModule::VirtualTrafficLight) {
      for (const auto lane_id : lane_ids) {
        for (const auto &feature : map_.virtualTrafficLightsForLanelet(lane_id)) {
          if (!seen_virtual_traffic_lights.insert(feature.regulatory_id).second) continue;
          const auto arcs = findVirtualTrafficLightLineArcs(
              result.trajectory, feature.start_lines, feature.stop_lines,
              feature.end_lines);
          VirtualTrafficLightArcInput arc_input;
          arc_input.arcs = arcs;
          arc_input.ego_arc_m = projectArc(
              result.trajectory, {input.ego_x, input.ego_y});
          arc_input.front_offset_m =
              input.virtual_traffic_light_front_offset_m;
          arc_input.dead_line_margin_m =
              input.virtual_traffic_light_dead_line_margin_m;
          arc_input.near_line_distance_m =
              input.virtual_traffic_light_near_line_distance_m;
          arc_input.now_s = input.now_s;
          arc_input.has_stop_line = !feature.stop_lines.empty();
          arc_input.vehicle_stopped = std::abs(input.ego_velocity_mps) < 1.0e-3;
          const auto message = input.virtual_traffic_light_messages.find(
              feature.regulatory_id);
          if (message != input.virtual_traffic_light_messages.end()) {
            arc_input.message = message->second;
          }
          auto [core, inserted] = virtual_traffic_lights_.try_emplace(
              feature.regulatory_id,
              input.virtual_traffic_light_parameters);
          (void)inserted;
          const auto state_input =
              makeVirtualTrafficLightInputFromArcs(arc_input);
          const auto decision = core->second.apply(state_input);
          result.virtual_traffic_light_states.emplace(
              feature.regulatory_id, decision);
          const auto command = makeVirtualTrafficLightCommand(
              state_input, decision, feature.command_metadata);
          if (command)
            result.virtual_traffic_light_commands.emplace(
                feature.regulatory_id, *command);
          VirtualTrafficLightPathInput path_input;
          path_input.trajectory = result.trajectory;
          path_input.decision = decision;
          path_input.stop_line_arc_m = arcs.stop_arc_m;
          path_input.end_line_arc_m = arcs.end_arc_m;
          path_input.ego_arc_m = arc_input.ego_arc_m;
          path_input.front_offset_m = arc_input.front_offset_m;
          path_input.hold_stop_margin_m =
              input.virtual_traffic_light_hold_stop_margin_m;
          path_input.vehicle_stopped = arc_input.vehicle_stopped;
          path_input.before_stop_line = state_input.before_stop_line;
          result.trajectory = applyVirtualTrafficLightStopVelocity(
              path_input).trajectory;
        }
      }
    } else if (module == BehaviorVelocityModule::StopSign) {
      for (const auto lane_id : lane_ids) {
        const auto lines = map_.stopSignStopLinesForLanelet(lane_id);
        for (std::size_t index = 0; index < lines.size(); ++index) {
          const auto &line = lines[index];
          const auto key = std::make_pair(line.regulatory_id, index);
          if (!seen_stop_signs.insert(key).second) continue;
          auto [core, inserted] = stop_signs_.try_emplace(key, line.segment);
          (void)inserted;
          result.trajectory = core->second.apply(
              result.trajectory, input.ego_x, input.ego_y,
              input.ego_velocity_mps < 1.0e-3, input.now_s).trajectory;
          appendUnique(result.stop_sign_regulatory_ids, line.regulatory_id);
        }
      }
    } else if (module == BehaviorVelocityModule::NoDrivableLane) {
      for (const auto lane_id : lane_ids) {
        if (!map_.isNoDrivableLane(lane_id) ||
            !seen_no_drivable_lanes.insert(lane_id).second) continue;
        std::vector<SpeedBumpPoint> shape;
        for (const auto &point : polygon(map_.laneBounds(lane_id))) {
          shape.push_back({point.x, point.y});
        }
        auto [core, inserted] = no_drivable_lanes_.try_emplace(
            lane_id, std::move(shape));
        (void)inserted;
        const auto stopped = core->second.apply(
            result.trajectory, input.ego_x, input.ego_y,
            input.ego_velocity_mps < 1.0e-3);
        result.trajectory = stopped.trajectory;
        if (stopped.stopped) {
          appendUnique(result.no_drivable_lanelet_ids, lane_id);
        }
      }
    } else if (module == BehaviorVelocityModule::SpeedBump) {
      for (const auto lane_id : lane_ids) {
        for (const auto &feature : map_.speedBumpsForLanelet(lane_id)) {
          if (!seen_speed_bumps.insert(feature.regulatory_id).second) continue;
          const auto slowed = planSpeedBump(result.trajectory, feature);
          result.trajectory = slowed.trajectory;
          if (slowed.slowed) {
            appendUnique(result.speed_bump_regulatory_ids,
                         feature.regulatory_id);
          }
        }
      }
    } else if (module == BehaviorVelocityModule::DetectionArea) {
      for (const auto lane_id : lane_ids) {
        for (auto feature : map_.detectionAreasForLanelet(lane_id)) {
          if (!seen_detection_areas.insert(feature.regulatory_id).second) continue;
          auto [core, inserted] = detection_areas_.try_emplace(
              feature.regulatory_id, feature.stop_line,
              std::move(feature.areas));
          (void)inserted;
          const auto stopped = core->second.apply(
              result.trajectory, input.ego_x, input.ego_y,
              input.now_s, input.detection_area_obstacle_points);
          result.trajectory = stopped.trajectory;
          if (stopped.stopped) {
            appendUnique(result.detection_area_regulatory_ids,
                         feature.regulatory_id);
          }
        }
      }
    } else if (module == BehaviorVelocityModule::NoStoppingArea) {
      for (const auto lane_id : lane_ids) {
        for (auto feature : map_.noStoppingAreasForLanelet(lane_id)) {
          if (!seen_no_stopping_areas.insert(feature.regulatory_id).second) {
            continue;
          }
          std::vector<CrosswalkPoint> stopped_vehicles;
          for (const auto &vehicle : input.no_stopping_vehicles) {
            for (const double value : {
                     vehicle.position.x, vehicle.position.y,
                     vehicle.yaw_rad, vehicle.length_m,
                     vehicle.width_m, vehicle.speed_mps}) {
              if (!std::isfinite(value)) {
                throw std::invalid_argument(
                    "정차 금지 구역 차량 값에는 유한한 값이 필요합니다");
              }
            }
            if (vehicle.length_m <= 0.0 || vehicle.width_m <= 0.0) {
              throw std::invalid_argument(
                  "정차 금지 구역 차량 크기가 잘못되었습니다");
            }
            if (vehicle.target_vehicle && vehicle.classification >= 1 &&
                vehicle.classification <= 5 &&
                std::abs(vehicle.speed_mps) <= 0.833) {
              for (const auto &area : feature.areas) {
                const auto overlap = noStoppingOverlapPoint(
                    area, vehicle, result.trajectory);
                if (overlap) stopped_vehicles.push_back(*overlap);
              }
            }
          }
          for (const auto &area : feature.areas) {
            if (!noStoppingPathStopInArea(
                    result.trajectory, area, input.ego_x, input.ego_y)) {
              continue;
            }
            CrosswalkPoint center;
            for (const auto &point : area) {
              center.x += point.x;
              center.y += point.y;
            }
            center.x /= area.size();
            center.y /= area.size();
            stopped_vehicles.push_back(center);
          }
          const auto stop_line = feature.stop_line
              ? feature.stop_line
              : noStoppingGeneratedStopLine(result.trajectory,
                                            feature.areas);
          if (!stop_line) continue;
          if (no_stopping_unstoppable_.count(feature.regulatory_id)) continue;
          StopLineParameters judge_parameters;
          judge_parameters.linear_insert = true;
          judge_parameters.copy_previous_orientation = true;
          StopLineCore judge_line(*stop_line, judge_parameters);
          const auto judge = judge_line.apply(
              result.trajectory, input.ego_x, input.ego_y, false,
              input.now_s);
          if (judge.stop_arc_m && input.ego_velocity_mps >= 2.0 &&
              noStoppingBrakeDistance(input.ego_velocity_mps,
                                      input.ego_acceleration_mps2) >=
                  *judge.stop_arc_m - judge.ego_arc_m) {
            no_stopping_unstoppable_.insert(feature.regulatory_id);
            continue;
          }
          DetectionAreaParameters parameters;
          parameters.distance_to_judge_over_stop_line_m = 1.0;
          parameters.linear_insert = true;
          parameters.copy_previous_orientation = true;
          auto [core, inserted] = no_stopping_areas_.try_emplace(
              feature.regulatory_id, *stop_line,
              std::move(feature.areas), parameters);
          (void)inserted;
          const auto stopped = core->second.apply(
              result.trajectory, input.ego_x, input.ego_y,
              input.now_s, stopped_vehicles);
          result.trajectory = stopped.trajectory;
          if (stopped.stopped) {
            appendUnique(result.no_stopping_area_regulatory_ids,
                         feature.regulatory_id);
          }
        }
      }
    } else if (module == BehaviorVelocityModule::TrafficLight) {
      for (const auto lane_id : lane_ids) {
        const auto lines = map_.trafficLightStopLinesForLanelet(lane_id);
        for (std::size_t index = 0; index < lines.size(); ++index) {
          const auto &line = lines[index];
          const auto key = std::make_pair(line.regulatory_id, index);
          if (!seen_traffic_lights.insert(key).second) continue;
          TrafficLightParameters light_parameters;
          light_parameters.simulation = input.is_simulation;
          auto [core, inserted] = traffic_lights_.try_emplace(
              key, line.segment, light_parameters);
          (void)inserted;
          TrafficLightInput light_input;
          light_input.ego_x = input.ego_x;
          light_input.ego_y = input.ego_y;
          light_input.ego_velocity_mps = input.ego_velocity_mps;
          light_input.ego_acceleration_mps2 = input.ego_acceleration_mps2;
          light_input.now_s = input.now_s;
          light_input.signal_stamp_s = input.signal_stamp_s;
          light_input.signal_requires_stop = input.signal_requires_stop;
          const auto signal = input.traffic_signal_observations.find(
              line.regulatory_id);
          if (signal != input.traffic_signal_observations.end()) {
            light_input.signal_stamp_s = signal->second.stamp_s;
            light_input.signal_requires_stop = trafficSignalRequiresStop(
                map_.laneTurnDirection(lane_id), signal->second.elements);
          }
          result.trajectory = core->second.apply(result.trajectory, light_input).trajectory;
          appendUnique(result.traffic_light_regulatory_ids, line.regulatory_id);
        }
      }
    } else if (module == BehaviorVelocityModule::Walkway) {
      for (auto &[id, core] : walkways_) {
        const auto output = core.apply(
            result.trajectory, input.ego_x, input.ego_y,
            input.ego_velocity_mps < 1.0e-3, input.now_s);
        if (!output.stop_arc_m && core.state() == WalkwayState::Approach) continue;
        result.trajectory = output.trajectory;
        appendUnique(result.walkway_lanelet_ids, id);
      }
    } else if (module == BehaviorVelocityModule::IntersectionStuck) {
      std::set<std::int64_t> handled_conflicts;
      const auto intersection_count = std::count_if(
          lane_ids.begin(), lane_ids.end(), [&](std::int64_t id) {
            const auto turn = map_.laneTurnDirection(id);
            return turn == LaneTurnDirection::Left ||
                   turn == LaneTurnDirection::Right;
          });
      for (const auto lane_id : lane_ids) {
        const auto direction = map_.laneTurnDirection(lane_id);
        if (direction != LaneTurnDirection::Left &&
            direction != LaneTurnDirection::Right) continue;
        std::vector<CrosswalkBounds> relevant_conflicts;
        std::vector<std::vector<CrosswalkPoint>> relevant_centerlines;
        for (const auto conflict_id :
             map_.intersectionConflictingLaneletIds(lane_id)) {
          if (!handled_conflicts.insert(conflict_id).second) continue;
          relevant_conflicts.push_back(map_.laneBounds(conflict_id));
          relevant_centerlines.push_back(map_.laneCenterline(conflict_id));
        }
        IntersectionStuckParameters stuck_settings;
        stuck_settings.detection_lanes.push_back(map_.laneBounds(lane_id));
        stuck_settings.insert_control_points = true;
        stuck_settings.ego_velocity_mps = input.ego_velocity_mps;
        stuck_settings.ego_acceleration_mps2 = input.ego_acceleration_mps2;
        std::optional<std::size_t> first_index;
        std::optional<std::size_t> last_index;
        for (std::size_t index = 0; index < input.lane_ids.size(); ++index) {
          const auto &ids = input.lane_ids[index];
          if (std::find(ids.begin(), ids.end(), lane_id) == ids.end()) continue;
          if (!first_index) first_index = index;
          last_index = index;
        }
        if (!first_index || !last_index) continue;
        stuck_settings.lane_entry_arc_m = projectArc(
            result.trajectory,
            {input.trajectory[*first_index].x,
             input.trajectory[*first_index].y});
        for (std::size_t index = 0; index < input.lane_ids.size(); ++index) {
          if (input.lane_ids[index].size() < 2) continue;
          stuck_settings.multi_lane_anchor_arcs_m.push_back(projectArc(
              result.trajectory,
              {input.trajectory[index].x, input.trajectory[index].y}));
        }
        // 연속 교차로에서는 뒤쪽 차로가 안전해도 원본이 기준점을 먼저 삽입합니다.
        if (intersection_count > 1) {
          auto prepared_settings = stuck_settings;
          for (const auto conflict_id :
               map_.intersectionConflictingLaneletIds(lane_id)) {
            prepared_settings.conflict_centerlines.push_back(
                map_.laneCenterline(conflict_id));
          }
          result.trajectory = planIntersectionStuckStop(
              result.trajectory, conflictsFor(lane_id), {},
              input.ego_x, input.ego_y, prepared_settings).trajectory;
        }
        // 원본은 차로의 주목 영역에서 나오는 경로점까지 있어야 감지 구역을 만듭니다.
        if (*last_index + 1 >= input.trajectory.size() ||
            relevant_conflicts.empty() ||
            !lanePathIntersectsConflict(
                input.trajectory, *first_index, *last_index,
                relevant_conflicts)) continue;
        if (intersection_count > 1) {
          for (std::size_t index = *last_index + 1;
               index < input.lane_ids.size(); ++index) {
            const auto &ids = input.lane_ids[index];
            const auto next = std::find_if(ids.begin(), ids.end(),
                [lane_id](std::int64_t id) { return id != lane_id; });
            if (next == ids.end()) continue;
            stuck_settings.detection_lanes.push_back(
                lanePrefix(map_.laneBounds(*next), 5.0));
            stuck_settings.maximum_detection_arc_m = projectArc(
                result.trajectory,
                {input.trajectory[*last_index].x,
                 input.trajectory[*last_index].y}) + 5.0;
            break;
          }
        }
        stuck_settings.conflict_centerlines =
            std::move(relevant_centerlines);
        const auto stopped = planIntersectionStuckStop(
            result.trajectory, relevant_conflicts,
            input.intersection_stuck_vehicles,
            input.ego_x, input.ego_y, stuck_settings);
        result.trajectory = stopped.trajectory;
        if (!stopped.stop_arc_m) continue;
        appendUnique(result.intersection_stuck_lanelet_ids, lane_id);
      }
    } else if (module == BehaviorVelocityModule::IntersectionCollision) {
      for (const auto lane_id : lane_ids) {
        if (map_.laneTurnDirection(lane_id) == LaneTurnDirection::Else) continue;
        seen_intersections.insert(lane_id);
        std::optional<CrosswalkPoint> entry;
        for (std::size_t index = 0; index < input.lane_ids.size(); ++index) {
          const auto &ids = input.lane_ids[index];
          if (std::find(ids.begin(), ids.end(), lane_id) != ids.end()) {
            entry = {input.trajectory[index].x, input.trajectory[index].y};
            break;
          }
        }
        if (!entry) continue;
        IntersectionStuckParameters stop_settings;
        stop_settings.insert_control_points = true;
        stop_settings.ego_velocity_mps = input.ego_velocity_mps;
        stop_settings.ego_acceleration_mps2 = input.ego_acceleration_mps2;
        const double entry_arc = projectArc(result.trajectory, *entry);
        stop_settings.lane_entry_arc_m = entry_arc;
        for (std::size_t index = 0; index < input.lane_ids.size(); ++index) {
          if (input.lane_ids[index].size() < 2) continue;
          stop_settings.multi_lane_anchor_arcs_m.push_back(projectArc(
              result.trajectory,
              {input.trajectory[index].x, input.trajectory[index].y}));
        }
        for (const auto conflict_id :
             map_.intersectionConflictingLaneletIds(lane_id)) {
          stop_settings.conflict_centerlines.push_back(
              map_.laneCenterline(conflict_id));
        }
        const auto prepared = planIntersectionStuckStop(
            result.trajectory, conflictsFor(lane_id), {},
            input.ego_x, input.ego_y, stop_settings);
        result.trajectory = prepared.trajectory;
        auto vehicles = input.intersection_predicted_vehicles;
        for (auto &vehicle : vehicles) {
          if (!vehicle.target_vehicle || vehicle.predicted_poses.empty()) continue;
          const auto &pose = vehicle.initial_pose
              ? *vehicle.initial_pose : vehicle.predicted_poses.front();
          vehicle.target_vehicle = map_.isVehicleOnIntersectionAttentionLane(
              lane_id, pose.x, pose.y, pose.yaw_rad);
          if (vehicle.target_vehicle) {
            const auto distances =
                map_.intersectionTrafficControlDistances(
                    lane_id, pose.x, pose.y, pose.yaw_rad);
            if (distances) {
              vehicle.distance_to_stopline_m =
                  distances->distance_to_stopline_m;
              vehicle.stopline_to_ego_path_m =
                  distances->stopline_to_ego_path_m;
            }
          }
        }
        IntersectionCollisionParameters collision_settings;
        collision_settings.ego_velocity_mps = input.ego_velocity_mps;
        collision_settings.ego_acceleration_mps2 = input.ego_acceleration_mps2;
        for (const auto light_id :
             map_.intersectionObservedTrafficLightIds(lane_id)) {
          const auto found = input.intersection_signal_colors.find(light_id);
          if (found != input.intersection_signal_colors.end() &&
              found->second == TrafficLampColor::Red) {
            collision_settings.fully_prioritized = true;
            collision_settings.collision_start_margin_s = 2.0;
            collision_settings.collision_end_margin_s = 0.0;
            break;
          } else if (found != input.intersection_signal_colors.end() &&
                     found->second == TrafficLampColor::Yellow) {
            collision_settings.partially_prioritized = true;
          }
        }
        const auto collision = detectIntersectionCollision(
            result.trajectory, input.ego_x, input.ego_y,
            entry_arc, vehicles, collision_settings);
        auto [state, inserted] = intersection_collision_states_.try_emplace(lane_id);
        (void)inserted;
        const bool collision_stop =
            state->second.update(collision.collision, input.now_s);
        bool occlusion_stop = false;
        IntersectionOcclusionStatus occlusion_status =
            IntersectionOcclusionStatus::Clear;
        if (input.intersection_occlusion_enabled &&
            input.intersection_occupancy_grid &&
            !collision_settings.fully_prioritized &&
            !collision_settings.partially_prioritized) {
          const auto light_ids =
              map_.intersectionObservedTrafficLightIds(lane_id);
          const bool has_green_light = std::any_of(
              light_ids.begin(), light_ids.end(), [&](std::int64_t id) {
                const auto signal = input.intersection_signal_colors.find(id);
                return signal != input.intersection_signal_colors.end() &&
                       signal->second == TrafficLampColor::Green;
              });
          if (has_green_light || !map_.intersectionHasTrafficLight(lane_id)) {
            std::vector<std::vector<CrosswalkPoint>> areas;
            for (const auto attention_id :
                 map_.intersectionOcclusionLaneletIds(lane_id)) {
              areas.push_back(polygon(map_.laneBounds(attention_id)));
            }
            const IntersectionOcclusionParameters occlusion_parameters;
            std::vector<IntersectionOcclusionBlocker> attention_vehicles;
            for (const auto &vehicle : input.intersection_predicted_vehicles) {
              if (!vehicle.target_vehicle || vehicle.classification < 1 ||
                  vehicle.classification > 5) continue;
              const auto pose = vehicle.initial_pose
                  ? vehicle.initial_pose
                  : (vehicle.predicted_poses.empty()
                         ? std::optional<CrosswalkPose2d>{}
                         : std::optional<CrosswalkPose2d>{
                               vehicle.predicted_poses.front()});
              if (!pose || !map_.isVehicleOnIntersectionAttentionLane(
                               lane_id, pose->x, pose->y, pose->yaw_rad,
                               0.0, 3.141592653589793)) continue;
              attention_vehicles.push_back({
                  *pose, vehicle.length_m, vehicle.width_m,
                  vehicle.initial_speed_mps.value_or(
                      occlusion_parameters.parked_vehicle_speed_threshold_mps)});
            }
            occlusion_status = classifyIntersectionOcclusion(
                *input.intersection_occupancy_grid, areas,
                result.trajectory, {input.ego_x, input.ego_y},
                occlusion_parameters,
                map_.intersectionOcclusionDivisions(
                    lane_id,
                    input.intersection_occupancy_grid->resolution_m),
                attention_vehicles);
            occlusion_stop = occlusion_status !=
                IntersectionOcclusionStatus::Clear;
          }
        }
        auto [occlusion_state, occlusion_inserted] =
            intersection_occlusion_states_.try_emplace(lane_id, 1.5);
        (void)occlusion_inserted;
        if (collision_settings.fully_prioritized ||
            collision_settings.partially_prioritized ||
            !input.intersection_occlusion_enabled ||
            !input.intersection_occupancy_grid) {
          occlusion_state->second = IntersectionCollisionState(1.5);
        } else {
          occlusion_stop = occlusion_state->second.update(
              occlusion_stop, input.now_s);
        }
        const auto stoplines = input.intersection_occlusion_stoplines.find(lane_id);
        if (stoplines != input.intersection_occlusion_stoplines.end() &&
            input.intersection_occlusion_enabled &&
            input.intersection_occupancy_grid) {
          const auto &lines = stoplines->second;
          if (!std::isfinite(lines.default_arc_m) ||
              !std::isfinite(lines.peeking_arc_m) ||
              lines.default_arc_m < 0.0 ||
              lines.peeking_arc_m < lines.default_arc_m) {
            throw std::invalid_argument("교차로 가림 정지선 위치가 잘못되었습니다");
          }
          const double ego_arc = projectArc(result.trajectory,
              {input.ego_x, input.ego_y});
          const bool stopped_for_peeking =
              intersection_ego_stopped_since_s_ &&
              input.now_s - *intersection_ego_stopped_since_s_ >= 0.1;
          const bool stopped_at_default =
              ego_arc > lines.default_arc_m ||
              (stopped_for_peeking &&
               std::abs(ego_arc - lines.default_arc_m) < 0.5);
          const bool stopped_at_peeking =
              ego_arc > lines.peeking_arc_m + 0.5 ||
              (stopped_for_peeking &&
               std::abs(ego_arc - lines.peeking_arc_m) < 0.5);
          auto &release = intersection_occlusion_release_states_[lane_id];
          if (release.update(
                  occlusion_status, map_.intersectionHasTrafficLight(lane_id),
                  collision_stop, stopped_at_default,
                  stopped_at_peeking, input.now_s)) {
            occlusion_stop = false;
          }
        } else {
          intersection_occlusion_release_states_.erase(lane_id);
        }
        if (!collision_stop && !occlusion_stop) continue;
        stop_settings.insert_control_points = false;
        const auto stopped = planIntersectionConflictStop(
            result.trajectory, conflictsFor(lane_id),
            input.ego_x, input.ego_y, stop_settings);
        if (!stopped.stop_arc_m) continue;
        result.trajectory = stopped.trajectory;
        if (collision_stop) {
          appendUnique(result.intersection_collision_lanelet_ids, lane_id);
        }
        if (occlusion_stop) {
          appendUnique(result.intersection_occlusion_lanelet_ids, lane_id);
          if (occlusion_status != IntersectionOcclusionStatus::Clear) {
            result.intersection_occlusion_statuses[lane_id] = occlusion_status;
          }
        }
      }
    } else if (module == BehaviorVelocityModule::MergeFromPrivate) {
      for (std::size_t lane_index = 0; lane_index < lane_ids.size();
           ++lane_index) {
        const auto lane_id = lane_ids[lane_index];
        if (!map_.isPrivateLane(lane_id) ||
            map_.laneTurnDirection(lane_id) == LaneTurnDirection::Else) continue;
        bool merging_to_public_road = lane_index + 1 < lane_ids.size()
            ? !map_.isPrivateLane(lane_ids[lane_index + 1])
            : false;
        if (lane_index + 1 == lane_ids.size()) {
          for (const auto conflict_id :
               map_.intersectionConflictingLaneletIds(lane_id)) {
            if (map_.isUrbanLane(conflict_id)) {
              merging_to_public_road = true;
              break;
            }
          }
        }
        if (!merging_to_public_road) continue;
        std::optional<CrosswalkPoint> entry;
        for (std::size_t index = 0; index < input.lane_ids.size(); ++index) {
          const auto &ids = input.lane_ids[index];
          if (std::find(ids.begin(), ids.end(), lane_id) != ids.end()) {
            entry = {input.trajectory[index].x, input.trajectory[index].y};
            break;
          }
        }
        if (!entry) continue;
        auto [core, inserted] = merge_from_private_.try_emplace(lane_id);
        (void)inserted;
        const auto stopped = core->second.apply(
            result.trajectory, conflictsFor(lane_id),
            input.ego_x, input.ego_y, input.ego_velocity_mps,
            input.now_s,
            std::max(0.0, projectArc(result.trajectory, *entry) - 3.79));
        if (!stopped.stopped) continue;
        result.trajectory = stopped.trajectory;
        appendUnique(result.merge_from_private_lanelet_ids, lane_id);
      }
    } else if (module == BehaviorVelocityModule::CrosswalkDefault) {
      for (const auto &feature : crosswalks_) {
        const auto crosswalk_id = feature.lanelet_id;
        const auto &shape = feature.polygon;
        const auto crossing = planCrosswalkDefaultStop(result.trajectory, shape);
        if (!crossing.stop_arc_m || !crossing.entry_arc_m ||
            !crossing.exit_arc_m) continue;
        seen_crosswalks.insert(crosswalk_id);
        if (input.ego_velocity_mps < 1.0e-3) {
          crosswalk_ego_stopped_since_s_.try_emplace(crosswalk_id, input.now_s);
        } else {
          crosswalk_ego_stopped_since_s_.erase(crosswalk_id);
        }
        const auto projection = projectCrosswalkPath(
            result.trajectory, {input.ego_x, input.ego_y},
            *crossing.entry_arc_m);
        const auto stopped_since = crosswalk_ego_stopped_since_s_.find(crosswalk_id);
        const bool ego_yielding = stopped_since != crosswalk_ego_stopped_since_s_.end() &&
            input.now_s - stopped_since->second >= 1.0 &&
            *crossing.stop_arc_m - projection.ego_arc_m < 1.0;
        auto &tracker = crosswalk_object_states_[crosswalk_id];
        std::vector<CrosswalkPedestrianSignal> signals;
        for (const auto light_id : feature.traffic_light_ids) {
          const auto found = input.pedestrian_signals.find(light_id);
          if (found != input.pedestrian_signals.end()) signals.push_back(found->second);
        }
        tracker.beginFrame(signals, input.now_s);
        if (pedestriansHaveRedSignal(signals, input.now_s)) {
          tracker.endFrame();
          continue;
        }
        CrosswalkResult chosen = crossing;
        if (input.crosswalk_enable_rtc ||
            input.crosswalk_auto_activate_when_safe) {
          chosen.trajectory = result.trajectory;
          chosen.stop_arc_m.reset();
          chosen.stop_point.reset();
        }
        if (!input.crosswalk_stuck_vehicles.empty()) {
          const auto stuck = planCrosswalkStuckVehicleStop(
              result.trajectory, shape, {input.ego_x, input.ego_y},
              input.ego_velocity_mps, input.ego_acceleration_mps2,
              input.crosswalk_stuck_vehicles, road_has_turn_direction,
              road_has_traffic_light, {}, input.crosswalk_stuck_parameters);
          if (stuck.stop_arc_m &&
              (!chosen.stop_arc_m || *stuck.stop_arc_m < *chosen.stop_arc_m)) {
            chosen = stuck;
          }
        }
        for (const auto &object : input.crosswalk_objects) {
          if (object.classification != motorcycle_classification &&
              object.classification != bicycle_classification &&
              object.classification != pedestrian_classification) continue;
          const auto collision = estimateCrosswalkCollision(
              result.trajectory, {input.ego_x, input.ego_y},
              *crossing.entry_arc_m, *crossing.exit_arc_m,
              input.ego_velocity_mps, object.predicted);
          CrosswalkObjectObservation observation;
          observation.now_s = input.now_s;
          observation.speed_mps = std::hypot(
              object.predicted.velocity_x_mps, object.predicted.velocity_y_mps);
          observation.distance_to_crosswalk_m =
              distanceToPolygon(object.position, shape);
          observation.ego_yielding = ego_yielding;
          observation.away_from_attention_area =
              distanceToPath(object.position, result.trajectory) > 3.5;
          observation.crosswalk_has_traffic_light = !signals.empty();
          observation.bicycle_or_motorcycle =
              object.classification == motorcycle_classification ||
              object.classification == bicycle_classification;
          if (collision) {
            observation.collision_times_s = std::make_pair(
                collision->time_to_collision_s, collision->time_to_vehicle_s);
          }
          const auto state = tracker.update(object.id, observation);
          result.crosswalk_object_states[crosswalk_id][object.id] = state;
          if (!collision || state != CrosswalkCollisionState::Yield) {
            continue;
          }
          const auto stopped = planCrosswalkObjectStopWithDynamics(
              result.trajectory, shape, {input.ego_x, input.ego_y}, *collision,
              input.ego_velocity_mps, input.ego_acceleration_mps2);
          if (stopped.stop_arc_m &&
              (!chosen.stop_arc_m || *stopped.stop_arc_m <= *chosen.stop_arc_m)) {
            chosen = stopped;
          }
        }
        tracker.endFrame();
        if (input.crosswalk_occlusion_enabled && input.crosswalk_occupancy_grid) {
          auto [occlusion, inserted] = crosswalk_occlusion_states_.try_emplace(
              crosswalk_id, input.crosswalk_occlusion_parameters);
          (void)inserted;
          const double detection_range = 0.948 + crosswalkDetectionRange(
              input.crosswalk_occlusion_parameters.assumed_object_speed_mps,
              *crossing.entry_arc_m - projection.ego_arc_m,
              input.ego_velocity_mps);
          const auto areas = crosswalkDetectionAreas(
              feature.bounds.left, feature.bounds.right,
              projection.entry, detection_range);
          const bool occluded = isCrosswalkOccluded(
              *input.crosswalk_occupancy_grid, areas,
              input.crosswalk_occlusion_parameters,
              input.crosswalk_occlusion_mask_objects);
          if (occlusion->second.update(
                  input.now_s, occluded, true, !signals.empty(), false,
                  projection.ego_arc_m >= *crossing.entry_arc_m)) {
            chosen.trajectory = occlusion->second.applySlowdown(
                chosen.trajectory, *crossing.entry_arc_m, *crossing.exit_arc_m,
                projection.ego_arc_m, input.ego_velocity_mps,
                input.ego_acceleration_mps2);
            appendUnique(result.crosswalk_lanelet_ids, crosswalk_id);
          }
        }
        if (chosen.stop_arc_m) {
          appendUnique(result.crosswalk_lanelet_ids, crosswalk_id);
        }
        result.trajectory = std::move(chosen.trajectory);
      }
    }
  }
  if (std::find(modules_.begin(), modules_.end(),
                BehaviorVelocityModule::StopSign) != modules_.end()) {
    for (auto it = stop_signs_.begin(); it != stop_signs_.end();) {
      it = seen_stop_signs.count(it->first) ? std::next(it) : stop_signs_.erase(it);
    }
  }
  if (std::find(modules_.begin(), modules_.end(),
                BehaviorVelocityModule::VirtualTrafficLight) != modules_.end()) {
    for (auto it = virtual_traffic_lights_.begin();
         it != virtual_traffic_lights_.end();) {
      it = seen_virtual_traffic_lights.count(it->first)
          ? std::next(it) : virtual_traffic_lights_.erase(it);
    }
  }
  if (std::find(modules_.begin(), modules_.end(),
                BehaviorVelocityModule::BlindSpot) != modules_.end()) {
    for (auto it = blind_spots_.begin(); it != blind_spots_.end();) {
      it = seen_blind_spots.count(it->first) ? std::next(it) : blind_spots_.erase(it);
    }
  }
  if (std::find(modules_.begin(), modules_.end(),
                BehaviorVelocityModule::DetectionArea) != modules_.end()) {
    for (auto it = detection_areas_.begin(); it != detection_areas_.end();) {
      it = seen_detection_areas.count(it->first)
          ? std::next(it) : detection_areas_.erase(it);
    }
  }
  if (std::find(modules_.begin(), modules_.end(),
                BehaviorVelocityModule::NoDrivableLane) != modules_.end()) {
    for (auto it = no_drivable_lanes_.begin(); it != no_drivable_lanes_.end();) {
      it = seen_no_drivable_lanes.count(it->first)
          ? std::next(it) : no_drivable_lanes_.erase(it);
    }
  }
  if (std::find(modules_.begin(), modules_.end(),
                BehaviorVelocityModule::TrafficLight) != modules_.end()) {
    for (auto it = traffic_lights_.begin(); it != traffic_lights_.end();) {
      it = seen_traffic_lights.count(it->first) ? std::next(it)
                                                  : traffic_lights_.erase(it);
    }
  }
  if (std::find(modules_.begin(), modules_.end(),
                BehaviorVelocityModule::CrosswalkDefault) != modules_.end()) {
    for (auto it = crosswalk_object_states_.begin();
         it != crosswalk_object_states_.end();) {
      it = seen_crosswalks.count(it->first) ? std::next(it)
                                           : crosswalk_object_states_.erase(it);
    }
    for (auto it = crosswalk_occlusion_states_.begin();
         it != crosswalk_occlusion_states_.end();) {
      it = seen_crosswalks.count(it->first) ? std::next(it)
                                           : crosswalk_occlusion_states_.erase(it);
    }
    for (auto it = crosswalk_ego_stopped_since_s_.begin();
         it != crosswalk_ego_stopped_since_s_.end();) {
      it = seen_crosswalks.count(it->first) ? std::next(it)
                                           : crosswalk_ego_stopped_since_s_.erase(it);
    }
  }
  if (std::find(modules_.begin(), modules_.end(),
                BehaviorVelocityModule::IntersectionCollision) != modules_.end()) {
    for (auto it = intersection_collision_states_.begin();
         it != intersection_collision_states_.end();) {
      it = seen_intersections.count(it->first)
          ? std::next(it) : intersection_collision_states_.erase(it);
    }
    for (auto it = intersection_occlusion_states_.begin();
         it != intersection_occlusion_states_.end();) {
      it = seen_intersections.count(it->first)
          ? std::next(it) : intersection_occlusion_states_.erase(it);
    }
    for (auto it = intersection_occlusion_release_states_.begin();
         it != intersection_occlusion_release_states_.end();) {
      it = seen_intersections.count(it->first)
          ? std::next(it) : intersection_occlusion_release_states_.erase(it);
    }
  }
  result.trajectory = postprocessBehaviorPath(result.trajectory);
  return result;
}

}  // planning 네임스페이스
