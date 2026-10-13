// Rosless-Lanelet2의 이진 지도 직렬화 형식을 독립 계획 입력으로 변환합니다.

#include "planning/lanelet_bin_adapter.hpp"
#include "planning/jerk_resample.hpp"

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_io/io_handlers/Serialize.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace planning {
namespace {

class DetectionAreaRule : public lanelet::RegulatoryElement {
 public:
  static constexpr char RuleName[] = "detection_area";

 private:
  friend class lanelet::RegisterRegulatoryElement<DetectionAreaRule>;
  explicit DetectionAreaRule(const lanelet::RegulatoryElementDataPtr &data)
      : RegulatoryElement(data) {}
};

[[maybe_unused]] const lanelet::RegisterRegulatoryElement<DetectionAreaRule>
    detection_area_registration;

class NoStoppingAreaRule : public lanelet::RegulatoryElement {
 public:
  static constexpr char RuleName[] = "no_stopping_area";

 private:
  friend class lanelet::RegisterRegulatoryElement<NoStoppingAreaRule>;
  explicit NoStoppingAreaRule(const lanelet::RegulatoryElementDataPtr &data)
      : RegulatoryElement(data) {}
};

[[maybe_unused]] const lanelet::RegisterRegulatoryElement<NoStoppingAreaRule>
    no_stopping_area_registration;

class SpeedBumpRule : public lanelet::RegulatoryElement {
 public:
  static constexpr char RuleName[] = "speed_bump";

 private:
  friend class lanelet::RegisterRegulatoryElement<SpeedBumpRule>;
  explicit SpeedBumpRule(const lanelet::RegulatoryElementDataPtr &data)
      : RegulatoryElement(data) {}
};

[[maybe_unused]] const lanelet::RegisterRegulatoryElement<SpeedBumpRule>
    speed_bump_registration;

class VirtualTrafficLightRule : public lanelet::RegulatoryElement {
 public:
  static constexpr char RuleName[] = "virtual_traffic_light";

 private:
  friend class lanelet::RegisterRegulatoryElement<VirtualTrafficLightRule>;
  explicit VirtualTrafficLightRule(
      const lanelet::RegulatoryElementDataPtr &data)
      : RegulatoryElement(data) {}
};

[[maybe_unused]] const lanelet::RegisterRegulatoryElement<VirtualTrafficLightRule>
    virtual_traffic_light_registration;

class RoutingCostDrivable : public lanelet::routing::RoutingCostDistance {
 public:
  RoutingCostDrivable() : RoutingCostDistance(10.0) {}

  double getCostSucceeding(
      const lanelet::traffic_rules::TrafficRules &rules,
      const lanelet::ConstLaneletOrArea &from,
      const lanelet::ConstLaneletOrArea &to) const override {
    if ((from.isLanelet() && forbidden(*from.lanelet())) ||
        (to.isLanelet() && forbidden(*to.lanelet()))) {
      return std::numeric_limits<double>::infinity();
    }
    return RoutingCostDistance::getCostSucceeding(rules, from, to);
  }

  double getCostLaneChange(
      const lanelet::traffic_rules::TrafficRules &rules,
      const lanelet::ConstLanelets &from,
      const lanelet::ConstLanelets &to) const noexcept override {
    for (const auto &lane : from) {
      if (forbidden(lane)) return std::numeric_limits<double>::infinity();
    }
    for (const auto &lane : to) {
      if (forbidden(lane)) return std::numeric_limits<double>::infinity();
    }
    return RoutingCostDistance::getCostLaneChange(rules, from, to);
  }

 private:
  static bool forbidden(const lanelet::ConstLanelet &lane) {
    return lane.attributeOr("no_drivable_lane", std::string{"no"}) == "yes";
  }
};

std::vector<StopLineSegment> segments(const lanelet::ConstLineString3d &line) {
  std::vector<StopLineSegment> result;
  for (std::size_t index = 1; index < line.size(); ++index) {
    result.push_back({line[index - 1].x(), line[index - 1].y(),
                      line[index].x(), line[index].y()});
  }
  return result;
}

std::vector<CrosswalkPoint> points(const lanelet::ConstLineString3d &line) {
  std::vector<CrosswalkPoint> result;
  for (const auto &point : line) result.push_back({point.x(), point.y()});
  return result;
}

LaneTurnDirection turnDirection(const lanelet::ConstLanelet &road) {
  const auto direction = road.attributeOr("turn_direction", std::string{});
  if (direction == "left") return LaneTurnDirection::Left;
  if (direction == "right") return LaneTurnDirection::Right;
  if (direction == "straight") return LaneTurnDirection::Straight;
  return LaneTurnDirection::Else;
}

double pointToSegment(CrosswalkPoint point, CrosswalkPoint first,
                      CrosswalkPoint second) {
  const double dx = second.x - first.x;
  const double dy = second.y - first.y;
  const double length2 = dx * dx + dy * dy;
  if (length2 <= 1.0e-12) return std::hypot(point.x - first.x, point.y - first.y);
  const double ratio = std::clamp(
      ((point.x - first.x) * dx + (point.y - first.y) * dy) / length2,
      0.0, 1.0);
  return std::hypot(point.x - first.x - ratio * dx,
                    point.y - first.y - ratio * dy);
}

double pointToPolygon(CrosswalkPoint point,
                      const std::vector<CrosswalkPoint> &shape) {
  if (shape.size() < 3) return std::numeric_limits<double>::infinity();
  bool inside = false;
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < shape.size(); ++index) {
    const auto first = shape[index];
    const auto second = shape[(index + 1) % shape.size()];
    distance = std::min(distance, pointToSegment(point, first, second));
    if ((first.y > point.y) != (second.y > point.y) &&
        point.x < first.x + (point.y - first.y) *
            (second.x - first.x) / (second.y - first.y)) {
      inside = !inside;
    }
  }
  return inside ? 0.0 : distance;
}

bool strictlyWithinPolygon(CrosswalkPoint point,
                           const std::vector<CrosswalkPoint> &shape) {
  if (shape.size() < 3) return false;
  bool inside = false;
  for (std::size_t index = 0; index < shape.size(); ++index) {
    const auto first = shape[index];
    const auto second = shape[(index + 1) % shape.size()];
    if (pointToSegment(point, first, second) <= 1.0e-9) return false;
    if ((first.y > point.y) != (second.y > point.y) &&
        point.x < first.x + (point.y - first.y) *
            (second.x - first.x) / (second.y - first.y)) {
      inside = !inside;
    }
  }
  return inside;
}

double laneDistance(const lanelet::ConstLanelet &road, CrosswalkPoint point) {
  auto shape = points(road.leftBound());
  auto right = points(road.rightBound());
  std::reverse(right.begin(), right.end());
  shape.insert(shape.end(), right.begin(), right.end());
  return pointToPolygon(point, shape);
}

double headingDifference(double first, double second) {
  return std::abs(std::atan2(std::sin(first - second),
                             std::cos(first - second)));
}

double projectLength(const std::vector<CrosswalkPoint> &line,
                     CrosswalkPoint point) {
  double nearest = std::numeric_limits<double>::infinity();
  double selected = 0.0;
  double cumulative = 0.0;
  for (std::size_t i = 0; i + 1 < line.size(); ++i) {
    const double dx = line[i + 1].x - line[i].x;
    const double dy = line[i + 1].y - line[i].y;
    const double length2 = dx * dx + dy * dy;
    if (length2 <= 1.0e-12) continue;
    const double ratio = std::clamp(
        ((point.x - line[i].x) * dx + (point.y - line[i].y) * dy) /
            length2, 0.0, 1.0);
    const double distance = std::hypot(
        point.x - line[i].x - ratio * dx,
        point.y - line[i].y - ratio * dy);
    if (distance < nearest) {
      nearest = distance;
      selected = cumulative + ratio * std::sqrt(length2);
    }
    cumulative += std::sqrt(length2);
  }
  return selected;
}

std::optional<CrosswalkPoint> segmentIntersection(
    CrosswalkPoint first, CrosswalkPoint second,
    CrosswalkPoint third, CrosswalkPoint fourth) {
  const CrosswalkPoint r{second.x - first.x, second.y - first.y};
  const CrosswalkPoint s{fourth.x - third.x, fourth.y - third.y};
  const double denominator = r.x * s.y - r.y * s.x;
  if (std::abs(denominator) < 1.0e-9) return std::nullopt;
  const CrosswalkPoint delta{third.x - first.x, third.y - first.y};
  const double t = (delta.x * s.y - delta.y * s.x) / denominator;
  const double u = (delta.x * r.y - delta.y * r.x) / denominator;
  if (t < 0.0 || t > 1.0 || u < 0.0 || u > 1.0) return std::nullopt;
  return CrosswalkPoint{first.x + t * r.x, first.y + t * r.y};
}

}  // 익명 네임스페이스

struct LaneletBinAdapter::Impl {
  explicit Impl(const std::vector<std::uint8_t> &data) {
    if (data.empty()) throw std::invalid_argument("Lanelet2 이진 지도가 비어 있습니다");
    static const bool road_marking_registered = [] {
      lanelet::RegulatoryElementFactory::instance().registerStrategy(
          "road_marking", [](const lanelet::RegulatoryElementDataPtr &element) {
            return std::make_shared<lanelet::GenericRegulatoryElement>(element);
          });
      return true;
    }();
    (void)road_marking_registered;
    const std::string bytes(data.begin(), data.end());
    std::istringstream stream(bytes, std::ios::in | std::ios::binary);
    boost::archive::binary_iarchive archive(stream);
    archive >> map;
    lanelet::Id next_id = 0;
    archive >> next_id;
    lanelet::utils::registerId(next_id);
    const auto rules = lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    graph = lanelet::routing::RoutingGraph::build(map, *rules);
  }

  lanelet::LaneletMap map;
  lanelet::routing::RoutingGraphUPtr graph;
};

LaneletBinAdapter::LaneletBinAdapter(const std::vector<std::uint8_t> &map_data)
    : impl_(std::make_unique<Impl>(map_data)) {}
LaneletBinAdapter::~LaneletBinAdapter() = default;
LaneletBinAdapter::LaneletBinAdapter(LaneletBinAdapter &&) noexcept = default;
LaneletBinAdapter &LaneletBinAdapter::operator=(LaneletBinAdapter &&) noexcept = default;

std::vector<StopSignStopLine> LaneletBinAdapter::stopSignStopLinesForLanelet(
    std::int64_t lanelet_id) const {
  const auto road = impl_->map.laneletLayer.get(lanelet_id);
  std::vector<StopSignStopLine> result;
  for (const auto &sign : road.regulatoryElementsAs<lanelet::TrafficSign>()) {
    if (sign->type() != "stop_sign") continue;
    for (const auto &line : sign->refLines()) {
      for (const auto &segment : segments(line)) {
        result.push_back({sign->id(), segment});
      }
    }
  }
  return result;
}

std::vector<StopLineSegment> LaneletBinAdapter::trafficLightStopLines(
    std::int64_t regulatory_id) const {
  const auto regulatory = impl_->map.regulatoryElementLayer.get(regulatory_id);
  const auto light = std::dynamic_pointer_cast<const lanelet::TrafficLight>(regulatory);
  if (!light) throw std::runtime_error("신호등 규제 요소가 아닙니다");
  const auto stop_line = light->stopLine();
  if (!stop_line) throw std::runtime_error("신호등 정지선이 없습니다");
  return segments(*stop_line);
}

std::vector<TrafficLightStopLine> LaneletBinAdapter::trafficLightStopLinesForLanelet(
    std::int64_t lanelet_id) const {
  const auto road = impl_->map.laneletLayer.get(lanelet_id);
  std::vector<TrafficLightStopLine> result;
  for (const auto &light : road.regulatoryElementsAs<lanelet::TrafficLight>()) {
    const auto line = light->stopLine();
    if (!line) continue;
    for (const auto &segment : segments(*line)) {
      result.push_back({light->id(), segment});
    }
  }
  return result;
}

LaneTurnDirection LaneletBinAdapter::laneTurnDirection(
    std::int64_t lanelet_id) const {
  return turnDirection(impl_->map.laneletLayer.get(lanelet_id));
}

CrosswalkBounds LaneletBinAdapter::crosswalkBounds(
    std::int64_t lanelet_id) const {
  const auto crosswalk = impl_->map.laneletLayer.get(lanelet_id);
  if (crosswalk.attributeOr("subtype", std::string{}) != "crosswalk") {
    throw std::runtime_error("횡단보도 차로가 아닙니다");
  }
  return {points(crosswalk.leftBound()), points(crosswalk.rightBound())};
}

std::vector<std::int64_t> LaneletBinAdapter::crosswalkLaneletIds() const {
  std::vector<std::int64_t> ids;
  for (const auto &road : impl_->map.laneletLayer) {
    if (road.attributeOr("subtype", std::string{}) == "crosswalk") {
      ids.push_back(road.id());
    }
  }
  return ids;
}

std::vector<std::int64_t> LaneletBinAdapter::crosswalkTrafficLightIds(
    std::int64_t lanelet_id) const {
  const auto crosswalk = impl_->map.laneletLayer.get(lanelet_id);
  if (crosswalk.attributeOr("subtype", std::string{}) != "crosswalk") {
    throw std::runtime_error("횡단보도 차로가 아닙니다");
  }
  std::vector<std::int64_t> ids;
  for (const auto &light : crosswalk.regulatoryElementsAs<lanelet::TrafficLight>()) {
    ids.push_back(light->id());
  }
  return ids;
}

std::vector<std::int64_t> LaneletBinAdapter::walkwayLaneletIds() const {
  std::vector<std::int64_t> ids;
  for (const auto &road : impl_->map.laneletLayer) {
    if (road.attributeOr("subtype", std::string{}) == "walkway") {
      ids.push_back(road.id());
    }
  }
  return ids;
}

CrosswalkBounds LaneletBinAdapter::walkwayBounds(
    std::int64_t lanelet_id) const {
  const auto walkway = impl_->map.laneletLayer.get(lanelet_id);
  if (walkway.attributeOr("subtype", std::string{}) != "walkway") {
    throw std::runtime_error("보행로 차로가 아닙니다");
  }
  return {points(walkway.leftBound()), points(walkway.rightBound())};
}

std::vector<StopLineSegment> LaneletBinAdapter::walkwayStopLines(
    std::int64_t lanelet_id) const {
  const auto walkway = impl_->map.laneletLayer.get(lanelet_id);
  if (walkway.attributeOr("subtype", std::string{}) != "walkway") {
    throw std::runtime_error("보행로 차로가 아닙니다");
  }
  std::vector<StopLineSegment> result;
  for (const auto &rule : walkway.regulatoryElements()) {
    if (rule->attributeOr("subtype", std::string{}) != "road_marking") continue;
    for (const auto &line : rule->getParameters<lanelet::ConstLineString3d>(
             lanelet::RoleName::Refers)) {
      if (line.attributeOr("type", std::string{}) != "stop_line") continue;
      if (line.attributeOr("crosswalk_id", std::string{}) !=
          std::to_string(lanelet_id)) continue;
      const auto pieces = segments(line);
      result.insert(result.end(), pieces.begin(), pieces.end());
      return result;
    }
  }
  return result;
}

std::vector<std::int64_t> LaneletBinAdapter::intersectionLaneletIds() const {
  std::vector<std::int64_t> ids;
  for (const auto &road : impl_->map.laneletLayer) {
    if (turnDirection(road) != LaneTurnDirection::Else) ids.push_back(road.id());
  }
  return ids;
}

std::vector<std::int64_t> LaneletBinAdapter::intersectionConflictingLaneletIds(
    std::int64_t lanelet_id) const {
  const auto road = impl_->map.laneletLayer.get(lanelet_id);
  if (turnDirection(road) == LaneTurnDirection::Else) {
    throw std::runtime_error("교차로 차로가 아닙니다");
  }
  std::vector<std::int64_t> ids;
  for (const auto &conflicting : impl_->graph->conflicting(road)) {
    ids.push_back(conflicting.id());
  }
  return ids;
}

std::vector<std::int64_t> LaneletBinAdapter::intersectionOcclusionLaneletIds(
    std::int64_t lanelet_id, double preceding_length_m) const {
  if (!std::isfinite(preceding_length_m) || preceding_length_m < 0.0) {
    throw std::invalid_argument("교차로 가림 주목 거리 설정이 잘못되었습니다");
  }
  const auto ego_lane = impl_->map.laneletLayer.get(lanelet_id);
  std::vector<std::int64_t> ids;
  std::set<std::int64_t> visited;
  const auto append_previous = [&](const auto &self,
                                   const lanelet::ConstLanelet &lane,
                                   double remaining_m) -> void {
    if (remaining_m <= 0.0 || !visited.insert(lane.id()).second) return;
    if (lane.id() == ego_lane.id() ||
        turnDirection(lane) == LaneTurnDirection::Left ||
        turnDirection(lane) == LaneTurnDirection::Right) return;
    ids.push_back(lane.id());
    const auto center = points(lane.centerline());
    double length_m = 0.0;
    for (std::size_t index = 1; index < center.size(); ++index) {
      length_m += std::hypot(center[index].x - center[index - 1].x,
                             center[index].y - center[index - 1].y);
    }
    if (length_m >= remaining_m) return;
    for (const auto &previous : impl_->graph->previous(lane)) {
      self(self, previous, remaining_m - length_m);
    }
  };
  for (const auto conflict_id : intersectionConflictingLaneletIds(lanelet_id)) {
    const auto conflict = impl_->map.laneletLayer.get(conflict_id);
    if (visited.insert(conflict_id).second) ids.push_back(conflict_id);
    for (const auto &previous : impl_->graph->previous(conflict)) {
      append_previous(append_previous, previous, preceding_length_m);
    }
  }
  return ids;
}

std::vector<std::vector<CrosswalkPoint>>
LaneletBinAdapter::intersectionOcclusionDivisions(
    std::int64_t lanelet_id, double resolution_m,
    double preceding_length_m) const {
  if (!std::isfinite(resolution_m) || resolution_m <= 0.0 ||
      !std::isfinite(preceding_length_m) || preceding_length_m < 0.0) {
    throw std::invalid_argument("교차로 가림 탐색선 간격이 잘못되었습니다");
  }
  const auto allowed_ids = intersectionOcclusionLaneletIds(
      lanelet_id, preceding_length_m);
  const std::set<std::int64_t> allowed(allowed_ids.begin(), allowed_ids.end());
  std::vector<std::vector<CrosswalkPoint>> divisions;
  const auto append_chain = [&](const auto &self,
                                const lanelet::ConstLanelet &lane,
                                std::vector<lanelet::ConstLanelet> chain,
                                std::set<std::int64_t> visited,
                                double remaining_m) -> void {
    if (!allowed.count(lane.id()) || !visited.insert(lane.id()).second) return;
    chain.push_back(lane);
    const auto center = points(lane.centerline());
    double length_m = 0.0;
    for (std::size_t index = 1; index < center.size(); ++index) {
      length_m += std::hypot(center[index].x - center[index - 1].x,
                             center[index].y - center[index - 1].y);
    }
    std::vector<lanelet::ConstLanelet> previous;
    if (remaining_m > length_m) {
      for (const auto &candidate : impl_->graph->previous(lane)) {
        if (allowed.count(candidate.id()) && !visited.count(candidate.id())) {
          previous.push_back(candidate);
        }
      }
    }
    if (!previous.empty()) {
      for (const auto &candidate : previous) {
        self(self, candidate, chain, visited, remaining_m - length_m);
      }
      return;
    }
    std::vector<CrosswalkPoint> centerline;
    double width_length_sum = 0.0;
    double chain_length = 0.0;
    for (const auto &part : chain) {
      const auto coordinates = points(part.centerline());
      for (auto it = coordinates.rbegin(); it != coordinates.rend(); ++it) {
        if (centerline.empty() ||
            std::hypot(centerline.back().x - it->x,
                       centerline.back().y - it->y) > 1.0e-6) {
          centerline.push_back(*it);
        }
      }
      double part_length = 0.0;
      for (std::size_t index = 1; index < coordinates.size(); ++index) {
        part_length += std::hypot(
            coordinates[index].x - coordinates[index - 1].x,
            coordinates[index].y - coordinates[index - 1].y);
      }
      const auto left = points(part.leftBound());
      const auto right = points(part.rightBound());
      if (!left.empty() && !right.empty()) {
        const double width = 0.5 * (
            std::hypot(left.front().x - right.front().x,
                       left.front().y - right.front().y) +
            std::hypot(left.back().x - right.back().x,
                       left.back().y - right.back().y));
        width_length_sum += width * part_length;
        chain_length += part_length;
      }
    }
    if (centerline.size() < 2 || chain_length <= 0.0) return;
    const double width = width_length_sum / chain_length;
    // 원본처럼 차로 중심에서 양쪽 경계까지 격자 해상도 간격으로 탐색합니다.
    const auto division_count = static_cast<std::size_t>(width / resolution_m);
    for (std::size_t division_index = 0; division_index <= division_count;
         ++division_index) {
      const double offset = division_index == division_count
          ? width / 2.0
          : resolution_m * division_index - width / 2.0;
      std::vector<CrosswalkPoint> division;
      division.reserve(centerline.size());
      for (std::size_t index = 0; index < centerline.size(); ++index) {
        const auto &before = centerline[index == 0 ? 0 : index - 1];
        const auto &after = centerline[
            index + 1 < centerline.size() ? index + 1 : index];
        const double dx = after.x - before.x;
        const double dy = after.y - before.y;
        const double length = std::hypot(dx, dy);
        if (length <= 1.0e-9) continue;
        division.push_back({centerline[index].x - offset * dy / length,
                            centerline[index].y + offset * dx / length});
      }
      if (division.size() >= 2) divisions.push_back(std::move(division));
    }
  };
  for (const auto conflict_id : intersectionConflictingLaneletIds(lanelet_id)) {
    append_chain(append_chain, impl_->map.laneletLayer.get(conflict_id),
                 {}, {}, preceding_length_m);
  }
  return divisions;
}

CrosswalkBounds LaneletBinAdapter::laneBounds(
    std::int64_t lanelet_id) const {
  const auto road = impl_->map.laneletLayer.get(lanelet_id);
  return {points(road.leftBound()), points(road.rightBound())};
}

std::vector<CrosswalkPoint> LaneletBinAdapter::laneCenterline(
    std::int64_t lanelet_id) const {
  const auto road = impl_->map.laneletLayer.get(lanelet_id);
  return points(road.centerline());
}

std::vector<LaneCenterlinePoint3D> LaneletBinAdapter::laneCenterline3d(
    std::int64_t lanelet_id) const {
  const auto road = impl_->map.laneletLayer.get(lanelet_id);
  std::vector<LaneCenterlinePoint3D> result;
  result.reserve(road.centerline().size());
  for (const auto &point : road.centerline())
    result.push_back({point.x(), point.y(), point.z()});
  return result;
}

double LaneletBinAdapter::laneSpeedLimit(
    std::int64_t lanelet_id) const {
  const auto road = impl_->map.laneletLayer.get(lanelet_id);
  const auto rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  return rules->speedLimit(road).speedLimit.value();
}

std::optional<LaneChangeMappedReference>
LaneletBinAdapter::laneChangeTargetReference(
    const std::vector<std::int64_t> &target_lanelet_ids,
    BehaviorPathShiftAnchor change_start, double change_length_m,
    double change_velocity_mps, double forward_path_length_m,
    double next_change_buffer_m,
    std::optional<BehaviorPathShiftAnchor> goal) const {
  std::vector<LaneChangeCenterline> centerlines;
  centerlines.reserve(target_lanelet_ids.size());
  for (const auto lane_id : target_lanelet_ids) {
    const auto road = impl_->map.laneletLayer.get(lane_id);
    LaneChangeCenterline lane;
    lane.lane_id = lane_id;
    for (const auto &source : road.centerline()) {
      TrajectoryPoint point;
      point.x = source.x();
      point.y = source.y();
      point.z = source.z();
      lane.points.push_back(point);
    }
    centerlines.push_back(std::move(lane));
  }
  return buildLaneChangeMappedReference(
      centerlines, change_start, change_length_m,
      change_velocity_mps, forward_path_length_m,
      next_change_buffer_m, goal);
}

LaneChangeRegulatoryDistances
LaneletBinAdapter::laneChangeRegulatoryDistances(
    const Trajectory &reference,
    const std::vector<std::int64_t> &current_lanelet_ids,
    double ego_x, double ego_y) const {
  if (!std::isfinite(ego_x) || !std::isfinite(ego_y))
    throw std::invalid_argument("차로 변경 규제 거리의 자차 위치가 유효하지 않습니다");
  LaneChangeRegulatoryDistances result;
  if (reference.size() < 2) return result;
  std::vector<CrosswalkPoint> path;
  std::vector<double> arcs(reference.size(), 0.0);
  path.reserve(reference.size());
  for (std::size_t index = 0; index < reference.size(); ++index) {
    if (!std::isfinite(reference[index].x) ||
        !std::isfinite(reference[index].y))
      throw std::invalid_argument("차로 변경 기준 경로가 유효하지 않습니다");
    path.push_back({reference[index].x, reference[index].y});
    if (index > 0)
      arcs[index] = arcs[index - 1] + std::hypot(
          path[index].x - path[index - 1].x,
          path[index].y - path[index - 1].y);
  }
  const double ego_arc = projectLength(path, {ego_x, ego_y});
  const auto consider_arc = [&](std::optional<double> &destination,
                                double arc) {
    if (arc + 1.0e-6 < ego_arc) return;
    const double distance = std::max(0.0, arc - ego_arc);
    if (!destination || distance < *destination)
      destination = distance;
  };
  const auto consider_polygon = [&](std::optional<double> &destination,
                                    const lanelet::ConstLanelet &lane) {
    auto polygon = points(lane.leftBound());
    auto right = points(lane.rightBound());
    std::reverse(right.begin(), right.end());
    polygon.insert(polygon.end(), right.begin(), right.end());
    if (polygon.size() < 3) return;
    for (std::size_t index = 0; index + 1 < path.size(); ++index) {
      if (pointToPolygon(path[index], polygon) <= 1.0e-6)
        consider_arc(destination, arcs[index]);
      for (std::size_t edge = 0; edge < polygon.size(); ++edge) {
        const auto crossing = segmentIntersection(
            path[index], path[index + 1], polygon[edge],
            polygon[(edge + 1) % polygon.size()]);
        if (crossing)
          consider_arc(destination, arcs[index] + std::hypot(
              crossing->x - path[index].x,
              crossing->y - path[index].y));
      }
    }
  };
  for (const auto lane_id : current_lanelet_ids) {
    const auto lane = impl_->map.laneletLayer.get(lane_id);
    if (turnDirection(lane) != LaneTurnDirection::Else ||
        lane.hasAttribute("intersection_area"))
      consider_polygon(result.intersection_m, lane);
    for (const auto &line : trafficLightStopLinesForLanelet(lane_id)) {
      const CrosswalkPoint first{line.segment.x1, line.segment.y1};
      const CrosswalkPoint second{line.segment.x2, line.segment.y2};
      for (std::size_t index = 0; index + 1 < path.size(); ++index) {
        const auto crossing = segmentIntersection(
            path[index], path[index + 1], first, second);
        if (crossing)
          consider_arc(result.traffic_light_m,
                       arcs[index] + std::hypot(
                           crossing->x - path[index].x,
                           crossing->y - path[index].y));
      }
    }
  }
  for (const auto &lane : impl_->map.laneletLayer) {
    if (lane.attributeOr("subtype", std::string{}) == "crosswalk")
      consider_polygon(result.crosswalk_m, lane);
  }
  return result;
}

std::optional<std::int64_t> LaneletBinAdapter::nearestRoadLaneletId(
    double x, double y, double maximum_distance_m) const {
  if (!std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(maximum_distance_m) || maximum_distance_m < 0.0) {
    throw std::invalid_argument("가까운 도로 차로 검색 입력이 유효하지 않습니다");
  }
  const auto closest = lanelet::geometry::findNearest(
      impl_->map.laneletLayer, lanelet::BasicPoint2d(x, y), 20);
  for (const auto &[distance, lane] : closest) {
    if (distance > maximum_distance_m) break;
    if (lane.attributeOr("subtype", std::string{}) == "road") {
      return lane.id();
    }
  }
  return std::nullopt;
}

std::optional<std::int64_t> LaneletBinAdapter::closestRouteRoadLaneletId(
    double x, double y, double yaw_rad,
    const std::vector<std::int64_t> &candidate_ids,
    double maximum_distance_m, double maximum_yaw_rad) const {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw_rad) ||
      std::isnan(maximum_distance_m) || maximum_distance_m < 0.0 ||
      std::isnan(maximum_yaw_rad) || maximum_yaw_rad < 0.0)
    throw std::invalid_argument("route 차로 검색 입력이 유효하지 않습니다");
  const lanelet::BasicPoint2d point{x, y};
  double best_distance = std::numeric_limits<double>::infinity();
  double best_angle = std::numeric_limits<double>::infinity();
  constexpr double tie_tolerance = std::numeric_limits<double>::epsilon();
  std::optional<std::int64_t> selected;
  for (const auto id : candidate_ids) {
    const auto lane = impl_->map.laneletLayer.get(id);
    if (lane.attributeOr("subtype", std::string{}) != "road") continue;
    const double distance = boost::geometry::distance(
        lane.polygon2d().basicPolygon(), point);
    if (distance > maximum_distance_m ||
        distance > best_distance + tie_tolerance) continue;
    const auto center = lane.centerline2d();
    double nearest_segment = std::numeric_limits<double>::infinity();
    double angle = 3.141592653589793;
    for (std::size_t i = 1; i < center.size(); ++i) {
      const double dx = center[i].x() - center[i - 1].x();
      const double dy = center[i].y() - center[i - 1].y();
      const double squared = dx * dx + dy * dy;
      if (squared < 1.0e-12) continue;
      const double ratio = std::clamp(
          ((x - center[i - 1].x()) * dx +
           (y - center[i - 1].y()) * dy) / squared, 0.0, 1.0);
      const double segment_distance = std::hypot(
          x - center[i - 1].x() - ratio * dx,
          y - center[i - 1].y() - ratio * dy);
      if (segment_distance < nearest_segment) {
        nearest_segment = segment_distance;
        angle = std::abs(std::remainder(
            std::atan2(dy, dx) - yaw_rad, 6.283185307179586));
      }
    }
    if (angle > maximum_yaw_rad) continue;
    if (!selected || distance < best_distance - tie_tolerance ||
        (std::abs(distance - best_distance) <= tie_tolerance &&
         angle < best_angle)) {
      selected = id;
      best_distance = distance;
      best_angle = angle;
    }
  }
  return selected;
}

bool LaneletBinAdapter::areAdjacentRoadLanelets(
    std::int64_t current_lanelet_id,
    std::int64_t target_lanelet_id) const {
  const auto current = impl_->map.laneletLayer.get(current_lanelet_id);
  const auto target = impl_->map.laneletLayer.get(target_lanelet_id);
  if (current.attributeOr("subtype", std::string{}) != "road" ||
      target.attributeOr("subtype", std::string{}) != "road") return false;
  const auto current_line = current.centerline2d();
  const auto target_line = target.centerline2d();
  if (current_line.size() < 2 || target_line.size() < 2) return false;
  const auto current_dx = current_line.back().x() - current_line.front().x();
  const auto current_dy = current_line.back().y() - current_line.front().y();
  const auto target_dx = target_line.back().x() - target_line.front().x();
  const auto target_dy = target_line.back().y() - target_line.front().y();
  if (current_dx * target_dx + current_dy * target_dy <= 0.0)
    return false;
  if (current.leftBound().id() == target.rightBound().id() ||
      current.rightBound().id() == target.leftBound().id())
    return true;
  for (const auto &beside : impl_->graph->besides(current)) {
    if (beside.id() == target_lanelet_id) return true;
  }
  return false;
}

std::optional<std::int64_t>
LaneletBinAdapter::nextRoadLaneletTowardPreferred(
    std::int64_t current_lanelet_id,
    std::int64_t preferred_lanelet_id,
    const std::vector<std::int64_t> &allowed_ids) const {
  const auto current = impl_->map.laneletLayer.get(
      current_lanelet_id).centerline2d();
  const auto preferred = impl_->map.laneletLayer.get(
      preferred_lanelet_id).centerline2d();
  if (current.empty() || preferred.empty()) return std::nullopt;
  const auto midpoint = [](const lanelet::ConstLineString2d &line) {
    const auto &first = line.front();
    const auto &last = line.back();
    return CrosswalkPoint{(first.x() + last.x()) / 2.0,
                          (first.y() + last.y()) / 2.0};
  };
  const auto desired = midpoint(preferred);
  const auto current_point = midpoint(current);
  double best_distance = std::hypot(
      current_point.x - desired.x,
      current_point.y - desired.y);
  std::optional<std::int64_t> selected;
  for (const auto candidate_id : allowed_ids) {
    if (candidate_id == current_lanelet_id ||
        !areAdjacentRoadLanelets(current_lanelet_id, candidate_id))
      continue;
    const auto candidate = impl_->map.laneletLayer.get(
        candidate_id).centerline2d();
    if (candidate.empty()) continue;
    const auto point = midpoint(candidate);
    const double distance = std::hypot(
        point.x - desired.x, point.y - desired.y);
    if (distance + 0.01 < best_distance) {
      best_distance = distance;
      selected = candidate_id;
    }
  }
  return selected;
}

bool LaneletBinAdapter::pointInCrosswalk(double x, double y) const {
  if (!std::isfinite(x) || !std::isfinite(y)) return false;
  for (const auto &lane : impl_->map.laneletLayer) {
    if (lane.attributeOr("subtype", std::string{}) == "crosswalk" &&
        laneDistance(lane, {x, y}) <= 1.0e-6)
      return true;
  }
  return false;
}

bool LaneletBinAdapter::pointInIntersection(double x, double y) const {
  if (!std::isfinite(x) || !std::isfinite(y)) return false;
  for (const auto &lane : impl_->map.laneletLayer) {
    if ((turnDirection(lane) != LaneTurnDirection::Else ||
         lane.hasAttribute("intersection_area")) &&
        laneDistance(lane, {x, y}) <= 1.0e-6)
      return true;
  }
  return false;
}

bool LaneletBinAdapter::pointInRoadLanelet(
    std::int64_t lanelet_id, double x, double y) const {
  if (!std::isfinite(x) || !std::isfinite(y)) return false;
  const auto lane = impl_->map.laneletLayer.get(lanelet_id);
  return lane.attributeOr("subtype", std::string{}) == "road" &&
         laneDistance(lane, {x, y}) <= 1.0e-6;
}

bool LaneletBinAdapter::hasAdjacentRoadLanelet(
    std::int64_t lanelet_id, bool left_side) const {
  return adjacentRoadLaneletId(lanelet_id, left_side).has_value();
}

std::optional<std::int64_t> LaneletBinAdapter::adjacentRoadLaneletId(
    std::int64_t lanelet_id, bool left_side) const {
  const auto current = impl_->map.laneletLayer.get(lanelet_id);
  const auto routed = left_side ? impl_->graph->left(current) :
                                  impl_->graph->right(current);
  if (routed && areAdjacentRoadLanelets(lanelet_id, routed->id()))
    return routed->id();
  const auto adjacent = left_side ?
      impl_->graph->adjacentLeft(current) :
      impl_->graph->adjacentRight(current);
  if (adjacent && areAdjacentRoadLanelets(lanelet_id, adjacent->id()))
    return adjacent->id();
  const auto boundary_id = left_side ? current.leftBound().id() :
                                  current.rightBound().id();
  for (const auto &neighbor : impl_->map.laneletLayer) {
    if (neighbor.id() == lanelet_id ||
        neighbor.attributeOr("subtype", std::string{}) != "road")
      continue;
    const auto shared_id = left_side ? neighbor.rightBound().id() :
                                       neighbor.leftBound().id();
    if (shared_id == boundary_id &&
        areAdjacentRoadLanelets(lanelet_id, neighbor.id()))
      return neighbor.id();
  }
  return std::nullopt;
}

bool LaneletBinAdapter::hasOppositeRoadLanelet(
    std::int64_t lanelet_id, bool left_side) const {
  const auto current = impl_->map.laneletLayer.get(lanelet_id);
  const auto shared = left_side ? current.leftBound() :
                                  current.rightBound();
  const auto reversed = shared.invert();
  for (const auto &neighbor :
       impl_->map.laneletLayer.findUsages(reversed)) {
    if (neighbor.id() == lanelet_id ||
        neighbor.attributeOr("subtype", std::string{}) != "road")
      continue;
    if (neighbor.leftBound() == reversed ||
        neighbor.rightBound() == reversed)
      return true;
  }
  return false;
}

std::optional<std::int64_t> LaneletBinAdapter::followingRoadLaneletId(
    std::int64_t lanelet_id,
    const std::vector<std::int64_t> &candidate_ids,
    std::int64_t preferred_lanelet_id) const {
  const auto current = impl_->map.laneletLayer.get(lanelet_id);
  std::optional<std::int64_t> selected;
  for (const auto &next : impl_->graph->following(current)) {
    if (next.attributeOr("subtype", std::string{}) != "road" ||
        std::find(candidate_ids.begin(), candidate_ids.end(),
                  next.id()) == candidate_ids.end())
      continue;
    if (next.id() == preferred_lanelet_id) return next.id();
    if (!selected) selected = next.id();
  }
  if (selected) return selected;
  const auto current_line = current.centerline2d();
  if (current_line.size() < 2) return std::nullopt;
  const auto current_dx = current_line.back().x() -
                          current_line.front().x();
  const auto current_dy = current_line.back().y() -
                          current_line.front().y();
  for (const auto candidate_id : candidate_ids) {
    const auto next = impl_->map.laneletLayer.get(candidate_id);
    if (next.attributeOr("subtype", std::string{}) != "road")
      continue;
    const auto next_line = next.centerline2d();
    if (next_line.size() < 2 ||
        std::hypot(current.leftBound().back().x() -
                       next.leftBound().front().x(),
                   current.leftBound().back().y() -
                       next.leftBound().front().y()) > 0.01 ||
        std::hypot(current.rightBound().back().x() -
                       next.rightBound().front().x(),
                   current.rightBound().back().y() -
                       next.rightBound().front().y()) > 0.01)
      continue;
    const double next_dx = next_line.back().x() - next_line.front().x();
    const double next_dy = next_line.back().y() - next_line.front().y();
    if (current_dx * next_dx + current_dy * next_dy <= 0.0)
      continue;
    if (candidate_id == preferred_lanelet_id) return candidate_id;
    if (!selected) selected = candidate_id;
  }
  return selected;
}

std::vector<std::vector<BehaviorPathPoint2d>>
LaneletBinAdapter::lanePolygons(
    const std::vector<std::int64_t> &lanelet_ids) const {
  std::vector<std::vector<BehaviorPathPoint2d>> polygons;
  polygons.reserve(lanelet_ids.size());
  for (const auto id : lanelet_ids) {
    const auto road = impl_->map.laneletLayer.get(id);
    std::vector<BehaviorPathPoint2d> polygon;
    polygon.reserve(road.leftBound().size() + road.rightBound().size());
    for (const auto &point : road.leftBound()) {
      polygon.push_back({point.x(), point.y()});
    }
    const auto right = road.rightBound();
    for (std::size_t index = right.size(); index > 0; --index) {
      const auto point = right[index - 1];
      polygon.push_back({point.x(), point.y()});
    }
    polygons.push_back(std::move(polygon));
  }
  return polygons;
}

CostmapAreaPolygons
LaneletBinAdapter::costmapAreaPolygons() const {
  CostmapAreaPolygons areas;
  for (const auto &lane : impl_->map.laneletLayer) {
    if (lane.attributeOr("subtype", std::string{}) != "road") continue;
    BehaviorSafetyPolygon polygon;
    for (const auto &point : lane.polygon2d().basicPolygon())
      polygon.emplace_back(point.x(), point.y());
    if (polygon.size() >= 3) areas.road.push_back(std::move(polygon));
  }
  for (const auto &parking : impl_->map.polygonLayer) {
    if (parking.attributeOr("type", std::string{}) != "parking_lot")
      continue;
    BehaviorSafetyPolygon polygon;
    for (const auto &point : parking)
      polygon.emplace_back(point.x(), point.y());
    if (polygon.size() >= 3) areas.parking.push_back(std::move(polygon));
  }
  for (const auto &space : impl_->map.lineStringLayer) {
    if (space.attributeOr("type", std::string{}) != "parking_space" ||
        space.size() != 2 || !space.hasAttribute("width")) continue;
    const double half_width = space.attributeOr("width", 0.0) * 0.5;
    if (!(half_width > 0.0)) continue;
    const auto &a = space.front();
    const auto &b = space.back();
    const double dx = b.x() - a.x();
    const double dy = b.y() - a.y();
    const double length = std::hypot(dx, dy);
    if (length < 1.0e-9) continue;
    const double nx = -dy / length * half_width;
    const double ny = dx / length * half_width;
    areas.parking.push_back({{a.x() + nx, a.y() + ny},
                     {b.x() + nx, b.y() + ny},
                     {b.x() - nx, b.y() - ny},
                     {a.x() - nx, a.y() - ny}});
  }
  return areas;
}

std::vector<std::vector<BehaviorPathPoint2d>>
LaneletBinAdapter::lanePolygonsNearPath(
    const Trajectory &path,
    const BehaviorPathVehicleFootprint &vehicle) const {
  namespace bg = boost::geometry;
  using Point = bg::model::d2::point_xy<double>;
  using Polygon = bg::model::polygon<Point>;
  if (path.empty()) return {};
  if (!std::isfinite(vehicle.front_m) || !std::isfinite(vehicle.back_m) ||
      !std::isfinite(vehicle.half_width_m) || vehicle.front_m < 0.0 ||
      vehicle.back_m < 0.0 || vehicle.half_width_m <= 0.0) {
    throw std::invalid_argument("차로 후보 차량 외형 입력이 유효하지 않습니다");
  }
  bg::model::multi_point<Point> footprint_points;
  for (const auto &point : path) {
    const double yaw = std::atan2(
        2.0 * (point.orientation_w * point.orientation_z +
               point.orientation_x * point.orientation_y),
        1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                     point.orientation_z * point.orientation_z));
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(yaw)) {
      throw std::invalid_argument("차로 후보 경로 점이 유효하지 않습니다");
    }
    for (const double along : {-vehicle.back_m, vehicle.front_m}) {
      for (const double across : {-vehicle.half_width_m,
                                  vehicle.half_width_m}) {
        footprint_points.emplace_back(
            point.x + along * std::cos(yaw) - across * std::sin(yaw),
            point.y + along * std::sin(yaw) + across * std::cos(yaw));
      }
    }
  }
  Polygon hull;
  bg::convex_hull(footprint_points, hull);
  std::vector<std::int64_t> ids;
  for (const auto &road : impl_->map.laneletLayer) {
    if (bg::intersects(hull, road.polygon2d().basicPolygon())) {
      ids.push_back(road.id());
    }
  }
  return lanePolygons(ids);
}

BehaviorGoalPath LaneletBinAdapter::refineFixedGoalPath(
    const BehaviorGoalPath &input, BehaviorGoalPose goal,
    std::int64_t goal_lanelet_id,
    double initial_search_radius_m) const {
  const auto goal_lane = impl_->map.laneletLayer.get(goal_lanelet_id);
  if (!lanelet::geometry::inside(
          goal_lane, lanelet::BasicPoint2d(goal.x, goal.y))) {
    const auto center = points(goal_lane.centerline());
    const CrosswalkPoint original_goal{goal.x, goal.y};
    double nearest = std::numeric_limits<double>::infinity();
    for (std::size_t index = 1; index < center.size(); ++index) {
      const double error = pointToSegment(
          original_goal, center[index - 1], center[index]);
      if (error >= nearest) continue;
      const double dx = center[index].x - center[index - 1].x;
      const double dy = center[index].y - center[index - 1].y;
      const double length_squared = dx * dx + dy * dy;
      if (length_squared < 1.0e-12) continue;
      nearest = error;
      const double ratio =
          ((original_goal.x - center[index - 1].x) * dx +
           (original_goal.y - center[index - 1].y) * dy) / length_squared;
      goal.x = center[index - 1].x + ratio * dx;
      goal.y = center[index - 1].y + ratio * dy;
      goal.yaw_rad = std::atan2(dy, dx);
    }
  }
  const auto valid = [&](const BehaviorGoalPath &path) {
    std::set<std::int64_t> ids;
    for (const auto &point_ids : path.lane_ids) {
      ids.insert(point_ids.begin(), point_ids.end());
    }
    std::vector<lanelet::ConstLanelet> lanes;
    for (const auto id : ids) {
      lanes.push_back(impl_->map.laneletLayer.get(id));
    }
    for (const auto &point : path.points) {
      const auto inside = std::any_of(lanes.begin(), lanes.end(),
          [&](const auto &lane) {
            return lanelet::geometry::inside(
                lane, lanelet::BasicPoint2d(point.x, point.y));
          });
      if (!inside) return false;
    }
    return true;
  };
  return refineFixedBehaviorGoalPath(
      input, goal, goal_lanelet_id, initial_search_radius_m, valid);
}

std::optional<BlindSpotMapRegion> LaneletBinAdapter::blindSpotRegion(
    std::int64_t lanelet_id, double ignore_width_from_center_m,
    double adjacent_extend_width_m, LaneTurnDirection direction) const {
  const auto road = impl_->map.laneletLayer.get(lanelet_id);
  if (direction == LaneTurnDirection::Else) direction = turnDirection(road);
  if (direction != LaneTurnDirection::Left &&
      direction != LaneTurnDirection::Right) return std::nullopt;

  const bool left_turn = direction == LaneTurnDirection::Left;
  const auto centerline = road.centerline2d();
  const auto inner = lanelet::geometry::offset(
      centerline, left_turn ? ignore_width_from_center_m
                            : -ignore_width_from_center_m);
  if (inner.size() < 2) return std::nullopt;
  std::vector<CrosswalkPoint> inner_points;
  for (const auto &point : inner) {
    inner_points.push_back({point.x(), point.y()});
  }

  auto outer_points = points(left_turn ? road.leftBound() : road.rightBound());
  const auto shared_bound = left_turn ? road.leftBound() : road.rightBound();
  std::optional<lanelet::ConstLanelet> shoulder;
  for (const auto &candidate : impl_->map.laneletLayer.findUsages(shared_bound)) {
    const bool same_boundary = left_turn
        ? candidate.rightBound() == shared_bound
        : candidate.leftBound() == shared_bound;
    if (same_boundary && candidate.attributeOr(
            "subtype", std::string{}) == "road_shoulder") {
      shoulder = candidate;
      break;
    }
  }
  const auto adjacent = left_turn ? impl_->graph->adjacentLeft(road)
                                  : impl_->graph->adjacentRight(road);
  std::optional<lanelet::ConstLanelet> opposite;
  if (!shoulder && !adjacent) {
    for (const auto &candidate : impl_->map.laneletLayer.findUsages(
             shared_bound.invert())) {
      if (candidate.id() != road.id()) {
        opposite = candidate;
        break;
      }
    }
  }
  if (shoulder) {
    outer_points = points(left_turn ? shoulder->leftBound()
                                    : shoulder->rightBound());
  } else if ((adjacent || opposite) && adjacent_extend_width_m > 0.0) {
    const auto neighbor = adjacent ? *adjacent : *opposite;
    const auto adjacent_centerline = neighbor.centerline2d();
    const auto adjacent_width =
        std::abs(boost::geometry::area(neighbor.polygon2d().basicPolygon())) /
        std::max(1.0e-9, static_cast<double>(
            boost::geometry::length(adjacent_centerline.basicLineString())));
    const auto extend = std::min(adjacent_extend_width_m, adjacent_width);
    const auto offset = left_turn ? -adjacent_width / 2.0 + extend
                                  : adjacent_width / 2.0 - extend;
    const auto outer = lanelet::geometry::offset(adjacent_centerline, offset);
    if (outer.size() >= 2) {
      outer_points.clear();
      for (const auto &point : outer) {
        outer_points.push_back({point.x(), point.y()});
      }
      if (opposite) std::reverse(outer_points.begin(), outer_points.end());
    }
  }
  if (outer_points.size() < 2) return std::nullopt;

  BlindSpotMapRegion region;
  region.lanelet_id = lanelet_id;
  const auto &left = left_turn ? outer_points : inner_points;
  const auto &right = left_turn ? inner_points : outer_points;
  lanelet::Points3d left_nodes;
  lanelet::Points3d right_nodes;
  for (const auto &point : left) {
    left_nodes.emplace_back(lanelet::InvalId, point.x, point.y, 0.0);
  }
  for (const auto &point : right) {
    right_nodes.emplace_back(lanelet::InvalId, point.x, point.y, 0.0);
  }
  const auto virtual_lane = lanelet::Lanelet(
      lanelet::InvalId,
      lanelet::LineString3d(lanelet::InvalId, std::move(left_nodes)),
      lanelet::LineString3d(lanelet::InvalId, std::move(right_nodes)));
  region.centerline = points(virtual_lane.centerline());
  region.centerline.erase(std::unique(
      region.centerline.begin(), region.centerline.end(),
      [](const auto &first, const auto &second) {
        return std::hypot(first.x - second.x, first.y - second.y) < 1.0e-9;
      }), region.centerline.end());
  region.polygon.insert(region.polygon.end(), left.begin(), left.end());
  region.polygon.insert(region.polygon.end(), right.rbegin(), right.rend());
  return region;
}

BlindSpotMappedObjects LaneletBinAdapter::mapBlindSpotObjects(
    const std::vector<std::int64_t> &preceding_lane_ids,
    LaneTurnDirection direction, CrosswalkPoint ego_position,
    const std::vector<BlindSpotMapObject> &objects,
    double backward_detection_length_m,
    std::optional<CrosswalkPoint> stop_position) const {
  namespace bg = boost::geometry;
  using Point = bg::model::d2::point_xy<double>;
  using Polygon = bg::model::polygon<Point>;
  struct Section {
    BlindSpotMapRegion region;
    Polygon polygon;
    double start_arc_m = 0.0;
    double length_m = 0.0;
  };
  std::vector<Section> sections;
  BlindSpotMappedObjects result;
  if (direction != LaneTurnDirection::Left &&
      direction != LaneTurnDirection::Right) return result;

  const auto projected = [](const std::vector<CrosswalkPoint> &line,
                            CrosswalkPoint position) {
    double best_arc = 0.0;
    double best_distance = std::numeric_limits<double>::infinity();
    double accumulated = 0.0;
    for (std::size_t index = 1; index < line.size(); ++index) {
      const auto &first = line[index - 1];
      const auto &second = line[index];
      const double dx = second.x - first.x;
      const double dy = second.y - first.y;
      const double length = std::hypot(dx, dy);
      if (length <= 1.0e-9) continue;
      const double ratio = std::clamp(
          ((position.x - first.x) * dx + (position.y - first.y) * dy) /
              (length * length), 0.0, 1.0);
      const double distance = std::hypot(
          position.x - first.x - ratio * dx,
          position.y - first.y - ratio * dy);
      if (distance < best_distance) {
        best_distance = distance;
        best_arc = accumulated + ratio * length;
      }
      accumulated += length;
    }
    return std::pair<double, double>{best_arc, best_distance};
  };

  for (const auto lane_id : preceding_lane_ids) {
    const auto region = blindSpotRegion(lane_id, 0.7, 1.5, direction);
    if (!region || region->centerline.size() < 2 ||
        region->polygon.size() < 3) continue;
    Section section;
    section.region = *region;
    section.start_arc_m = result.stop_arc_m;
    for (std::size_t index = 1; index < region->centerline.size(); ++index) {
      section.length_m += std::hypot(
          region->centerline[index].x - region->centerline[index - 1].x,
          region->centerline[index].y - region->centerline[index - 1].y);
    }
    if (section.length_m <= 1.0e-9) continue;
    for (const auto &point : region->polygon) {
      bg::append(section.polygon.outer(), Point(point.x, point.y));
    }
    bg::correct(section.polygon);
    result.stop_arc_m += section.length_m;
    sections.push_back(std::move(section));
  }
  if (sections.empty()) return result;

  double ego_distance = std::numeric_limits<double>::infinity();
  double stop_distance = std::numeric_limits<double>::infinity();
  for (const auto &section : sections) {
    const auto [arc, distance] = projected(section.region.centerline,
                                           ego_position);
    if (distance < ego_distance) {
      ego_distance = distance;
      result.ego_arc_m = section.start_arc_m + arc;
    }
    if (stop_position) {
      const auto [stop_arc, distance_to_stop] = projected(
          section.region.centerline, *stop_position);
      if (distance_to_stop < stop_distance) {
        stop_distance = distance_to_stop;
        result.stop_arc_m = section.start_arc_m + stop_arc;
      }
    }
  }

  for (const auto &object : objects) {
    BlindSpotObject mapped;
    mapped.id = object.id;
    mapped.classification = object.classification;
    mapped.longitudinal_velocity_mps = object.longitudinal_velocity_mps;
    for (const auto &section : sections) {
      if (!bg::within(Point(object.position.x, object.position.y),
                      section.polygon)) continue;
      const auto [arc, distance] = projected(section.region.centerline,
                                             object.position);
      (void)distance;
      const double global_arc = section.start_arc_m + arc;
      if (global_arc < result.stop_arc_m - backward_detection_length_m ||
          global_arc > result.stop_arc_m) continue;
      mapped.inside_detection_area = true;
      mapped.arc_on_detection_lane_m = global_arc;
      break;
    }
    result.objects.push_back(std::move(mapped));
  }
  return result;
}

std::optional<BlindSpotStopArcs> LaneletBinAdapter::blindSpotStopArcs(
    std::int64_t lanelet_id, const Trajectory &path,
    double front_offset_m, double rear_offset_m, double half_width_m,
    double stop_line_margin_m,
    const std::vector<CrosswalkPoint> &multi_lane_points) const {
  namespace bg = boost::geometry;
  using Point = bg::model::d2::point_xy<double>;
  using Polygon = bg::model::polygon<Point>;
  using Line = bg::model::linestring<Point>;
  if (path.size() < 2) return std::nullopt;
  const auto road = impl_->map.laneletLayer.get(lanelet_id);
  if (turnDirection(road) != LaneTurnDirection::Left &&
      turnDirection(road) != LaneTurnDirection::Right) return std::nullopt;

  const auto projected_arc = [&](CrosswalkPoint position) {
    double best_distance = std::numeric_limits<double>::infinity();
    double best_arc = 0.0;
    double accumulated = 0.0;
    for (std::size_t index = 1; index < path.size(); ++index) {
      const double dx = path[index].x - path[index - 1].x;
      const double dy = path[index].y - path[index - 1].y;
      const double length = std::hypot(dx, dy);
      if (length <= 1.0e-9) continue;
      const double ratio = std::clamp(
          ((position.x - path[index - 1].x) * dx +
           (position.y - path[index - 1].y) * dy) /
              (length * length), 0.0, 1.0);
      const double distance = std::hypot(
          position.x - path[index - 1].x - ratio * dx,
          position.y - path[index - 1].y - ratio * dy);
      if (distance < best_distance) {
        best_distance = distance;
        best_arc = accumulated + ratio * length;
      }
      accumulated += length;
    }
    return best_arc;
  };
  const auto entry_point = CrosswalkPoint{
      (road.leftBound().front().x() + road.rightBound().front().x()) / 2.0,
      (road.leftBound().front().y() + road.rightBound().front().y()) / 2.0};
  const double entry_arc = projected_arc(entry_point);

  std::optional<lanelet::ConstLanelet> sibling;
  for (const auto &previous : impl_->graph->previous(road)) {
    for (const auto &following : impl_->graph->following(previous)) {
      if (turnDirection(following) == LaneTurnDirection::Straight) {
        sibling = following;
        break;
      }
    }
    if (sibling) break;
  }
  Trajectory compact_path;
  for (const auto &point : path) {
    if (compact_path.empty() ||
        std::hypot(point.x - compact_path.back().x,
                   point.y - compact_path.back().y) > 1.0e-9) {
      compact_path.push_back(point);
    }
  }
  if (compact_path.size() < 2) return std::nullopt;
  if (!sibling) {
    double total_arc = 0.0;
    for (std::size_t index = 1; index < compact_path.size(); ++index)
      total_arc += std::hypot(
          compact_path[index].x - compact_path[index - 1].x,
          compact_path[index].y - compact_path[index - 1].y);
    std::vector<double> sample_arcs;
    for (double arc = 0.0; arc < total_arc; arc += 0.2)
      sample_arcs.push_back(arc);
    if (sample_arcs.size() > 1 && total_arc - sample_arcs.back() < 0.1)
      sample_arcs.back() = total_arc;
    else
      sample_arcs.push_back(total_arc);
    for (const auto &anchor : multi_lane_points) {
      const double anchor_arc = projected_arc(anchor);
      for (std::size_t index = 1; index < sample_arcs.size(); ++index) {
        if (sample_arcs[index - 1] <= anchor_arc &&
            anchor_arc < sample_arcs[index]) {
          if (anchor_arc - sample_arcs[index - 1] < 0.1)
            sample_arcs[index - 1] = anchor_arc;
          else if (sample_arcs[index] - anchor_arc < 0.1)
            sample_arcs[index] = anchor_arc;
          else
            sample_arcs.insert(sample_arcs.begin() + index, anchor_arc);
          break;
        }
      }
    }
    const auto sampled = resampleJerkAtArcs(compact_path, sample_arcs);
    std::size_t closest_segment = 0;
    double closest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index + 1 < sampled.size(); ++index) {
      const double distance = pointToSegment(
          entry_point, {sampled[index].x, sampled[index].y},
          {sampled[index + 1].x, sampled[index + 1].y});
      if (distance < closest_distance) {
        closest_distance = distance;
        closest_segment = index;
      }
    }
    double stop_arc = 0.0;
    for (std::size_t index = 1; index <= closest_segment; ++index)
      stop_arc += std::hypot(sampled[index].x - sampled[index - 1].x,
                             sampled[index].y - sampled[index - 1].y);
    return BlindSpotStopArcs{stop_arc, stop_arc};
  }
  const auto sampled = resampleJerkUniform(compact_path, 0.2);

  const auto boundary = turnDirection(road) == LaneTurnDirection::Left
      ? sibling->leftBound() : sibling->rightBound();
  Line line;
  for (const auto &point : boundary) {
    bg::append(line, Point(point.x(), point.y()));
  }
  if (line.size() < 2) return std::nullopt;
  double arc = 0.0;
  for (std::size_t index = 0; index < sampled.size(); ++index) {
    if (index > 0) {
      arc += std::hypot(sampled[index].x - sampled[index - 1].x,
                        sampled[index].y - sampled[index - 1].y);
    }
    if (arc + 1.0e-9 < entry_arc - front_offset_m - rear_offset_m) continue;
    const double yaw = 2.0 * std::atan2(
        sampled[index].orientation_z, sampled[index].orientation_w);
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    Polygon footprint;
    for (const auto &offset : {
             std::pair<double, double>{front_offset_m, half_width_m},
             {front_offset_m, -half_width_m},
             {-rear_offset_m, -half_width_m},
             {-rear_offset_m, half_width_m}}) {
      bg::append(footprint.outer(), Point(
          sampled[index].x + cosine * offset.first - sine * offset.second,
          sampled[index].y + sine * offset.first + cosine * offset.second));
    }
    bg::correct(footprint);
    if (bg::intersects(footprint, line)) {
      return BlindSpotStopArcs{
          std::max(0.0, arc - stop_line_margin_m), arc};
    }
  }
  return std::nullopt;
}

std::vector<VelocityLimiterSegment> LaneletBinAdapter::staticObstacleSegments(
    const double minimum_x, const double minimum_y,
    const double maximum_x, const double maximum_y,
    const std::vector<std::string> &tags) const {
  if (!std::isfinite(minimum_x) || !std::isfinite(minimum_y) ||
      !std::isfinite(maximum_x) || !std::isfinite(maximum_y) ||
      minimum_x > maximum_x || minimum_y > maximum_y)
    throw std::invalid_argument("정적 장애물 탐색 영역이 유효하지 않습니다");
  lanelet::BoundingBox2d bounds;
  bounds.extend(lanelet::BasicPoint2d{minimum_x, minimum_y});
  bounds.extend(lanelet::BasicPoint2d{maximum_x, maximum_y});
  std::vector<VelocityLimiterSegment> result;
  for (const auto &line : impl_->map.lineStringLayer.search(bounds)) {
    const auto type = line.attributeOr("type", std::string{});
    if (std::find(tags.begin(), tags.end(), type) == tags.end()) continue;
    lanelet::BasicLineString2d source;
    for (const auto &point : line)
      source.emplace_back(point.x(), point.y());
    lanelet::BasicLineString2d simplified;
    boost::geometry::simplify(source, simplified, 0.5);
    for (std::size_t i = 1; i < simplified.size(); ++i)
      result.push_back({{simplified[i - 1].x(), simplified[i - 1].y()},
                        {simplified[i].x(), simplified[i].y()}});
  }
  return result;
}

std::vector<VelocityLimiterSegment> LaneletBinAdapter::partitionSegments() const {
  std::vector<VelocityLimiterSegment> result;
  for (const auto &line : impl_->map.lineStringLayer) {
    const auto type = line.attributeOr("type", std::string{});
    if (type != "guard_rail" && type != "fence" && type != "wall") continue;
    for (std::size_t i = 1; i < line.size(); ++i)
      result.push_back({{line[i - 1].x(), line[i - 1].y()},
                        {line[i].x(), line[i].y()}});
  }
  return result;
}

MotionVelocityPlan LaneletBinAdapter::planStaticObstacleVelocityLimit(
    const Trajectory &path, const std::vector<double> &time_from_start,
    const std::size_t ego_index, const double current_ego_velocity,
    const ObstacleVelocityLimiterParameters &parameters,
    const std::vector<std::string> &tags,
    const BicycleVelocityProjectionParameters &bicycle_parameters) const {
  auto prepared = path;
  if (parameters.calculate_steering_angles)
    calculateVelocityLimiterSteeringAngles(
        prepared, bicycle_parameters.wheel_base);
  const auto sampled = sampleObstacleVelocityLimiterPath(
      prepared, time_from_start, ego_index, parameters);
  if (sampled.path.empty()) return {};
  double minimum_x = std::numeric_limits<double>::infinity();
  double minimum_y = std::numeric_limits<double>::infinity();
  double maximum_x = -std::numeric_limits<double>::infinity();
  double maximum_y = -std::numeric_limits<double>::infinity();
  double search_radius = parameters.vehicle_lateral_offset;
  for (const auto &point : sampled.path) {
    minimum_x = std::min(minimum_x, point.x);
    minimum_y = std::min(minimum_y, point.y);
    maximum_x = std::max(maximum_x, point.x);
    maximum_y = std::max(maximum_y, point.y);
    search_radius = std::max(search_radius,
        std::abs(point.longitudinal_velocity_mps *
                 parameters.minimum_time_to_collision +
                 parameters.extra_projection_length) +
        parameters.vehicle_lateral_offset);
  }
  const auto segments = staticObstacleSegments(
      minimum_x - search_radius, minimum_y - search_radius,
      maximum_x + search_radius, maximum_y + search_radius, tags);
  if (parameters.model == VelocityLimiterProjectionModel::Bicycle)
    return planObstacleVelocityLimitBicycle(
        sampled.path, sampled.time_from_start, {}, segments,
        current_ego_velocity, parameters, bicycle_parameters);
  return planObstacleVelocityLimitParticle(
      sampled.path, sampled.time_from_start, {}, segments,
      current_ego_velocity, parameters);
}

std::vector<std::int64_t> LaneletBinAdapter::outOfLaneRoadLaneletIds(
    const std::vector<CrosswalkPoint> &path,
    const double maximum_footprint_offset_m,
    const double front_offset_m) const {
  if (path.size() < 2 || !std::isfinite(maximum_footprint_offset_m) ||
      !std::isfinite(front_offset_m) || maximum_footprint_offset_m < 0.0 ||
      front_offset_m < 0.0)
    throw std::invalid_argument("차로 밖 주행 경로·차량 외형 입력이 유효하지 않습니다");
  lanelet::BasicLineString2d line;
  for (const auto &point : path)
    line.emplace_back(point.x, point.y);
  const double dx = path.back().x - path[path.size() - 2].x;
  const double dy = path.back().y - path[path.size() - 2].y;
  const double length = std::hypot(dx, dy);
  if (length > 1e-9)
    line.emplace_back(path.back().x + front_offset_m * dx / length,
                      path.back().y + front_offset_m * dy / length);
  std::set<std::int64_t> trajectory;
  for (const auto &road : impl_->map.laneletLayer) {
    if (road.attributeOr("subtype", std::string{}) != "road") continue;
    if (!boost::geometry::disjoint(line, road.polygon2d().basicPolygon()))
      trajectory.insert(road.id());
  }
  std::set<std::int64_t> consecutive;
  std::set<std::int64_t> adjacent;
  for (const auto id : trajectory) {
    const auto road = impl_->map.laneletLayer.get(id);
    for (const auto &next : impl_->graph->following(road))
      consecutive.insert(next.id());
    for (const auto &previous : impl_->graph->previous(road))
      consecutive.insert(previous.id());
    for (const auto &beside : impl_->graph->besides(road))
      adjacent.insert(beside.id());
  }
  for (const auto id : adjacent)
    if (consecutive.count(id)) trajectory.insert(id);
  std::set<std::int64_t> ignored;
  for (const auto id : trajectory) {
    const auto road = impl_->map.laneletLayer.get(id);
    for (const auto &previous : impl_->graph->previous(road))
      if (!trajectory.count(previous.id())) ignored.insert(previous.id());
  }
  std::vector<std::int64_t> result;
  for (const auto &road : impl_->map.laneletLayer) {
    if (road.attributeOr("subtype", std::string{}) != "road" ||
        trajectory.count(road.id()) || ignored.count(road.id())) continue;
    if (boost::geometry::distance(line, road.polygon2d().basicPolygon()) <=
        maximum_footprint_offset_m) result.push_back(road.id());
  }
  std::sort(result.begin(), result.end());
  return result;
}

std::vector<OutOfLanePolygon> LaneletBinAdapter::outOfLaneRoadPolygons(
    const Trajectory &path,
    const double maximum_footprint_offset_m,
    const double front_offset_m) const {
  std::vector<CrosswalkPoint> centers;
  centers.reserve(path.size());
  for (const auto &point : path)
    centers.push_back({point.x, point.y});
  const auto ids = outOfLaneRoadLaneletIds(
      centers, maximum_footprint_offset_m, front_offset_m);
  std::vector<OutOfLanePolygon> polygons;
  polygons.reserve(ids.size());
  for (const auto id : ids) {
    OutOfLanePolygon vertices;
    const auto geometry =
        impl_->map.laneletLayer.get(id).polygon2d().basicPolygon();
    vertices.reserve(geometry.size());
    for (const auto &point : geometry)
      vertices.push_back({point.x(), point.y()});
    polygons.push_back(std::move(vertices));
  }
  return polygons;
}

std::optional<MotionVelocityPoint2D>
LaneletBinAdapter::findOutOfLaneMapCandidate(
    const Trajectory &path,
    const std::vector<double> &time_from_start,
    const std::vector<OutOfLanePredictedPath> &objects,
    const double minimum_stop_arc,
    const OutOfLaneParameters &decision,
    const OutOfLaneGeometryParameters &geometry) const {
  const double maximum_offset = std::max({
      geometry.ego_front_offset + geometry.extra_front_offset,
      -geometry.ego_rear_offset + geometry.extra_rear_offset,
      geometry.ego_left_offset + geometry.extra_left_offset,
      -geometry.ego_right_offset + geometry.extra_right_offset});
  const auto polygons = outOfLaneRoadPolygons(
      path, maximum_offset, geometry.ego_front_offset);
  return planning::findOutOfLaneSlowdownCandidate(
      path, time_from_start, polygons, objects, minimum_stop_arc,
      decision, geometry);
}

std::vector<OutOfLaneStopLine> LaneletBinAdapter::outOfLaneRedStopLines(
    const MotionVelocityPoint2D ego_position,
    const double search_distance,
    const std::vector<std::int64_t> &red_traffic_light_ids) const {
  if (!std::isfinite(ego_position.x) || !std::isfinite(ego_position.y) ||
      !std::isfinite(search_distance) || search_distance < 0.0)
    throw std::invalid_argument("차로 밖 주행 정지선 검색 범위가 유효하지 않습니다");
  const std::set<std::int64_t> red_ids(
      red_traffic_light_ids.begin(), red_traffic_light_ids.end());
  if (red_ids.empty()) return {};
  const lanelet::BoundingBox2d search_box(
      lanelet::BasicPoint2d(ego_position.x - search_distance,
                            ego_position.y - search_distance),
      lanelet::BasicPoint2d(ego_position.x + search_distance,
                            ego_position.y + search_distance));
  std::vector<OutOfLaneStopLine> result;
  for (const auto &road : impl_->map.laneletLayer.search(search_box)) {
    for (const auto &light : road.regulatoryElementsAs<lanelet::TrafficLight>()) {
      if (!red_ids.count(light->id())) continue;
      const auto stop_line = light->stopLine();
      if (!stop_line || stop_line->size() < 2) continue;
      std::vector<MotionVelocityPoint2D> line;
      for (const auto &point : *stop_line)
        line.push_back({point.x(), point.y()});
      const MotionVelocityPoint2D diff{
          line.back().x - line.front().x,
          line.back().y - line.front().y};
      line.front().x -= 0.5 * diff.x;
      line.front().y -= 0.5 * diff.y;
      line.back().x += 0.5 * diff.x;
      line.back().y += 0.5 * diff.y;
      for (const auto &previous : impl_->graph->previous(road)) {
        OutOfLanePolygon polygon;
        const auto vertices = previous.polygon2d().basicPolygon();
        for (const auto &point : vertices)
          polygon.push_back({point.x(), point.y()});
        result.push_back({line, std::move(polygon)});
      }
    }
  }
  return result;
}

std::optional<MotionVelocityPoint2D>
LaneletBinAdapter::findOutOfLaneMapCandidateFromTrackedObjects(
    const Trajectory &path,
    const std::vector<double> &time_from_start,
    const MotionVelocityPoint2D ego_position,
    const std::vector<OutOfLaneTrackedObject> &objects,
    const std::vector<std::int64_t> &red_traffic_light_ids,
    const double search_distance, const double minimum_stop_arc,
    const OutOfLaneParameters &decision_parameters,
    const OutOfLaneGeometryParameters &geometry_parameters,
    const OutOfLaneObjectFilterParameters &filter_parameters) const {
  const auto stop_lines = outOfLaneRedStopLines(
      ego_position, search_distance, red_traffic_light_ids);
  const auto filtered = filterOutOfLanePredictedObjects(
      path, ego_position, objects, stop_lines, filter_parameters);
  return findOutOfLaneMapCandidate(
      path, time_from_start, filtered, minimum_stop_arc,
      decision_parameters, geometry_parameters);
}

bool LaneletBinAdapter::isVehicleOnIntersectionConflictLane(
    std::int64_t lanelet_id, double x, double y, double yaw_rad,
    double distance_margin_m, double heading_margin_rad) const {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw_rad) ||
      !std::isfinite(distance_margin_m) || !std::isfinite(heading_margin_rad) ||
      distance_margin_m < 0.0 || heading_margin_rad < 0.0) {
    throw std::invalid_argument("교차로 차량 위치와 방향 설정이 잘못되었습니다");
  }
  const auto road = impl_->map.laneletLayer.get(lanelet_id);
  if (turnDirection(road) == LaneTurnDirection::Else) {
    throw std::runtime_error("교차로 차로가 아닙니다");
  }
  for (const auto &conflicting : impl_->graph->conflicting(road)) {
    const auto conflict_lane = conflicting.lanelet();
    if (!conflict_lane ||
        laneDistance(*conflict_lane, {x, y}) > distance_margin_m) continue;
    const auto center = conflict_lane->centerline();
    double nearest = std::numeric_limits<double>::infinity();
    double lane_yaw = 0.0;
    for (std::size_t i = 1; i < center.size(); ++i) {
      const CrosswalkPoint first{center[i - 1].x(), center[i - 1].y()};
      const CrosswalkPoint second{center[i].x(), center[i].y()};
      const double distance = pointToSegment({x, y}, first, second);
      if (distance < nearest) {
        nearest = distance;
        lane_yaw = std::atan2(second.y - first.y, second.x - first.x);
      }
    }
    if (nearest < std::numeric_limits<double>::infinity() &&
        headingDifference(yaw_rad, lane_yaw) <= heading_margin_rad) {
      return true;
    }
  }
  return false;
}

bool LaneletBinAdapter::isVehicleOnIntersectionAttentionLane(
    std::int64_t lanelet_id, double x, double y, double yaw_rad,
    double distance_margin_m, double heading_margin_rad) const {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw_rad) ||
      !std::isfinite(distance_margin_m) || !std::isfinite(heading_margin_rad) ||
      distance_margin_m < 0.0 || heading_margin_rad < 0.0) {
    throw std::invalid_argument("교차로 차량 위치와 방향 설정이 잘못되었습니다");
  }
  for (const auto id : intersectionOcclusionLaneletIds(lanelet_id, 200.0)) {
    const auto lane = impl_->map.laneletLayer.get(id);
    if (laneDistance(lane, {x, y}) > distance_margin_m) continue;
    const auto center = points(lane.centerline());
    double nearest = std::numeric_limits<double>::infinity();
    double lane_yaw = 0.0;
    for (std::size_t index = 1; index < center.size(); ++index) {
      const double distance = pointToSegment({x, y},
          center[index - 1], center[index]);
      if (distance < nearest) {
        nearest = distance;
        lane_yaw = std::atan2(center[index].y - center[index - 1].y,
                              center[index].x - center[index - 1].x);
      }
    }
    if (nearest < std::numeric_limits<double>::infinity() &&
        headingDifference(yaw_rad, lane_yaw) <= heading_margin_rad) return true;
  }
  return false;
}

std::vector<std::int64_t> LaneletBinAdapter::intersectionObservedTrafficLightIds(
    std::int64_t lanelet_id) const {
  const auto road = impl_->map.laneletLayer.get(lanelet_id);
  std::vector<std::int64_t> ids;
  for (const auto &light : road.regulatoryElementsAs<lanelet::TrafficLight>()) {
    for (const auto &bulb :
         light->getParameters<lanelet::ConstLineString3d>("light_bulbs")) {
      if (bulb.hasAttribute("traffic_light_id")) {
        ids.push_back(light->id());
        break;
      }
    }
  }
  return ids;
}

bool LaneletBinAdapter::intersectionHasTrafficLight(
    std::int64_t lanelet_id) const {
  const auto road = impl_->map.laneletLayer.get(lanelet_id);
  for (const auto &light : road.regulatoryElementsAs<lanelet::TrafficLight>()) {
    if (light->stopLine()) return true;
  }
  return false;
}

std::vector<DetectionAreaMapFeature>
LaneletBinAdapter::detectionAreasForLanelet(
    std::int64_t lanelet_id) const {
  const auto lane = impl_->map.laneletLayer.get(lanelet_id);
  std::vector<DetectionAreaMapFeature> result;
  for (const auto &rule : lane.regulatoryElements()) {
    if (rule->attributeOr("subtype", std::string{}) != "detection_area") {
      continue;
    }
    const auto lines = rule->getParameters<lanelet::ConstLineString3d>(
        lanelet::RoleName::RefLine);
    if (lines.empty() || lines.front().size() < 2) continue;
    DetectionAreaMapFeature feature;
    feature.regulatory_id = rule->id();
    feature.stop_line = {
        lines.front().front().x(), lines.front().front().y(),
        lines.front().back().x(), lines.front().back().y()};
    for (const auto &area :
         rule->getParameters<lanelet::ConstPolygon3d>(lanelet::RoleName::Refers)) {
      std::vector<CrosswalkPoint> shape;
      for (const auto &point : area) {
        shape.push_back({point.x(), point.y()});
      }
      if (shape.size() >= 3) feature.areas.push_back(std::move(shape));
    }
    if (!feature.areas.empty()) result.push_back(std::move(feature));
  }
  return result;
}

std::vector<NoStoppingAreaMapFeature>
LaneletBinAdapter::noStoppingAreasForLanelet(std::int64_t lanelet_id) const {
  const auto lane = impl_->map.laneletLayer.get(lanelet_id);
  std::vector<NoStoppingAreaMapFeature> result;
  for (const auto &rule : lane.regulatoryElements()) {
    if (rule->attributeOr("subtype", std::string{}) !=
        "no_stopping_area") continue;
    const auto lines = rule->getParameters<lanelet::ConstLineString3d>(
        lanelet::RoleName::RefLine);
    NoStoppingAreaMapFeature feature;
    feature.regulatory_id = rule->id();
    if (!lines.empty() && lines.front().size() >= 2) {
      feature.stop_line = StopLineSegment{
          lines.front().front().x(), lines.front().front().y(),
          lines.front().back().x(), lines.front().back().y()};
    }
    for (const auto &area :
         rule->getParameters<lanelet::ConstPolygon3d>(
             lanelet::RoleName::Refers)) {
      std::vector<CrosswalkPoint> shape;
      for (const auto &point : area) {
        shape.push_back({point.x(), point.y()});
      }
      if (shape.size() >= 3) feature.areas.push_back(std::move(shape));
    }
    if (!feature.areas.empty()) result.push_back(std::move(feature));
  }
  return result;
}

std::vector<VirtualTrafficLightMapFeature>
LaneletBinAdapter::virtualTrafficLightsForLanelet(
    std::int64_t lanelet_id) const {
  const auto lane = impl_->map.laneletLayer.get(lanelet_id);
  std::vector<VirtualTrafficLightMapFeature> result;
  for (const auto &rule : lane.regulatoryElements()) {
    if (rule->attributeOr("subtype", std::string{}) !=
        "virtual_traffic_light") continue;
    VirtualTrafficLightMapFeature feature;
    feature.regulatory_id = rule->id();
    const auto instruments = rule->getParameters<lanelet::ConstLineString3d>(
        lanelet::RoleName::Refers);
    if (!instruments.empty()) {
      const auto &instrument = instruments.front();
      feature.command_metadata.instrument_type =
          instrument.attributeOr("type", std::string{});
      feature.command_metadata.instrument_id =
          std::to_string(instrument.id());
      for (const auto &attribute : instrument.attributes()) {
        if (attribute.first == "type") continue;
        feature.command_metadata.custom_tags.emplace_back(
            attribute.first, attribute.second.value());
      }
      feature.command_metadata.custom_tags.emplace_back(
          "lane_id", std::to_string(lanelet_id));
      feature.command_metadata.custom_tags.emplace_back(
          "turn_direction",
          lane.attributeOr("turn_direction", std::string{"straight"}));
    }
    const auto append_lines = [](const auto &lines, auto &output) {
      for (const auto &line : lines) {
        const auto pieces = segments(line);
        output.insert(output.end(), pieces.begin(), pieces.end());
      }
    };
    append_lines(rule->getParameters<lanelet::ConstLineString3d>(
        "start_line"), feature.start_lines);
    append_lines(rule->getParameters<lanelet::ConstLineString3d>(
        lanelet::RoleName::RefLine), feature.stop_lines);
    append_lines(rule->getParameters<lanelet::ConstLineString3d>(
        "end_line"), feature.end_lines);
    if (!feature.start_lines.empty() && !feature.end_lines.empty()) {
      result.push_back(std::move(feature));
    }
  }
  return result;
}

std::vector<SpeedBumpFeature> LaneletBinAdapter::speedBumpsForLanelet(
    std::int64_t lanelet_id) const {
  const auto lane = impl_->map.laneletLayer.get(lanelet_id);
  std::vector<SpeedBumpFeature> result;
  for (const auto &rule : lane.regulatoryElements()) {
    if (rule->attributeOr("subtype", std::string{}) != "speed_bump") continue;
    for (const auto &area :
         rule->getParameters<lanelet::ConstPolygon3d>(lanelet::RoleName::Refers)) {
      SpeedBumpFeature feature;
      feature.regulatory_id = rule->id();
      feature.height_m = area.attributeOr("height", 0.5);
      if (area.hasAttribute("slow_down_speed")) {
        feature.speed_kph = area.attribute("slow_down_speed").asDouble().value();
      }
      for (const auto &point : area) {
        feature.polygon.push_back({point.x(), point.y()});
      }
      if (feature.polygon.size() >= 3) result.push_back(std::move(feature));
    }
  }
  return result;
}

bool LaneletBinAdapter::isNoDrivableLane(std::int64_t lanelet_id) const {
  return impl_->map.laneletLayer.get(lanelet_id).attributeOr(
      "no_drivable_lane", std::string{"no"}) == "yes";
}

bool LaneletBinAdapter::isPrivateLane(std::int64_t lanelet_id) const {
  return impl_->map.laneletLayer.get(lanelet_id).attributeOr(
      "location", std::string{}) == "private";
}

bool LaneletBinAdapter::isUrbanLane(std::int64_t lanelet_id) const {
  return impl_->map.laneletLayer.get(lanelet_id).attributeOr(
      "location", std::string{}) == "urban";
}

std::optional<MissionRoute> LaneletBinAdapter::routeBetweenCheckpoints(
    RouteCheckpoint start, RouteCheckpoint goal,
    bool consider_no_drivable_lanes) const {
  return routeBetweenCheckpointsImpl(
      start, goal, consider_no_drivable_lanes, true);
}

std::optional<MissionRoute> LaneletBinAdapter::routeBetweenCheckpointsImpl(
    RouteCheckpoint start, RouteCheckpoint goal,
    bool consider_no_drivable_lanes, bool validate_goal,
    std::vector<std::int64_t> *path_lanelet_ids) const {
  for (const double value : {
           start.x, start.y, start.yaw_rad,
           start.z, goal.x, goal.y, goal.yaw_rad, goal.z}) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("경로 체크포인트에는 유한한 값이 필요합니다");
    }
  }
  std::vector<lanelet::ConstLanelet> starts;
  std::optional<lanelet::ConstLanelet> nearest_start;
  std::optional<lanelet::ConstLanelet> nearest_goal;
  double start_distance = 20.0;
  double goal_distance = 20.0;
  double goal_heading_error = std::numeric_limits<double>::infinity();
  for (const auto &lane : impl_->map.laneletLayer) {
    if (lane.attributeOr("subtype", std::string{}) != "road") continue;
    const double distance_start = laneDistance(lane, {start.x, start.y});
    const double distance_goal = laneDistance(lane, {goal.x, goal.y});
    if (lanelet::geometry::inside(
            lane, lanelet::BasicPoint2d(start.x, start.y))) {
      starts.push_back(lane);
    }
    if (distance_start < start_distance) {
      start_distance = distance_start;
      nearest_start = lane;
    }
    double angle_to_goal = std::numeric_limits<double>::infinity();
    if (distance_goal <= goal_distance + 1.0e-9) {
      const auto center = points(lane.centerline());
      double nearest_center = std::numeric_limits<double>::infinity();
      for (std::size_t index = 1; index < center.size(); ++index) {
        const double distance = pointToSegment(
            {goal.x, goal.y}, center[index - 1], center[index]);
        if (distance < nearest_center) {
          nearest_center = distance;
          angle_to_goal = headingDifference(
              goal.yaw_rad,
              std::atan2(center[index].y - center[index - 1].y,
                         center[index].x - center[index - 1].x));
        }
      }
    }
    if (distance_goal < goal_distance - 1.0e-9 ||
        (std::abs(distance_goal - goal_distance) <= 1.0e-9 &&
         angle_to_goal < goal_heading_error)) {
      goal_distance = distance_goal;
      goal_heading_error = angle_to_goal;
      nearest_goal = lane;
    }
  }
  if (starts.empty() && nearest_start) starts.push_back(*nearest_start);
  if (starts.empty() || !nearest_goal) return std::nullopt;
  bool goal_in_aligned_shoulder = false;
  for (const auto &lane : impl_->map.laneletLayer) {
    if (lane.attributeOr("subtype", std::string{}) != "road_shoulder" ||
        !lanelet::geometry::inside(
            lane, lanelet::BasicPoint2d(goal.x, goal.y))) continue;
    const auto center = points(lane.centerline());
    double nearest = std::numeric_limits<double>::infinity();
    double lane_yaw = 0.0;
    for (std::size_t i = 1; i < center.size(); ++i) {
      const double distance = pointToSegment(
          {goal.x, goal.y}, center[i - 1], center[i]);
      if (distance < nearest) {
        nearest = distance;
        lane_yaw = std::atan2(center[i].y - center[i - 1].y,
                              center[i].x - center[i - 1].x);
      }
    }
    if (headingDifference(goal.yaw_rad, lane_yaw) <
        0.7853981633974483) {
      goal_in_aligned_shoulder = true;
      break;
    }
  }
  bool goal_in_parking_lot = false;
  for (const auto &area : impl_->map.polygonLayer) {
    if (area.attributeOr("type", std::string{}) != "parking_lot") continue;
    std::vector<CrosswalkPoint> shape;
    for (const auto &point : area) shape.push_back({point.x(), point.y()});
    if (pointToPolygon({goal.x, goal.y}, shape) <= 1.0e-9) {
      goal_in_parking_lot = true;
      break;
    }
  }
  bool goal_in_parking_space = false;
  for (const auto &space : impl_->map.lineStringLayer) {
    if (space.attributeOr("type", std::string{}) != "parking_space" ||
        space.size() != 2 || !space.hasAttribute("width")) continue;
    const double width = space.attributeOr("width", 0.0);
    if (width <= 0.0) continue;
    const auto &first = space.front();
    const auto &last = space.back();
    const double dx = last.x() - first.x();
    const double dy = last.y() - first.y();
    const double length_squared = dx * dx + dy * dy;
    if (length_squared <= 0.0) continue;
    const double projection =
        ((goal.x - first.x()) * dx + (goal.y - first.y()) * dy) /
        length_squared;
    const double lateral = std::abs(
        (goal.x - first.x()) * dy - (goal.y - first.y()) * dx) /
        std::sqrt(length_squared);
    if (projection >= 0.0 && projection <= 1.0 &&
        lateral <= width / 2.0) {
      goal_in_parking_space = true;
      break;
    }
  }
  const auto goal_centerline = points(nearest_goal->centerline());
  double nearest_goal_center = std::numeric_limits<double>::infinity();
  double goal_lane_yaw = 0.0;
  for (std::size_t i = 1; i < goal_centerline.size(); ++i) {
    const double distance = pointToSegment(
        {goal.x, goal.y}, goal_centerline[i - 1], goal_centerline[i]);
    if (distance < nearest_goal_center) {
      nearest_goal_center = distance;
      goal_lane_yaw = std::atan2(
          goal_centerline[i].y - goal_centerline[i - 1].y,
          goal_centerline[i].x - goal_centerline[i - 1].x);
    }
  }
  if (validate_goal && !goal_in_aligned_shoulder && !goal_in_parking_lot &&
      !goal_in_parking_space &&
      (!lanelet::geometry::inside(
           *nearest_goal, lanelet::BasicPoint2d(goal.x, goal.y)) ||
       headingDifference(goal.yaw_rad, goal_lane_yaw) >= 0.7853981633974483)) {
    return std::nullopt;
  }
  std::optional<lanelet::routing::LaneletPath> selected_path;
  double best_angle = std::numeric_limits<double>::infinity();
  for (const auto &lane : starts) {
    const auto center = points(lane.centerline());
    double nearest = std::numeric_limits<double>::infinity();
    double lane_yaw = 0.0;
    for (std::size_t i = 1; i < center.size(); ++i) {
      const double distance = pointToSegment(
          {start.x, start.y}, center[i - 1], center[i]);
      if (distance < nearest) {
        nearest = distance;
        lane_yaw = std::atan2(center[i].y - center[i - 1].y,
                              center[i].x - center[i - 1].x);
      }
    }
    const double angle = headingDifference(start.yaw_rad, lane_yaw);
    if (angle > 1.5707963267948966 || angle >= best_angle) continue;
    const auto route = impl_->graph->getRoute(lane, *nearest_goal, 0);
    if (!route) continue;
    best_angle = angle;
    selected_path = route->shortestPath();
  }
  if (!selected_path || selected_path->empty()) return std::nullopt;
  if (consider_no_drivable_lanes && std::any_of(
          selected_path->begin(), selected_path->end(), [](const auto &lane) {
            return lane.attributeOr("no_drivable_lane", std::string{"no"}) == "yes";
          })) {
    const auto rules = lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    const auto drivable_graph = lanelet::routing::RoutingGraph::build(
        impl_->map, *rules,
        lanelet::routing::RoutingCostPtrs{
            std::make_shared<RoutingCostDrivable>()});
    const auto route = drivable_graph->getRoute(
        selected_path->front(), *nearest_goal, 0);
    if (route) selected_path = route->shortestPath();
  }
  if (path_lanelet_ids) {
    path_lanelet_ids->clear();
    for (const auto &lane : *selected_path) {
      path_lanelet_ids->push_back(lane.id());
    }
  }
  if (validate_goal) {
    namespace bg = boost::geometry;
    using Point = bg::model::d2::point_xy<double>;
    using Polygon = bg::model::polygon<Point>;
    bg::model::multi_polygon<Polygon> road_area;
    const auto append_lane = [&](const lanelet::ConstLanelet &lane) {
      Polygon shape;
      for (const auto &point : lane.leftBound()) {
        bg::append(shape.outer(), Point(point.x(), point.y()));
      }
      const auto right = points(lane.rightBound());
      for (auto it = right.rbegin(); it != right.rend(); ++it) {
        bg::append(shape.outer(), Point(it->x, it->y));
      }
      bg::correct(shape);
      road_area.push_back(std::move(shape));
    };
    for (const auto &lane : *selected_path) {
      append_lane(lane);
    }
    std::set<lanelet::Id> visited_next;
    const auto append_next = [&](const auto &self,
                                 const lanelet::ConstLanelet &lane,
                                 double remaining_m) -> void {
      if (!visited_next.insert(lane.id()).second) return;
      append_lane(lane);
      const auto center = points(lane.centerline());
      double length_m = 0.0;
      for (std::size_t i = 1; i < center.size(); ++i) {
        length_m += std::hypot(center[i].x - center[i - 1].x,
                               center[i].y - center[i - 1].y);
      }
      if (length_m >= remaining_m) return;
      for (const auto &next : impl_->graph->following(lane)) {
        self(self, next, remaining_m - length_m);
      }
    };
    for (const auto &next : impl_->graph->following(*nearest_goal)) {
      append_next(append_next, next, 3.79);
    }
    Polygon footprint_shape;
    constexpr double front_m = 3.79;
    constexpr double rear_m = -1.1;
    constexpr double center_m = 1.395;
    constexpr double half_width_m = 0.948;
    const double cosine = std::cos(goal.yaw_rad);
    const double sine = std::sin(goal.yaw_rad);
    for (const auto &offset : {
             std::pair<double, double>{front_m, half_width_m},
             {front_m, -half_width_m}, {center_m, -half_width_m},
             {rear_m, -half_width_m}, {rear_m, half_width_m},
             {center_m, half_width_m}}) {
      bg::append(footprint_shape.outer(), Point(
          goal.x + cosine * offset.first - sine * offset.second,
          goal.y + sine * offset.first + cosine * offset.second));
    }
    bg::correct(footprint_shape);
    bg::model::multi_polygon<Polygon> outside;
    bg::difference(footprint_shape, road_area, outside);
    if (!outside.empty() && !goal_in_parking_lot &&
        !goal_in_aligned_shoulder) return std::nullopt;
  }
  std::set<lanelet::Id> route_ids;
  for (const auto &lane : *selected_path) {
    route_ids.insert(lane.id());
    for (const auto &relation : impl_->graph->leftRelations(lane)) {
      if (relation.relationType == lanelet::routing::RelationType::Left) {
        route_ids.insert(relation.lanelet.id());
      }
    }
    for (const auto &relation : impl_->graph->rightRelations(lane)) {
      if (relation.relationType == lanelet::routing::RelationType::Right) {
        route_ids.insert(relation.lanelet.id());
      }
    }
  }
  MissionRoute result;
  {
    const auto center = nearest_goal->centerline();
    double nearest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1; i < center.size(); ++i) {
      const double dx = center[i].x() - center[i - 1].x();
      const double dy = center[i].y() - center[i - 1].y();
      const double dz = center[i].z() - center[i - 1].z();
      const double length2 = dx * dx + dy * dy + dz * dz;
      if (length2 <= 1.0e-12) continue;
      const double ratio = std::clamp(
          ((goal.x - center[i - 1].x()) * dx +
           (goal.y - center[i - 1].y()) * dy +
           (goal.z - center[i - 1].z()) * dz) / length2,
          0.0, 1.0);
      const double distance = std::hypot(
          goal.x - center[i - 1].x() - ratio * dx,
          goal.y - center[i - 1].y() - ratio * dy,
          goal.z - center[i - 1].z() - ratio * dz);
      if (distance < nearest) {
        nearest = distance;
        result.goal_height_m = center[i - 1].z() + ratio * dz;
      }
    }
  }
  std::size_t main_path_start = 0;
  if (selected_path->size() >= 2) {
    const auto following = impl_->graph->following((*selected_path)[0]);
    const bool connected = std::any_of(
        following.begin(), following.end(), [&](const auto &lane) {
          return lane.id() == (*selected_path)[1].id();
        });
    if (!connected) main_path_start = 1;
  }
  for (std::size_t path_index = main_path_start;
       path_index < selected_path->size(); ++path_index) {
    const auto &lane = (*selected_path)[path_index];
    MissionRouteSection section;
    section.preferred_lanelet_id = lane.id();
    const auto left = impl_->graph->leftRelations(lane);
    for (auto it = left.rbegin(); it != left.rend(); ++it) {
      if (route_ids.count(it->lanelet.id())) {
        section.primitive_lanelet_ids.push_back(it->lanelet.id());
      }
    }
    section.primitive_lanelet_ids.push_back(lane.id());
    for (const auto &relation : impl_->graph->rightRelations(lane)) {
      if (route_ids.count(relation.lanelet.id())) {
        section.primitive_lanelet_ids.push_back(relation.lanelet.id());
      }
    }
    result.sections.push_back(std::move(section));
  }
  return result;
}

std::optional<MissionRoute> LaneletBinAdapter::routeThroughCheckpoints(
    const std::vector<RouteCheckpoint> &checkpoints,
    bool consider_no_drivable_lanes) const {
  if (checkpoints.size() < 2) {
    throw std::invalid_argument("경로 체크포인트는 두 개 이상 필요합니다");
  }
  MissionRoute result;
  std::vector<std::int64_t> path_ids;
  for (std::size_t i = 1; i < checkpoints.size(); ++i) {
    std::vector<std::int64_t> section_path_ids;
    const auto section_route = routeBetweenCheckpointsImpl(
        checkpoints[i - 1], checkpoints[i], consider_no_drivable_lanes,
        i + 1 == checkpoints.size(), &section_path_ids);
    if (!section_route) return std::nullopt;
    result.goal_height_m = section_route->goal_height_m;
    for (const auto id : section_path_ids) {
      if (path_ids.empty() || path_ids.back() != id) {
        path_ids.push_back(id);
      }
    }
  }
  if (path_ids.empty()) return std::nullopt;
  const std::set<std::int64_t> path_id_set(path_ids.begin(), path_ids.end());
  std::set<std::int64_t> route_ids(path_ids.begin(), path_ids.end());
  for (const auto id : path_ids) {
    const auto lane = impl_->map.laneletLayer.get(id);
    for (const auto &relation : impl_->graph->leftRelations(lane)) {
      if (relation.relationType == lanelet::routing::RelationType::Left) {
        route_ids.insert(relation.lanelet.id());
      }
    }
    for (const auto &relation : impl_->graph->rightRelations(lane)) {
      if (relation.relationType == lanelet::routing::RelationType::Right) {
        route_ids.insert(relation.lanelet.id());
      }
    }
  }
  std::set<std::int64_t> start_ids{path_ids.front()};
  const auto start_lane = impl_->map.laneletLayer.get(path_ids.front());
  for (const auto &relation : impl_->graph->leftRelations(start_lane)) {
    start_ids.insert(relation.lanelet.id());
  }
  for (const auto &relation : impl_->graph->rightRelations(start_lane)) {
    start_ids.insert(relation.lanelet.id());
  }
  std::vector<lanelet::ConstLanelet> main_path;
  std::set<std::int64_t> visited;
  lanelet::ConstLanelet current = impl_->map.laneletLayer.get(path_ids.back());
  while (visited.insert(current.id()).second) {
    main_path.push_back(current);
    if (start_ids.count(current.id())) break;
    bool found = false;
    for (const auto &previous : impl_->graph->previous(current)) {
      if (route_ids.count(previous.id()) && !visited.count(previous.id())) {
        current = previous;
        found = true;
        break;
      }
    }
    if (!found) {
      const auto try_lateral_previous = [&](const auto &relations) {
        for (const auto &relation : relations) {
          if (!route_ids.count(relation.lanelet.id())) continue;
          for (const auto &previous :
               impl_->graph->previous(relation.lanelet)) {
            if (path_id_set.count(previous.id()) &&
                !visited.count(previous.id())) {
              current = previous;
              return true;
            }
          }
        }
        return false;
      };
      found = try_lateral_previous(impl_->graph->rightRelations(current)) ||
              try_lateral_previous(impl_->graph->leftRelations(current));
    }
    if (!found) break;
  }
  std::reverse(main_path.begin(), main_path.end());
  for (const auto &lane : main_path) {
    MissionRouteSection section;
    section.preferred_lanelet_id = lane.id();
    const auto left = impl_->graph->leftRelations(lane);
    for (auto it = left.rbegin(); it != left.rend(); ++it) {
      if (route_ids.count(it->lanelet.id())) {
        section.primitive_lanelet_ids.push_back(it->lanelet.id());
      }
    }
    section.primitive_lanelet_ids.push_back(lane.id());
    for (const auto &relation : impl_->graph->rightRelations(lane)) {
      if (route_ids.count(relation.lanelet.id())) {
        section.primitive_lanelet_ids.push_back(relation.lanelet.id());
      }
    }
    result.sections.push_back(std::move(section));
  }
  std::set<std::int64_t> section_ids;
  for (const auto &section : result.sections) {
    for (const auto id : section.primitive_lanelet_ids) {
      if (!section_ids.insert(id).second) return std::nullopt;
    }
  }
  return result;
}

std::optional<IntersectionTrafficControlDistances>
LaneletBinAdapter::intersectionTrafficControlDistances(
    std::int64_t lanelet_id, double x, double y,
    double yaw_rad) const {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw_rad)) {
    throw std::invalid_argument("교차 차량 위치와 방향이 잘못되었습니다");
  }
  const auto ego_lane = impl_->map.laneletLayer.get(lanelet_id);
  const auto ego_centerline = points(ego_lane.centerline());
  for (const auto conflict_id :
       intersectionConflictingLaneletIds(lanelet_id)) {
    const auto lane = impl_->map.laneletLayer.get(conflict_id);
    if (laneDistance(lane, {x, y}) > 0.75) continue;
    const auto centerline = points(lane.centerline());
    if (centerline.size() < 2) continue;
    double nearest = std::numeric_limits<double>::infinity();
    double lane_yaw = 0.0;
    for (std::size_t i = 0; i + 1 < centerline.size(); ++i) {
      const auto distance = pointToSegment(
          {x, y}, centerline[i], centerline[i + 1]);
      if (distance < nearest) {
        nearest = distance;
        lane_yaw = std::atan2(
            centerline[i + 1].y - centerline[i].y,
            centerline[i + 1].x - centerline[i].x);
      }
    }
    if (headingDifference(yaw_rad, lane_yaw) > 0.785) continue;
    for (const auto &line : trafficLightStopLinesForLanelet(conflict_id)) {
      const CrosswalkPoint stop_mid{
          0.5 * (line.segment.x1 + line.segment.x2),
          0.5 * (line.segment.y1 + line.segment.y2)};
      const double distance_to_stopline =
          projectLength(centerline, stop_mid) -
          projectLength(centerline, {x, y});
      std::optional<double> crossing_distance;
      for (std::size_t i = 0; i + 1 < ego_centerline.size(); ++i) {
        const auto crossing = segmentIntersection(
            stop_mid, centerline.back(),
            ego_centerline[i], ego_centerline[i + 1]);
        if (crossing) {
          const double distance = std::hypot(
              crossing->x - stop_mid.x, crossing->y - stop_mid.y);
          if (!crossing_distance || distance < *crossing_distance) {
            crossing_distance = distance;
          }
        }
      }
      if (crossing_distance) {
        return IntersectionTrafficControlDistances{
            conflict_id, distance_to_stopline, *crossing_distance};
      }
    }
  }
  if (!isVehicleOnIntersectionAttentionLane(lanelet_id, x, y, yaw_rad)) {
    return std::nullopt;
  }
  std::optional<IntersectionTrafficControlDistances> nearest_control;
  for (const auto conflict_id :
       intersectionConflictingLaneletIds(lanelet_id)) {
    const auto centerline = laneCenterline(conflict_id);
    if (centerline.size() < 2) continue;
    for (const auto &line : trafficLightStopLinesForLanelet(conflict_id)) {
      const CrosswalkPoint stop_mid{
          0.5 * (line.segment.x1 + line.segment.x2),
          0.5 * (line.segment.y1 + line.segment.y2)};
      const double dx = stop_mid.x - x;
      const double dy = stop_mid.y - y;
      const double forward = dx * std::cos(yaw_rad) +
                             dy * std::sin(yaw_rad);
      const double lateral = std::abs(-dx * std::sin(yaw_rad) +
                                       dy * std::cos(yaw_rad));
      if (forward < 0.0 || forward > 200.0 || lateral > 2.5) continue;
      for (std::size_t index = 0; index + 1 < ego_centerline.size(); ++index) {
        const auto crossing = segmentIntersection(
            stop_mid, centerline.back(),
            ego_centerline[index], ego_centerline[index + 1]);
        if (!crossing) continue;
        const double to_ego = std::hypot(
            crossing->x - stop_mid.x, crossing->y - stop_mid.y);
        if (!nearest_control ||
            forward < nearest_control->distance_to_stopline_m) {
          nearest_control = IntersectionTrafficControlDistances{
              conflict_id, forward, to_ego};
        }
      }
    }
  }
  return nearest_control;
}

ScenarioPosition LaneletBinAdapter::scenarioPosition(
    double ego_x, double ego_y, double ego_yaw_rad,
    double goal_x, double goal_y,
    const std::vector<std::int64_t> &route_lanelet_ids) const {
  if (!std::isfinite(ego_x) || !std::isfinite(ego_y) ||
      !std::isfinite(ego_yaw_rad) || !std::isfinite(goal_x) ||
      !std::isfinite(goal_y)) {
    throw std::invalid_argument("시나리오 위치 좌표가 잘못되었습니다");
  }
  ScenarioPosition position;
  const auto ego = lanelet::BasicPoint2d(ego_x, ego_y);
  const auto goal = lanelet::BasicPoint2d(goal_x, goal_y);
  const auto nearest_ego = lanelet::geometry::findNearest(
      impl_->map.laneletLayer, ego, 1);
  const auto nearest_goal = lanelet::geometry::findNearest(
      impl_->map.laneletLayer, goal, 1);
  position.in_lane = !nearest_ego.empty() &&
      laneDistance(nearest_ego.front().second, {ego_x, ego_y}) < 0.01;
  position.goal_in_lane = !nearest_goal.empty() &&
      laneDistance(nearest_goal.front().second, {goal_x, goal_y}) < 0.01;
  const auto nearby_polygons = impl_->map.polygonLayer.search(
      lanelet::geometry::boundingBox2d(ego));
  for (const auto &parking : nearby_polygons) {
    if (parking.attributeOr("type", std::string{}) != "parking_lot") continue;
    std::vector<CrosswalkPoint> shape;
    for (const auto &point : parking) shape.push_back({point.x(), point.y()});
    if (strictlyWithinPolygon({ego_x, ego_y}, shape)) {
      position.in_parking_lot = true;
      break;
    }
  }
  double best_centerline_distance = std::numeric_limits<double>::infinity();
  for (const auto route_id : route_lanelet_ids) {
    if (!impl_->map.laneletLayer.exists(route_id)) continue;
    const auto centerline = impl_->map.laneletLayer.get(route_id).centerline();
    for (std::size_t index = 1; index < centerline.size(); ++index) {
      const CrosswalkPoint first{centerline[index - 1].x(),
                                 centerline[index - 1].y()};
      const CrosswalkPoint second{centerline[index].x(),
                                  centerline[index].y()};
      const double yaw = std::atan2(second.y - first.y,
                                    second.x - first.x);
      if (headingDifference(ego_yaw_rad, yaw) >= 0.7853981633974483) continue;
      best_centerline_distance = std::min(
          best_centerline_distance, pointToSegment({ego_x, ego_y}, first, second));
    }
  }
  position.along_lane = best_centerline_distance < 1.0;
  return position;
}

}  // planning 네임스페이스
