// Lanelet2 OSM 파일에서 독립 코어에 필요한 입력만 추출합니다.

#include "planning/lanelet_osm_adapter.hpp"

#include <tinyxml2.h>

#include <stdexcept>
#include <string>
#include <vector>
#include <algorithm>

namespace planning {
namespace {

void openMap(const std::string &path, tinyxml2::XMLDocument &document) {
  if (document.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Lanelet2 OSM 파일을 읽을 수 없습니다: " + path);
  }
  if (!document.FirstChildElement("osm")) {
    throw std::runtime_error("Lanelet2 OSM 최상위 요소가 없습니다");
  }
}

const tinyxml2::XMLElement *findById(
    const tinyxml2::XMLElement *osm, const char *name, std::int64_t id) {
  for (auto *element = osm->FirstChildElement(name); element;
       element = element->NextSiblingElement(name)) {
    std::int64_t current = 0;
    if (element->QueryInt64Attribute("id", &current) == tinyxml2::XML_SUCCESS &&
        current == id) return element;
  }
  return nullptr;
}

std::string tagValue(const tinyxml2::XMLElement *element, const char *key) {
  for (auto *tag = element->FirstChildElement("tag"); tag;
       tag = tag->NextSiblingElement("tag")) {
    const auto *tag_key = tag->Attribute("k");
    if (tag_key && std::string(tag_key) == key) {
      const auto *value = tag->Attribute("v");
      return value ? value : "";
    }
  }
  return "";
}

struct MapPoint {
  double x;
  double y;
};

MapPoint localPoint(const tinyxml2::XMLElement *osm, std::int64_t id) {
  const auto *node = findById(osm, "node", id);
  if (!node) throw std::runtime_error("정지선 노드가 지도에 없습니다");
  const auto x = tagValue(node, "local_x");
  const auto y = tagValue(node, "local_y");
  if (x.empty() || y.empty()) {
    throw std::runtime_error("정지선 노드에 local_x 또는 local_y가 없습니다");
  }
  return {std::stod(x), std::stod(y)};
}

std::vector<StopLineSegment> referenceLineSegments(
    const tinyxml2::XMLElement *osm, const tinyxml2::XMLElement *relation,
    const std::string &kind) {
  std::vector<StopLineSegment> segments;
  for (auto *member = relation->FirstChildElement("member"); member;
       member = member->NextSiblingElement("member")) {
    const auto *role = member->Attribute("role");
    const auto *type = member->Attribute("type");
    if (!role || !type || std::string(role) != "ref_line" ||
        std::string(type) != "way") continue;
    std::int64_t way_id = 0;
    if (member->QueryInt64Attribute("ref", &way_id) != tinyxml2::XML_SUCCESS) {
      throw std::runtime_error(kind + " 정지선 참조가 잘못되었습니다");
    }
    const auto *way = findById(osm, "way", way_id);
    if (!way) throw std::runtime_error(kind + " 정지선이 지도에 없습니다");
    std::vector<MapPoint> points;
    for (auto *nd = way->FirstChildElement("nd"); nd;
         nd = nd->NextSiblingElement("nd")) {
      std::int64_t node_id = 0;
      if (nd->QueryInt64Attribute("ref", &node_id) != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error(kind + " 정지선 노드 참조가 잘못되었습니다");
      }
      points.push_back(localPoint(osm, node_id));
    }
    for (std::size_t i = 1; i < points.size(); ++i) {
      segments.push_back({points[i - 1].x, points[i - 1].y,
                          points[i].x, points[i].y});
    }
  }
  if (segments.empty()) throw std::runtime_error(kind + " 정지선에 선분이 없습니다");
  return segments;
}

bool containsStopSign(
    const tinyxml2::XMLElement *osm, const tinyxml2::XMLElement *relation) {
  bool has_stop_sign = false;
  for (auto *member = relation->FirstChildElement("member"); member;
       member = member->NextSiblingElement("member")) {
    const auto *role = member->Attribute("role");
    const auto *type = member->Attribute("type");
    if (!role || !type || std::string(role) != "refers" ||
        std::string(type) != "way") continue;
    std::int64_t sign_id = 0;
    if (member->QueryInt64Attribute("ref", &sign_id) != tinyxml2::XML_SUCCESS) {
      throw std::runtime_error("정지 표지 참조가 잘못되었습니다");
    }
    const auto *sign = findById(osm, "way", sign_id);
    if (!sign) throw std::runtime_error("정지 표지가 지도에 없습니다");
    if (tagValue(sign, "type") == "traffic_sign" &&
        tagValue(sign, "subtype") == "stop_sign") has_stop_sign = true;
  }
  return has_stop_sign;
}

std::vector<StopLineSegment> stopSignSegments(
    const tinyxml2::XMLElement *osm, const tinyxml2::XMLElement *relation) {
  if (!relation || tagValue(relation, "type") != "regulatory_element" ||
      tagValue(relation, "subtype") != "traffic_sign") {
    throw std::runtime_error("정지 표지 규제 요소를 찾지 못했습니다");
  }
  const bool has_stop_sign = containsStopSign(osm, relation);
  if (!has_stop_sign) throw std::runtime_error("정지 표지가 연결되지 않았습니다");
  return referenceLineSegments(osm, relation, "정지 표지");
}

}  // 익명 네임스페이스

std::vector<StopLineSegment> loadTrafficLightStopLines(
    const std::string &osm_path, std::int64_t regulatory_id) {
  tinyxml2::XMLDocument document;
  openMap(osm_path, document);
  const auto *osm = document.FirstChildElement("osm");
  const auto *relation = findById(osm, "relation", regulatory_id);
  if (!relation || tagValue(relation, "type") != "regulatory_element" ||
      tagValue(relation, "subtype") != "traffic_light") {
    throw std::runtime_error("신호등 규제 요소를 찾지 못했습니다");
  }
  return referenceLineSegments(osm, relation, "신호등");
}

std::vector<StopLineSegment> loadStopSignStopLines(
    const std::string &osm_path, std::int64_t regulatory_id) {
  tinyxml2::XMLDocument document;
  openMap(osm_path, document);
  const auto *osm = document.FirstChildElement("osm");
  const auto *relation = findById(osm, "relation", regulatory_id);
  return stopSignSegments(osm, relation);
}

std::vector<StopSignStopLine> loadStopSignStopLinesForLanelet(
    const std::string &osm_path, std::int64_t lanelet_id) {
  tinyxml2::XMLDocument document;
  openMap(osm_path, document);
  const auto *osm = document.FirstChildElement("osm");
  const auto *lanelet = findById(osm, "relation", lanelet_id);
  if (!lanelet || tagValue(lanelet, "type") != "lanelet") {
    throw std::runtime_error("정지 표지 차로를 찾지 못했습니다");
  }
  std::vector<StopSignStopLine> result;
  for (auto *member = lanelet->FirstChildElement("member"); member;
       member = member->NextSiblingElement("member")) {
    const auto *role = member->Attribute("role");
    const auto *type = member->Attribute("type");
    if (!role || !type || std::string(role) != "regulatory_element" ||
        std::string(type) != "relation") continue;
    std::int64_t regulatory_id = 0;
    if (member->QueryInt64Attribute("ref", &regulatory_id) != tinyxml2::XML_SUCCESS) {
      throw std::runtime_error("차로의 규제 요소 참조가 잘못되었습니다");
    }
    const auto *relation = findById(osm, "relation", regulatory_id);
    if (!relation) throw std::runtime_error("차로의 규제 요소가 지도에 없습니다");
    if (tagValue(relation, "type") != "regulatory_element" ||
        tagValue(relation, "subtype") != "traffic_sign") continue;
    if (!containsStopSign(osm, relation)) continue;
    for (const auto &segment : referenceLineSegments(osm, relation, "정지 표지")) {
      result.push_back({regulatory_id, segment});
    }
  }
  return result;
}

LaneTurnDirection loadLaneTurnDirection(
    const std::string &osm_path, std::int64_t lanelet_id) {
  tinyxml2::XMLDocument document;
  openMap(osm_path, document);
  const auto *relation = findById(document.FirstChildElement("osm"),
                                  "relation", lanelet_id);
  if (!relation || tagValue(relation, "type") != "lanelet") {
    throw std::runtime_error("차로를 찾지 못했습니다");
  }
  const auto direction = tagValue(relation, "turn_direction");
  if (direction == "left") return LaneTurnDirection::Left;
  if (direction == "right") return LaneTurnDirection::Right;
  if (direction == "straight") return LaneTurnDirection::Straight;
  return LaneTurnDirection::Else;
}

CrosswalkBounds loadCrosswalkBounds(
    const std::string &osm_path, std::int64_t lanelet_id) {
  tinyxml2::XMLDocument document;
  openMap(osm_path, document);
  const auto *osm = document.FirstChildElement("osm");
  const auto *relation = findById(osm, "relation", lanelet_id);
  if (!relation || tagValue(relation, "type") != "lanelet" ||
      tagValue(relation, "subtype") != "crosswalk") {
    throw std::runtime_error("횡단보도 차로를 찾지 못했습니다");
  }
  auto boundary = [osm, relation](const char *role) {
    for (auto *member = relation->FirstChildElement("member"); member;
         member = member->NextSiblingElement("member")) {
      const auto *member_role = member->Attribute("role");
      const auto *member_type = member->Attribute("type");
      if (!member_role || !member_type || std::string(member_role) != role ||
          std::string(member_type) != "way") continue;
      std::int64_t way_id = 0;
      if (member->QueryInt64Attribute("ref", &way_id) != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("횡단보도 경계 참조가 잘못되었습니다");
      }
      const auto *way = findById(osm, "way", way_id);
      if (!way) throw std::runtime_error("횡단보도 경계가 지도에 없습니다");
      std::vector<CrosswalkPoint> points;
      for (auto *nd = way->FirstChildElement("nd"); nd;
           nd = nd->NextSiblingElement("nd")) {
        std::int64_t node_id = 0;
        if (nd->QueryInt64Attribute("ref", &node_id) != tinyxml2::XML_SUCCESS) {
          throw std::runtime_error("횡단보도 노드 참조가 잘못되었습니다");
        }
        const auto point = localPoint(osm, node_id);
        points.push_back({point.x, point.y});
      }
      if (points.size() < 2) throw std::runtime_error("횡단보도 경계가 짧습니다");
      return points;
    }
    throw std::runtime_error("횡단보도 경계가 없습니다");
  };
  return {boundary("left"), boundary("right")};
}

std::vector<CrosswalkPoint> loadCrosswalkPolygon(
    const std::string &osm_path, std::int64_t lanelet_id) {
  auto bounds = loadCrosswalkBounds(osm_path, lanelet_id);
  std::reverse(bounds.right.begin(), bounds.right.end());
  bounds.left.insert(bounds.left.end(),
                     bounds.right.begin(), bounds.right.end());
  return bounds.left;
}

}  // planning 네임스페이스
