// Lanelet2 OSM 파일의 규제 요소와 차로 형상을 읽는 선택적 어댑터입니다.

#pragma once

#include "planning/stop_line_core.hpp"
#include "planning/crosswalk_core.hpp"
#include "planning/traffic_light_signal.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace planning {

std::vector<StopLineSegment> loadTrafficLightStopLines(
    const std::string &osm_path, std::int64_t regulatory_id);

std::vector<StopLineSegment> loadStopSignStopLines(
    const std::string &osm_path, std::int64_t regulatory_id);

struct StopSignStopLine {
  std::int64_t regulatory_id = 0;
  StopLineSegment segment;
};

std::vector<StopSignStopLine> loadStopSignStopLinesForLanelet(
    const std::string &osm_path, std::int64_t lanelet_id);

LaneTurnDirection loadLaneTurnDirection(
    const std::string &osm_path, std::int64_t lanelet_id);

std::vector<CrosswalkPoint> loadCrosswalkPolygon(
    const std::string &osm_path, std::int64_t lanelet_id);

struct CrosswalkBounds {
  std::vector<CrosswalkPoint> left;
  std::vector<CrosswalkPoint> right;
};

CrosswalkBounds loadCrosswalkBounds(
    const std::string &osm_path, std::int64_t lanelet_id);

}  // planning 네임스페이스
