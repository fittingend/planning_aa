#include "planning/lanelet_osm_adapter.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

bool near(double a, double b) { return std::abs(a - b) < 1.0e-9; }
}  // 익명 네임스페이스

int main(int argc, char **argv) {
  if (argc < 2) return EXIT_FAILURE;
  const auto lines = planning::loadTrafficLightStopLines(argv[1], 20);
  check(lines.size() == 2 && near(lines[0].x1, 7.0) &&
            near(lines[0].y1, -1.0) && near(lines[0].y2, 0.0) &&
            near(lines[1].y2, 1.0),
        "다점 정지선을 순서대로 선분으로 변환합니다");
  const auto sign_lines = planning::loadStopSignStopLines(argv[1], 21);
  check(sign_lines.size() == 2 && near(sign_lines[0].y1, -1.0) &&
            near(sign_lines[1].y2, 1.0),
        "정지 표지와 연결된 다점 정지선을 읽습니다");
  const auto lane_signs = planning::loadStopSignStopLinesForLanelet(argv[1], 30);
  check(lane_signs.size() == 2 && lane_signs[0].regulatory_id == 21 &&
            near(lane_signs[1].segment.y2, 1.0),
        "차로에 연결된 정지 표지 규제 요소만 추출합니다");
  check(planning::loadLaneTurnDirection(argv[1], 30) ==
            planning::LaneTurnDirection::Right,
        "차로의 회전 방향을 읽습니다");
  bool rejected = false;
  try {
    planning::loadTrafficLightStopLines(argv[1], 999);
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  check(rejected, "없는 신호등 규제 요소를 거부합니다");
  rejected = false;
  try {
    planning::loadStopSignStopLines(argv[1], 20);
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  check(rejected, "신호등 규제 요소를 정지 표지로 해석하지 않습니다");
  if (argc >= 3) {
    const auto real_lines = planning::loadTrafficLightStopLines(argv[2], 18720);
    check(real_lines.size() == 1 &&
              near(real_lines[0].x1, 2590.5778) &&
              near(real_lines[0].y1, 23989.3724) &&
              near(real_lines[0].x2, 2602.5348) &&
              near(real_lines[0].y2, 23989.2223),
          "K-City 지도에서 기준 신호등 정지선을 추출합니다");
    const auto real_sign = planning::loadStopSignStopLines(argv[2], 18851);
    check(real_sign.size() == 1 &&
              near(real_sign[0].x1, 2544.3993) &&
              near(real_sign[0].y1, 24131.5639) &&
              near(real_sign[0].x2, 2547.4514) &&
              near(real_sign[0].y2, 24134.9212),
          "K-City 실제 정지 표지 규제 요소 18851의 정지선을 추출합니다");
    const auto real_lane_signs =
        planning::loadStopSignStopLinesForLanelet(argv[2], 15146);
    check(real_lane_signs.size() == 1 &&
              real_lane_signs[0].regulatory_id == 18851 &&
              near(real_lane_signs[0].segment.x1, 2544.3993),
          "K-City 차로 15146의 실제 정지 표지를 찾습니다");
  }
}
