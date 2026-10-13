// K-City 기록 차로에 비교용 정지 표지와 정지선을 연결합니다.

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_io/Io.h>

#include <iostream>

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  try {
    auto map = lanelet::load(argv[1]);
    auto road = map->laneletLayer.get(24747);
    lanelet::LineString3d stop_line(900000003, {
      lanelet::Point3d(900000001, 2590.0, 23981.0, 0.0),
      lanelet::Point3d(900000002, 2602.0, 23981.0, 0.0)});
    lanelet::LineString3d sign(900000006, {
      lanelet::Point3d(900000004, 2590.0, 23982.0, 0.0),
      lanelet::Point3d(900000005, 2590.5, 23982.0, 0.0)});
    auto rule = lanelet::TrafficSign::make(
      900000007, {}, lanelet::TrafficSignsWithType{{sign}, "stop_sign"},
      {}, {stop_line});
    road.addRegulatoryElement(rule);
    lanelet::write(argv[2], *map);
    std::cout << "차로 " << road.id() << "에 합성 정지 표지 " << rule->id()
              << "와 정지선 " << stop_line.id() << " 연결\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 4;
  }
}
