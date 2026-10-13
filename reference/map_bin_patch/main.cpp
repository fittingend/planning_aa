// rosbag 지도의 횡단보도 24783에 기존 신호등 25005를 연결합니다.

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_io/Io.h>

#include <iostream>
#include <memory>

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  try {
    auto map = lanelet::load(argv[1]);
    auto crosswalk = map->laneletLayer.get(24783);
    const auto regulatory = map->regulatoryElementLayer.get(25005);
    const auto light = std::dynamic_pointer_cast<lanelet::TrafficLight>(regulatory);
    if (!light) return 3;
    crosswalk.addRegulatoryElement(light);
    lanelet::write(argv[2], *map);
    std::cout << "횡단보도 " << crosswalk.id() << "에 신호등 "
              << light->id() << " 연결\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 4;
  }
}
