// K-City 교차로 차로에서 신호등 연결을 제거해 무신호 가림 판정을 비교합니다.

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_io/io_handlers/Serialize.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <fstream>
#include <iostream>

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  try {
    std::ifstream input(argv[1], std::ios::binary);
    if (!input) return 3;
    lanelet::LaneletMap map;
    lanelet::Id next_id = 0;
    {
      boost::archive::binary_iarchive archive(input);
      archive >> map;
      archive >> next_id;
    }
    auto lane = map.laneletLayer.get(25001);
    for (const auto &light : lane.regulatoryElementsAs<lanelet::TrafficLight>()) {
      lane.removeRegulatoryElement(light);
    }
    std::ofstream output(argv[2], std::ios::binary);
    if (!output) return 4;
    {
      boost::archive::binary_oarchive archive(output);
      archive << map;
      archive << next_id;
    }
    std::cout << "교차로 차로 25001의 신호등 연결 제거\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 5;
  }
}
