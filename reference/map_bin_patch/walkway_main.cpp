// 기록된 이진 지도에서 횡단보도 한 곳을 비교용 보행로로 바꿉니다.

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/RegulatoryElement.h>
#include <lanelet2_io/io_handlers/Serialize.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <fstream>
#include <iostream>

int main(int argc, char **argv) {
  if (argc != 3 && argc != 4) return 2;
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
    auto lane = map.laneletLayer.get(24783);
    lane.setAttribute("subtype", "walkway");
    if (argc == 4) {
      lanelet::LineString3d stop_line(900000031, {
          lanelet::Point3d(900000032, 2590.0, 24109.0, 0.0),
          lanelet::Point3d(900000033, 2603.0, 24109.0, 0.0)});
      stop_line.setAttribute("type", "stop_line");
      stop_line.setAttribute("crosswalk_id", "24783");
      auto rule = std::make_shared<lanelet::GenericRegulatoryElement>(900000034);
      rule->setAttribute("type", "regulatory_element");
      rule->setAttribute("subtype", "road_marking");
      rule->addParameter(lanelet::RoleName::Refers, stop_line);
      lane.addRegulatoryElement(rule);
    }
    std::ofstream output(argv[2], std::ios::binary);
    if (!output) return 4;
    {
      boost::archive::binary_oarchive archive(output);
      archive << map;
      archive << next_id;
    }
    std::cout << "차로 " << lane.id() << "을 보행로로 변경\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 5;
  }
}
