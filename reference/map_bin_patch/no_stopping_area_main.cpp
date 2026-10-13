// K-City 경로에 비교용 정차 금지 구역과 정지선을 추가합니다.

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/RegulatoryElement.h>
#include <lanelet2_io/io_handlers/Serialize.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>

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
    lanelet::Polygon3d area(900000220, {
        lanelet::Point3d(900000221, 2587.0, 23982.0, 0.0),
        lanelet::Point3d(900000222, 2603.0, 23982.0, 0.0),
        lanelet::Point3d(900000223, 2603.0, 23991.0, 0.0),
        lanelet::Point3d(900000224, 2587.0, 23991.0, 0.0)});
    lanelet::LineString3d stop_line(900000225, {
        lanelet::Point3d(900000226, 2587.0, 23980.0, 0.0),
        lanelet::Point3d(900000227, 2603.0, 23980.0, 0.0)});
    auto rule = std::make_shared<lanelet::GenericRegulatoryElement>(900000228);
    rule->setAttribute("subtype", "no_stopping_area");
    rule->addParameter("refers", area);
    if (argc != 4 || std::string(argv[3]) != "none") {
      rule->addParameter("ref_line", stop_line);
    }
    map.add(rule);
    map.laneletLayer.get(24747).addRegulatoryElement(rule);
    std::ofstream output(argv[2], std::ios::binary);
    if (!output) return 4;
    {
      boost::archive::binary_oarchive archive(output);
      archive << map;
      archive << static_cast<lanelet::Id>(900000229);
    }
    std::cout << "차로 24747에 비교용 정차 금지 구역 추가\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 5;
  }
}
