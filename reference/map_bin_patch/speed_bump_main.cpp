// K-City 정지선 경로에 비교용 속도 방지턱 다각형을 추가합니다.

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
  if (argc < 3 || argc > 5) return 2;
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
    lanelet::Polygon3d bump(900000120, {
        lanelet::Point3d(900000121, 2588.0, 23980.0, 0.0),
        lanelet::Point3d(900000122, 2602.0, 23980.0, 0.0),
        lanelet::Point3d(900000123, 2602.0, 23984.0, 0.0),
        lanelet::Point3d(900000124, 2588.0, 23984.0, 0.0)});
    if (argc == 3) bump.setAttribute("height", 0.2);
    else if (std::string(argv[3]) != "none") {
      bump.setAttribute("height", std::stod(argv[3]));
    }
    if (argc == 5) bump.setAttribute("slow_down_speed", std::stod(argv[4]));
    auto rule = std::make_shared<lanelet::GenericRegulatoryElement>(900000125);
    rule->setAttribute("subtype", "speed_bump");
    rule->addParameter("refers", bump);
    map.add(rule);
    map.laneletLayer.get(24747).addRegulatoryElement(rule);
    std::ofstream output(argv[2], std::ios::binary);
    if (!output) return 4;
    {
      boost::archive::binary_oarchive archive(output);
      archive << map;
      archive << static_cast<lanelet::Id>(900000126);
    }
    std::cout << "차로 24747에 비교용 속도 방지턱 추가\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 5;
  }
}
