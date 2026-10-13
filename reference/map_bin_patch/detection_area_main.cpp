// K-City 경로에 비교용 감지 영역과 정지선을 연결합니다.

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/RegulatoryElement.h>
#include <lanelet2_io/io_handlers/Serialize.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <fstream>
#include <iostream>
#include <memory>

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
    lanelet::Polygon3d area(900000100, {
        lanelet::Point3d(900000101, 2587.0, 23982.0, 0.0),
        lanelet::Point3d(900000102, 2601.0, 23982.0, 0.0),
        lanelet::Point3d(900000103, 2601.0, 23996.0, 0.0),
        lanelet::Point3d(900000104, 2587.0, 23996.0, 0.0)});
    lanelet::LineString3d stopline(900000105, {
        lanelet::Point3d(900000106, 2588.0, 23981.0, 0.0),
        lanelet::Point3d(900000107, 2602.0, 23981.0, 0.0)});
    auto rule = std::make_shared<lanelet::GenericRegulatoryElement>(900000108);
    rule->setAttribute("subtype", "detection_area");
    rule->addParameter("refers", area);
    rule->addParameter("ref_line", stopline);
    map.add(rule);
    map.laneletLayer.get(24747).addRegulatoryElement(rule);
    std::ofstream output(argv[2], std::ios::binary);
    if (!output) return 4;
    {
      boost::archive::binary_oarchive archive(output);
      archive << map;
      archive << static_cast<lanelet::Id>(900000109);
    }
    std::cout << "차로 24747에 비교용 감지 영역 추가\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 5;
  }
}
