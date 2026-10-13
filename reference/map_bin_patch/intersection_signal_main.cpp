// 기록된 교차로 신호등에 비교용 전구 선을 연결합니다.

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
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
    const auto rule = map.regulatoryElementLayer.get(25004);
    const auto light = std::dynamic_pointer_cast<lanelet::TrafficLight>(rule);
    if (!light) return 4;
    lanelet::LineString3d bulbs(900000043, {
        lanelet::Point3d(900000041, 2598.7, 23996.0, 0.0),
        lanelet::Point3d(900000042, 2599.8, 23996.0, 0.0)});
    bulbs.setAttribute("traffic_light_id", "25004");
    auto data = std::const_pointer_cast<lanelet::RegulatoryElementData>(
        light->constData());
    data->parameters["light_bulbs"].push_back(bulbs);
    std::ofstream output(argv[2], std::ios::binary);
    if (!output) return 5;
    {
      boost::archive::binary_oarchive archive(output);
      archive << map;
      archive << next_id;
    }
    std::cout << "교차로 신호등 " << light->id() << "에 전구 선 연결\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 6;
  }
}
