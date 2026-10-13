// 교차 차량 차로에 신호등 정지선을 추가해 제어하 충돌 판단을 비교합니다.

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
    lanelet::LineString3d stopline(900000061, {
        lanelet::Point3d(900000062, 2614.8, 24005.2, 0.0),
        lanelet::Point3d(900000063, 2614.8, 24009.4, 0.0)});
    lanelet::LineString3d lamp(900000064, {
        lanelet::Point3d(900000065, 2615.2, 24010.0, 0.0),
        lanelet::Point3d(900000066, 2616.2, 24010.0, 0.0)});
    lamp.setAttribute("traffic_light_id", "900000060");
    const auto light = lanelet::TrafficLight::make(
        900000060, {{"subtype", "traffic_light"}}, {lamp}, stopline);
    map.add(light);
    map.laneletLayer.get(24868).addRegulatoryElement(light);
    map.laneletLayer.get(24869).addRegulatoryElement(light);
    std::ofstream output(argv[2], std::ios::binary);
    if (!output) return 4;
    {
      boost::archive::binary_oarchive archive(output);
      archive << map;
      archive << static_cast<lanelet::Id>(900000070);
    }
    std::cout << "교차 차량 차로 24868·24869에 비교용 정지선 연결\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 5;
  }
}
