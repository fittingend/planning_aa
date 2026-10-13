// K-City 정지선 기록의 차량 위치에 비교용 주차장 다각형을 추가합니다.

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/io_handlers/Serialize.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <fstream>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
  if (argc != 3 && argc != 7) return 2;
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
    const double x1 = argc == 7 ? std::stod(argv[3]) : 2582.0;
    const double y1 = argc == 7 ? std::stod(argv[4]) : 23939.0;
    const double x2 = argc == 7 ? std::stod(argv[5]) : 2588.0;
    const double y2 = argc == 7 ? std::stod(argv[6]) : 23946.0;
    lanelet::Polygon3d lot(900000080, {
        lanelet::Point3d(900000081, x1, y1, 0.0),
        lanelet::Point3d(900000082, x2, y1, 0.0),
        lanelet::Point3d(900000083, x2, y2, 0.0),
        lanelet::Point3d(900000084, x1, y2, 0.0)});
    lot.setAttribute("type", "parking_lot");
    map.add(lot);
    std::ofstream output(argv[2], std::ios::binary);
    if (!output) return 4;
    {
      boost::archive::binary_oarchive archive(output);
      archive << map;
      archive << static_cast<lanelet::Id>(900000085);
    }
    std::cout << "K-City 차량 위치에 비교용 주차장 다각형 추가\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 5;
  }
}
