// K-City 목적지 부근에 비교용 주차 공간 표식을 추가합니다.

#include <lanelet2_core/LaneletMap.h>
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
    lanelet::LineString3d space(900000140, {
        lanelet::Point3d(900000141, 2566.0, 24006.878, 0.0),
        lanelet::Point3d(900000142, 2572.0, 24006.878, 0.0)});
    space.setAttribute("type", "parking_space");
    space.setAttribute("width", 3.0);
    map.add(space);
    std::ofstream output(argv[2], std::ios::binary);
    if (!output) return 4;
    {
      boost::archive::binary_oarchive archive(output);
      archive << map;
      archive << static_cast<lanelet::Id>(900000143);
    }
    std::cout << "목적지 부근에 비교용 주차 공간 추가\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 5;
  }
}
