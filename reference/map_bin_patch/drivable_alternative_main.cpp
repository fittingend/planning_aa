// 비교용 주행 금지 차로와 같은 연결을 가진 대체 차로를 추가합니다.

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
    auto original = map.laneletLayer.get(25001);
    lanelet::Lanelet alternate(900000210,
                              original.leftBound(), original.rightBound());
    alternate.setAttribute("subtype", "road");
    alternate.setAttribute("no_drivable_lane", "yes");
    map.add(alternate);
    std::ofstream output(argv[2], std::ios::binary);
    if (!output) return 4;
    {
      boost::archive::binary_oarchive archive(output);
      archive << map;
      archive << static_cast<lanelet::Id>(900000211);
    }
    std::cout << "주행 금지 차로와 대체 차로 추가\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 5;
  }
}
