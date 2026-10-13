// K-City 교차로 차로를 비교용 사유 도로로 표시합니다.

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/io_handlers/Serialize.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <fstream>
#include <iostream>
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
    map.laneletLayer.get(25001).setAttribute("location", "private");
    if (argc == 4 && std::string(argv[3]) == "both") {
      map.laneletLayer.get(24857).setAttribute("location", "private");
    }
    std::ofstream output(argv[2], std::ios::binary);
    if (!output) return 4;
    {
      boost::archive::binary_oarchive archive(output);
      archive << map;
      archive << next_id;
    }
    std::cout << "차로 25001을 비교용 사유 도로로 표시\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 5;
  }
}
