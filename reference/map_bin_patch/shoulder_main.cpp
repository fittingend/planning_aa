// 목적지 근처에 방향이 지정된 비교용 갓길 차로를 추가합니다.

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
    lanelet::LineString3d left(900000151, {
        lanelet::Point3d(900000152, 2573.0, 24007.3, 0.0),
        lanelet::Point3d(900000153, 2565.0, 24007.3, 0.0)});
    lanelet::LineString3d right(900000154, {
        lanelet::Point3d(900000155, 2573.0, 24009.0, 0.0),
        lanelet::Point3d(900000156, 2565.0, 24009.0, 0.0)});
    lanelet::Lanelet shoulder(900000150, left, right);
    shoulder.setAttribute("subtype", "road_shoulder");
    map.add(shoulder);
    std::ofstream output(argv[2], std::ios::binary);
    if (!output) return 4;
    {
      boost::archive::binary_oarchive archive(output);
      archive << map;
      archive << static_cast<lanelet::Id>(900000157);
    }
    std::cout << "목적지 부근에 비교용 갓길 차로 추가\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 5;
  }
}
