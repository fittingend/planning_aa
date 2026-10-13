// 이진 지도에서 지정한 차로의 회전 속성과 중심선 끝점을 확인합니다.

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/io_handlers/Serialize.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/geometry.hpp>

#include <fstream>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
  if (argc < 3) return 2;
  std::ifstream input(argv[1], std::ios::binary);
  if (!input) return 3;
  lanelet::LaneletMap map;
  lanelet::Id next_id = 0;
  {
    boost::archive::binary_iarchive archive(input);
    archive >> map;
    archive >> next_id;
  }
  if (std::string(argv[2]) == "--point" && argc == 5) {
    const boost::geometry::model::d2::point_xy<double> point{
        std::stod(argv[3]), std::stod(argv[4])};
    for (const auto &lane : map.laneletLayer) {
      boost::geometry::model::polygon<
          boost::geometry::model::d2::point_xy<double>> shape;
      for (const auto &corner : lane.polygon2d()) {
        boost::geometry::append(shape.outer(),
                                boost::geometry::model::d2::point_xy<double>{
                                    corner.x(), corner.y()});
      }
      boost::geometry::correct(shape);
      if (!boost::geometry::within(point, shape)) continue;
      std::cout << lane.id() << ' '
                << lane.attributeOr("turn_direction", "else") << '\n';
    }
    return 0;
  }
  for (int index = 2; index < argc; ++index) {
    const auto id = static_cast<lanelet::Id>(std::stoll(argv[index]));
    const auto lane = map.laneletLayer.get(id);
    const auto center = lane.centerline();
    std::cout << id << ' ' << lane.attributeOr("turn_direction", "else")
              << ' ' << center.front().x() << ' ' << center.front().y()
              << ' ' << center.back().x() << ' ' << center.back().y()
              << '\n';
  }
}
