// SPDX-License-Identifier: Apache-2.0
// rosbag에서 추출한 원시 점군 바이트의 XYZ 통계를 독립 어댑터로 계산합니다.

#include "planning/pointcloud2_adapter.hpp"

#include <cstdint>
#include <fstream>
#include <stdexcept>

namespace {
std::uint32_t read32(std::ifstream &stream) {
  std::uint8_t bytes[4]{};
  stream.read(reinterpret_cast<char *>(bytes), 4);
  if (!stream) throw std::runtime_error("점군 재생 입력이 중간에서 끝났습니다");
  return static_cast<std::uint32_t>(bytes[0]) |
         (static_cast<std::uint32_t>(bytes[1]) << 8) |
         (static_cast<std::uint32_t>(bytes[2]) << 16) |
         (static_cast<std::uint32_t>(bytes[3]) << 24);
}
}  // 익명 네임스페이스

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  std::ifstream input(argv[1], std::ios::binary);
  std::ofstream output(argv[2]);
  if (!input || !output) return 2;
  const auto frame_count = read32(input);
  for (std::uint32_t i = 0; i < frame_count; ++i) {
    planning::PointcloudBinaryFrame frame;
    frame.width = read32(input);
    frame.height = read32(input);
    frame.point_step = read32(input);
    frame.row_step = read32(input);
    frame.big_endian = read32(input) != 0;
    frame.fields = {{"x", read32(input), static_cast<std::uint8_t>(read32(input)), 1},
                    {"y", read32(input), static_cast<std::uint8_t>(read32(input)), 1},
                    {"z", read32(input), static_cast<std::uint8_t>(read32(input)), 1}};
    const auto size = read32(input);
    frame.data.resize(size);
    input.read(reinterpret_cast<char *>(frame.data.data()), size);
    if (!input) throw std::runtime_error("점군 재생 바이트가 부족합니다");
    const auto points = planning::decodePointcloud2XYZ(frame);
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    for (const auto &point : points) {
      sum_x += point.x;
      sum_y += point.y;
      sum_z += point.z;
    }
    output.precision(17);
    output << points.size() << ' ' << sum_x << ' ' << sum_y << ' ' <<
        sum_z << '\n';
  }
}
