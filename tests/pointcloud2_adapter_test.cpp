// SPDX-License-Identifier: Apache-2.0
// 점군 행 여백·바이트 순서·무효 점·좌표 변환을 확인합니다.

#include "planning/pointcloud2_adapter.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace {
void writeFloat(std::vector<std::uint8_t> &data, std::size_t offset,
                float value, bool big_endian) {
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  for (std::size_t i = 0; i < 4; ++i) {
    const auto shift = static_cast<unsigned>((big_endian ? 3 - i : i) * 8);
    data[offset + i] = static_cast<std::uint8_t>((bits >> shift) & 0xffU);
  }
}

void writeDouble(std::vector<std::uint8_t> &data, std::size_t offset,
                 double value, bool big_endian) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  for (std::size_t i = 0; i < 8; ++i) {
    const auto shift = static_cast<unsigned>((big_endian ? 7 - i : i) * 8);
    data[offset + i] = static_cast<std::uint8_t>((bits >> shift) & 0xffU);
  }
}

void require(bool condition) {
  if (!condition) throw std::runtime_error("PointCloud2 입력 어댑터 검사 실패");
}
}  // 익명 네임스페이스

int main() {
  using namespace planning;
  PointcloudBinaryFrame frame;
  frame.width = 2;
  frame.height = 2;
  frame.point_step = 12;
  frame.row_step = 32;
  frame.fields = {{"x", 0, 7, 1}, {"y", 4, 7, 1}, {"z", 8, 7, 1}};
  frame.data.resize(64, 0);
  for (const auto [base, x, y, z] : {
           std::array<float, 4>{0, 1, 2, 3},
           std::array<float, 4>{12, 4, 5, 6},
           std::array<float, 4>{32, 7, 8, 9},
           std::array<float, 4>{44, std::numeric_limits<float>::quiet_NaN(), 0, 0}}) {
    writeFloat(frame.data, static_cast<std::size_t>(base), x, false);
    writeFloat(frame.data, static_cast<std::size_t>(base) + 4, y, false);
    writeFloat(frame.data, static_cast<std::size_t>(base) + 8, z, false);
  }
  PointcloudRigidTransform transform;
  transform.rotation[0] = 0.0;
  transform.rotation[1] = -1.0;
  transform.rotation[3] = 1.0;
  transform.rotation[4] = 0.0;
  transform.translation[0] = 10.0;
  const auto points = decodePointcloud2XYZ(frame, transform);
  require(points.size() == 3);
  require(std::abs(points[0].x - 8.0) < 1e-9);
  require(std::abs(points[0].y - 1.0) < 1e-9);
  require(std::abs(points[2].x - 2.0) < 1e-9);
  require(std::abs(points[2].z - 9.0) < 1e-9);

  PointcloudBinaryFrame big;
  big.width = big.height = 1;
  big.point_step = big.row_step = 24;
  big.big_endian = true;
  big.fields = {{"x", 0, 8, 1}, {"y", 8, 8, 1}, {"z", 16, 8, 1}};
  big.data.resize(24);
  writeDouble(big.data, 0, -1.25, true);
  writeDouble(big.data, 8, 2.5, true);
  writeDouble(big.data, 16, 3.75, true);
  const auto decoded = decodePointcloud2XYZ(big);
  require(decoded.size() == 1);
  require(std::abs(decoded[0].x + 1.25) < 1e-12);
  require(std::abs(decoded[0].y - 2.5) < 1e-12);
  big.data.pop_back();
  bool rejected = false;
  try { (void)decodePointcloud2XYZ(big); }
  catch (const std::invalid_argument &) { rejected = true; }
  require(rejected);
}
