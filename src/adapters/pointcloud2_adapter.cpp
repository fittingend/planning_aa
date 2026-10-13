// SPDX-License-Identifier: Apache-2.0
// PointCloud2의 조직화된 행 배열과 XYZ 실수 필드를 독립적으로 읽습니다.

#include "planning/pointcloud2_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace planning {
namespace {

bool hostBigEndian() {
  const std::uint16_t marker = 0x0102;
  return *reinterpret_cast<const std::uint8_t *>(&marker) == 0x01;
}

double readNumber(const std::uint8_t *bytes, std::uint8_t datatype,
                  bool big_endian) {
  const std::size_t size = datatype == 7 ? sizeof(float) : sizeof(double);
  std::uint8_t native[sizeof(double)]{};
  if (hostBigEndian() == big_endian)
    std::copy_n(bytes, size, native);
  else
    std::reverse_copy(bytes, bytes + size, native);
  if (datatype == 7) {
    float value = 0.0F;
    std::memcpy(&value, native, size);
    return value;
  }
  double value = 0.0;
  std::memcpy(&value, native, size);
  return value;
}

const PointcloudBinaryField &xyzField(
    const PointcloudBinaryFrame &frame, const char *name) {
  const auto found = std::find_if(frame.fields.begin(), frame.fields.end(),
      [&](const auto &field) { return field.name == name; });
  if (found == frame.fields.end() || found->count < 1 ||
      (found->datatype != 7 && found->datatype != 8))
    throw std::invalid_argument("점군 XYZ 필드 형식이 유효하지 않습니다");
  const std::uint64_t size = found->datatype == 7 ? 4 : 8;
  if (static_cast<std::uint64_t>(found->offset) + size > frame.point_step)
    throw std::invalid_argument("점군 XYZ 필드가 점 간격을 넘습니다");
  return *found;
}
}  // 익명 네임스페이스

std::vector<CruisePoint3D> decodePointcloud2XYZ(
    const PointcloudBinaryFrame &frame,
    const PointcloudRigidTransform &transform) {
  if (frame.width == 0 || frame.height == 0) return {};
  if (frame.point_step == 0 ||
      static_cast<std::uint64_t>(frame.width) * frame.point_step >
          frame.row_step ||
      static_cast<std::uint64_t>(frame.height) * frame.row_step >
          frame.data.size())
    throw std::invalid_argument("점군 행·점 간격 또는 자료 크기가 유효하지 않습니다");
  const auto &x_field = xyzField(frame, "x");
  const auto &y_field = xyzField(frame, "y");
  const auto &z_field = xyzField(frame, "z");
  for (double coefficient : transform.rotation)
    if (!std::isfinite(coefficient))
      throw std::invalid_argument("점군 좌표 변환 행렬이 유효하지 않습니다");
  for (double displacement : transform.translation)
    if (!std::isfinite(displacement))
      throw std::invalid_argument("점군 평행 이동이 유효하지 않습니다");
  std::vector<CruisePoint3D> points;
  const auto total = static_cast<std::uint64_t>(frame.width) * frame.height;
  if (total <= 1'000'000) points.reserve(static_cast<std::size_t>(total));
  for (std::uint32_t row = 0; row < frame.height; ++row) {
    const auto row_offset = static_cast<std::size_t>(row) * frame.row_step;
    for (std::uint32_t column = 0; column < frame.width; ++column) {
      const auto base = row_offset + static_cast<std::size_t>(column) *
                                    frame.point_step;
      const double x = readNumber(frame.data.data() + base + x_field.offset,
                                  x_field.datatype, frame.big_endian);
      const double y = readNumber(frame.data.data() + base + y_field.offset,
                                  y_field.datatype, frame.big_endian);
      const double z = readNumber(frame.data.data() + base + z_field.offset,
                                  z_field.datatype, frame.big_endian);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      const auto &r = transform.rotation;
      const auto &t = transform.translation;
      points.push_back({r[0] * x + r[1] * y + r[2] * z + t[0],
                        r[3] * x + r[4] * y + r[5] * z + t[1],
                        r[6] * x + r[7] * y + r[8] * z + t[2]});
    }
  }
  return points;
}

}  // planning 네임스페이스
