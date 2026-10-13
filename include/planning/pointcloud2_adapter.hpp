#pragma once

// ROS 자료형 없이 PointCloud2의 XYZ 바이너리 배열을 해석하는 입력 어댑터입니다.

#include "planning/obstacle_cruise_pointcloud.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace planning {

struct PointcloudBinaryField {
  std::string name;
  std::uint32_t offset = 0;
  // PointField의 FLOAT32는 7, FLOAT64는 8입니다.
  std::uint8_t datatype = 7;
  std::uint32_t count = 1;
};

struct PointcloudBinaryFrame {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t point_step = 0;
  std::uint32_t row_step = 0;
  bool big_endian = false;
  std::vector<PointcloudBinaryField> fields;
  std::vector<std::uint8_t> data;
};

struct PointcloudRigidTransform {
  // 행 우선 순서의 회전 행렬과 미터 단위 평행 이동입니다.
  double rotation[9] = {1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0};
  double translation[3] = {0.0, 0.0, 0.0};
};

// 유효하지 않은 값의 점은 버리고, 유효한 점에 지정한 좌표 변환을 적용합니다.
std::vector<CruisePoint3D> decodePointcloud2XYZ(
    const PointcloudBinaryFrame &frame,
    const PointcloudRigidTransform &transform = {});

}  // planning 네임스페이스
