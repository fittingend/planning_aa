// SPDX-License-Identifier: Apache-2.0
// 기록 경로와 예측 객체를 이진 지도 차로 밖 주행 후보 계산에 넣습니다.

#include "planning/lanelet_bin_adapter.hpp"

#include <cstdint>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 4) return 2;
  std::ifstream map_stream(argv[1], std::ios::binary);
  std::ifstream input(argv[2]);
  std::ofstream output(argv[3]);
  if (!map_stream || !input || !output) return 2;
  const std::vector<std::uint8_t> map_data(
      std::istreambuf_iterator<char>(map_stream), {});
  planning::LaneletBinAdapter map(map_data);
  std::size_t frame_count = 0;
  input >> frame_count;
  for (std::size_t frame = 0; frame < frame_count; ++frame) {
    std::size_t point_count = 0, object_count = 0, red_count = 0;
    double ego_x = 0.0, ego_y = 0.0;
    input >> point_count >> object_count >> red_count >> ego_x >> ego_y;
    std::vector<std::int64_t> red_ids(red_count);
    for (auto &id : red_ids) input >> id;
    planning::Trajectory path(point_count);
    std::vector<double> times(point_count);
    for (std::size_t i = 0; i < point_count; ++i)
      input >> path[i].x >> path[i].y >> path[i].orientation_z >>
          path[i].orientation_w >> path[i].longitudinal_velocity_mps >>
          times[i];
    std::vector<planning::OutOfLaneTrackedObject> objects(object_count);
    for (auto &object : objects) {
      std::size_t predicted_count = 0;
      int pedestrian = 0;
      input >> object.initial_position.x >> object.initial_position.y >>
          object.width >> pedestrian >> predicted_count;
      object.is_pedestrian = pedestrian != 0;
      for (std::size_t p = 0; p < predicted_count; ++p) {
        planning::OutOfLanePredictedPath prediction;
        std::size_t pose_count = 0;
        input >> prediction.confidence >> prediction.time_step >>
            prediction.object_length >> prediction.object_width >> pose_count;
        prediction.poses.resize(pose_count);
        for (auto &pose : prediction.poses)
          input >> pose.x >> pose.y >> pose.orientation_z >> pose.orientation_w;
        object.predicted_paths.push_back(std::move(prediction));
      }
    }
    if (!input || point_count < 2)
      throw std::runtime_error("차로 밖 주행 기록 입력이 잘못되었습니다");
    const auto candidate = map.findOutOfLaneMapCandidateFromTrackedObjects(
        path, times, {ego_x, ego_y}, objects, red_ids, 100.0, 0.0);
    const auto red_lines = red_ids.empty()
        ? std::vector<planning::OutOfLaneStopLine>{}
        : map.outOfLaneRedStopLines({ego_x, ego_y}, 100.0, red_ids);
    output << (candidate ? 1 : 0) << ' ' << red_lines.size() << '\n';
  }
}
