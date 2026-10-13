// 감지 영역의 장애물 관측 이력과 정지선 속도 계획을 독립적으로 계산합니다.

#pragma once

#include "planning/crosswalk_core.hpp"
#include "planning/stop_line_core.hpp"

#include <optional>
#include <vector>

namespace planning {

struct DetectionAreaParameters {
  double state_clear_time_s = 2.0;
  double stop_margin_m = 0.0;
  double front_offset_m = 3.79;
  double stop_line_extension_m = 5.0;
  double distance_to_judge_over_stop_line_m = 0.5;
  bool linear_insert = false;
  bool copy_previous_orientation = false;
};

struct DetectionAreaResult {
  Trajectory trajectory;
  bool stopped = false;
  std::optional<double> stop_arc_m;
};

class DetectionAreaCore {
 public:
  DetectionAreaCore(
      StopLineSegment stop_line,
      std::vector<std::vector<CrosswalkPoint>> detection_areas,
      DetectionAreaParameters parameters = {});

  DetectionAreaResult apply(
      const Trajectory &path, double ego_x, double ego_y,
      double now_s, const std::vector<CrosswalkPoint> &obstacle_points);

 private:
  StopLineSegment stop_line_;
  std::vector<std::vector<CrosswalkPoint>> detection_areas_;
  DetectionAreaParameters parameters_;
  std::optional<double> last_obstacle_found_s_;
  bool stopped_ = false;
};

}  // planning 네임스페이스
