#include "planning/stop_line_core.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {
void check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

bool near(double a, double b) { return std::abs(a - b) < 1.0e-9; }
}  // 익명 네임스페이스

int main() {
  planning::Trajectory path;
  for (int x = 0; x <= 10; ++x) {
    path.push_back({static_cast<double>(x), 0.0, 5.0F, 0.0F});
  }
  planning::StopLineParameters parameters;
  parameters.stop_margin_m = 0.5;
  parameters.front_offset_m = 1.0;
  parameters.stop_duration_s = 2.0;
  parameters.hold_stop_margin_m = 0.5;
  planning::StopLineCore core({7.0, -1.0, 7.0, 1.0}, parameters);

  auto result = core.apply(path, 5.0, 1.0, false, 0.0);
  check(result.stop_arc_m && near(*result.stop_arc_m, 5.5) &&
            near(result.ego_arc_m, 5.0),
        "원본 테스트처럼 교차점 7m에서 여유 0.5m와 앞부분 1m를 뺍니다");
  check(result.trajectory.size() == 12 &&
            near(result.trajectory[6].x, 5.5) &&
            result.trajectory[5].longitudinal_velocity_mps == 5.0F &&
            result.trajectory[6].longitudinal_velocity_mps == 0.0F &&
            result.trajectory.back().longitudinal_velocity_mps == 0.0F,
        "정지점부터 속도를 0으로 제한하고 경계점을 삽입합니다");

  result = core.apply(path, 5.4, 0.0, true, 1.0);
  check(result.state == planning::StopLineState::Stopped,
        "정지점 가까이 정차하면 정차 상태로 전환합니다");
  result = core.apply(path, 5.4, 0.0, true, 3.0);
  check(result.state == planning::StopLineState::Stopped &&
            result.stop_arc_m && near(*result.stop_arc_m, 5.4),
        "정차 상태에서는 현재 위치를 정지점으로 사용하고 정확히 2초에는 유지합니다");
  result = core.apply(path, 5.4, 0.0, true, 3.1);
  check(result.state == planning::StopLineState::Start && result.stop_arc_m,
        "정차 시간이 2초를 넘는 콜백까지 속도 0을 적용합니다");
  result = core.apply(path, 5.4, 0.0, false, 3.2);
  check(!result.stop_arc_m && result.trajectory.size() == path.size() &&
            result.trajectory[6].longitudinal_velocity_mps == 5.0F,
        "출발 상태에서는 정지선을 다시 적용하지 않습니다");

  planning::StopLineCore no_crossing({20.0, -1.0, 20.0, 1.0}, parameters);
  result = no_crossing.apply(path, 5.0, 0.0, false, 0.0);
  check(!result.stop_arc_m && result.trajectory.size() == path.size(),
        "경로와 만나지 않는 정지선은 궤적을 바꾸지 않습니다");

  planning::StopLineCore extended({7.0, 2.0, 7.0, 3.0}, parameters);
  result = extended.apply(path, 5.0, 0.0, false, 0.0);
  check(result.stop_arc_m && near(*result.stop_arc_m, 5.5),
        "정지선 양끝을 연장해 경로와의 교차점을 찾습니다");

  planning::StopLineCore behind_start({1.0, -1.0, 1.0, 1.0}, parameters);
  result = behind_start.apply(path, 0.0, 0.0, false, 0.0);
  check(!result.stop_arc_m && result.trajectory.size() == path.size(),
        "계산한 정지점이 경로 시작 전이면 변경하지 않습니다");

  const planning::Trajectory curve{{0.0, 0.0, 5.0F, 0.0F},
                                   {1.0, 0.0, 5.0F, 0.0F},
                                   {2.0, 1.0, 5.0F, 0.0F},
                                   {3.0, 1.0, 5.0F, 0.0F}};
  planning::StopLineParameters curve_parameters;
  curve_parameters.front_offset_m = 0.0;
  planning::StopLineCore curved({2.5, -1.0, 2.5, 2.0}, curve_parameters);
  result = curved.apply(curve, 0.0, 0.0, false, 0.0);
  check(result.stop_arc_m &&
            near(*result.stop_arc_m, 1.0 + std::sqrt(2.0) + 0.5) &&
            result.trajectory.size() == 5 &&
            std::abs(result.trajectory[3].x - 2.467830085889911) < 1.0e-9 &&
            std::abs(result.trajectory[3].y - 1.0776650429449552) < 1.0e-9 &&
            result.trajectory[3].longitudinal_velocity_mps == 0.0F,
        "곡선 삽입점은 원본과 같은 자연 3차 스플라인으로 계산합니다");
}
