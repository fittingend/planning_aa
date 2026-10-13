#include "planning/traffic_light_core.hpp"

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
  planning::TrafficLightParameters parameters;
  parameters.front_offset_m = 1.0;
  parameters.stop_margin_m = 0.5;
  planning::TrafficLightCore core({7.0, -1.0, 7.0, 1.0}, parameters);
  planning::TrafficLightInput input;
  input.ego_x = 2.0;
  input.ego_velocity_mps = 1.0;

  auto result = core.apply(path, input);
  check(result.stop_arc_m && near(*result.stop_arc_m, 5.5) &&
            near(result.distance_to_stop_m, 3.5) &&
            !result.applied_stop && result.safe,
        "신호 미수신 첫 입력은 히스테리시스 동안 원래 경로를 유지합니다");
  input.now_s = 0.1;
  result = core.apply(path, input);
  check(!result.applied_stop, "정확히 히스테리시스 경계에서는 정지하지 않습니다");
  input.now_s = 0.11;
  result = core.apply(path, input);
  check(result.applied_stop && !result.safe && !result.activated &&
            result.trajectory.size() == 12 &&
            near(result.trajectory[6].x, 5.5) &&
            result.trajectory[5].longitudinal_velocity_mps == 5.0F &&
            result.trajectory[6].longitudinal_velocity_mps == 0.0F &&
            result.trajectory.back().longitudinal_velocity_mps == 0.0F,
        "신호 미수신 안전 정지에서는 정지점을 넣고 뒤쪽 속도를 0으로 만듭니다");
  input.now_s = 0.12;
  input.signal_stamp_s = 0.12;
  input.signal_requires_stop = false;
  result = core.apply(path, input);
  check(!result.applied_stop && result.safe && result.activated,
        "진행 신호를 받으면 정지 상태를 해제합니다");
  input.now_s = 1.13;
  input.signal_stamp_s.reset();
  input.signal_requires_stop.reset();
  result = core.apply(path, input);
  check(!result.applied_stop, "신호가 시간 초과되어도 첫 판정은 히스테리시스를 지킵니다");
  input.now_s = 1.24;
  result = core.apply(path, input);
  check(result.applied_stop, "시간이 지난 신호는 정지 신호로 처리합니다");

  planning::TrafficLightCore simulation({7.0, -1.0, 7.0, 1.0},
      [] { planning::TrafficLightParameters p; p.front_offset_m = 1.0;
           p.simulation = true; return p; }());
  input = {};
  input.now_s = 1.0;
  result = simulation.apply(path, input);
  input.now_s = 2.0;
  result = simulation.apply(path, input);
  check(!result.applied_stop && result.safe,
        "시뮬레이션에서는 신호 미수신만으로 정지하지 않습니다");

  planning::TrafficLightCore fast({7.0, -1.0, 7.0, 1.0}, parameters);
  input = {};
  input.ego_x = 5.0;
  input.ego_velocity_mps = 10.0;
  input.now_s = 0.2;
  fast.apply(path, input);
  input.now_s = 0.4;
  result = fast.apply(path, input);
  check(!result.applied_stop && !result.safe,
        "정지할 거리가 부족하고 황색 시간 안에 도달하면 통과합니다");

  parameters.yellow_period_s = 0.01;
  planning::TrafficLightCore emergency({7.0, -1.0, 7.0, 1.0}, parameters);
  input.ego_x = 0.0;
  input.now_s = 0.0;
  emergency.apply(path, input);
  input.now_s = 0.2;
  result = emergency.apply(path, input);
  check(result.applied_stop,
        "제동 거리가 부족해도 황색 시간 안에 도달하지 못하면 정지합니다");

  input.ego_x = 9.0;
  result = emergency.apply(path, input);
  check(result.state == planning::TrafficLightState::GoOut && !result.applied_stop,
        "정지점을 2m 넘게 지나면 통과 상태로 바뀝니다");
  input.ego_x = 0.0;
  result = emergency.apply(path, input);
  check(result.state == planning::TrafficLightState::Approach && !result.applied_stop,
        "정지선 앞 1m 이상으로 돌아오면 접근 상태로 복귀합니다");

  parameters.enable_rtc = true;
  planning::TrafficLightCore rtc({7.0, -1.0, 7.0, 1.0}, parameters);
  input = {};
  input.ego_velocity_mps = 1.0;
  input.rtc_activated = true;
  rtc.apply(path, input);
  input.now_s = 0.2;
  result = rtc.apply(path, input);
  check(!result.applied_stop && !result.safe && result.activated,
        "RTC 승인 상태에서는 정지 판정이 있어도 경로를 바꾸지 않습니다");

  using planning::LaneTurnDirection;
  using planning::TrafficLampColor;
  using planning::TrafficLampElement;
  using planning::TrafficLampShape;
  const std::vector<TrafficLampElement> red{{TrafficLampColor::Red,
                                              TrafficLampShape::Circle}};
  const std::vector<TrafficLampElement> green{{TrafficLampColor::Green,
                                                TrafficLampShape::Circle}};
  const std::vector<TrafficLampElement> right_arrow{{TrafficLampColor::Green,
                                                      TrafficLampShape::RightArrow}};
  check(planning::trafficSignalRequiresStop(LaneTurnDirection::Straight, red) &&
            !planning::trafficSignalRequiresStop(LaneTurnDirection::Else, green) &&
            !planning::trafficSignalRequiresStop(LaneTurnDirection::Right, right_arrow) &&
            planning::trafficSignalRequiresStop(LaneTurnDirection::Left, right_arrow) &&
            planning::trafficSignalRequiresStop(LaneTurnDirection::Else, right_arrow),
        "원형 녹색과 차로별 화살표 신호를 원본과 같이 판정합니다");

  parameters.enable_rtc = false;
  planning::TrafficLightCore observed({7.0, -1.0, 7.0, 1.0}, parameters);
  planning::TrafficLightSignalObservation observation;
  observation.lane_turn = LaneTurnDirection::Straight;
  observation.elements = red;
  input = {};
  input.ego_velocity_mps = 1.0;
  observed.apply(path, input, observation);
  input.now_s = 0.2;
  result = observed.apply(path, input, observation);
  check(result.applied_stop, "인식한 적색 신호를 코어 정지 판정에 연결합니다");
  observation.stamp_s = 0.21;
  observation.elements = green;
  input.now_s = 0.21;
  result = observed.apply(path, input, observation);
  check(!result.applied_stop && result.safe,
        "인식한 녹색 신호를 코어 진행 판정에 연결합니다");
}
