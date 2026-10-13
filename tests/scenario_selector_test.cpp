#include "planning/scenario_selector.hpp"

#include <cstdlib>
#include <iostream>

namespace {
void check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

planning::ScenarioTrajectory path(double last_x, double stamp) {
  planning::ScenarioTrajectory result;
  result.stamp_s = stamp;
  result.points = {{0.0, 0.0, 2.0F, 0.0F},
                   {last_x, 0.0, 0.0F, 0.0F}};
  return result;
}
}  // 익명 네임스페이스

int main() {
  using planning::Scenario;
  using planning::ScenarioPosition;
  planning::ScenarioSelectorCore selector;
  const ScenarioPosition lane{true, true, false, true};
  const ScenarioPosition lot{false, false, true, false};
  check(!selector.tick(0.0, lane), "필수 자료가 오기 전에는 발행하지 않습니다");
  selector.onMap(true);
  selector.onRoute("route-a");
  selector.onOperationMode(true, true);
  selector.onOdometry(0.0, 0.0, 0.0, 0.0);
  check(!selector.onLaneTrajectory(path(10.0, 0.0), 0.0),
        "시나리오를 정하기 전에는 차로 궤적을 발행하지 않습니다");
  auto selection = selector.tick(0.0, lane);
  check(selection && selection->current == Scenario::LaneDriving &&
            !selection->parking_active && !selection->changed,
        "차로와 목적지가 차로 안에 있으면 차로 주행을 고릅니다");
  check(selector.onLaneTrajectory(path(10.0, 0.0), 0.9).has_value(),
        "활성 차로 궤적의 지연이 허용 범위이면 발행합니다");
  check(!selector.onLaneTrajectory(path(10.0, 0.0), 1.1),
        "오래된 차로 궤적은 발행하지 않습니다");
  check(!selector.onParkingTrajectory(path(10.0, 1.1), 1.1),
        "비활성 주차 궤적은 발행하지 않습니다");

  selector.onOdometry(2.0, 10.0, 0.0, 0.0);
  selection = selector.tick(2.0, lot);
  check(selection && selection->current == Scenario::Parking &&
            selection->parking_active && selection->changed,
        "종점에서 정차하고 주차장에 있으면 주차로 전환합니다");
  check(selector.onParkingTrajectory(path(10.0, 2.0), 2.0).has_value(),
        "활성 주차 궤적을 발행합니다");
  selector.onParkingCompleted(true);
  selection = selector.tick(2.1, lane);
  check(selection && selection->current == Scenario::LaneDriving,
        "주차 완료 후 차로에 진입하면 차로 주행으로 돌아갑니다");

  selector.onRoute("route-b");
  check(selector.currentScenario() == Scenario::Empty,
        "다른 경로를 받으면 시나리오를 초기화합니다");
  selector.tick(3.0, lane);
  selector.onLaneTrajectory(path(100.0, 3.0), 3.0);
  selector.onOdometry(3.0, 0.0, 0.0, 0.0);
  selection = selector.tick(3.0, lot);
  check(selection && selection->current == Scenario::LaneDriving,
        "주차 전환의 정차 시간 측정을 시작합니다");
  selection = selector.tick(8.0, lot);
  check(selection && selection->current == Scenario::LaneDriving,
        "정차 시간은 5초를 초과해야 합니다");
  selection = selector.tick(8.1, lot);
  check(selection && selection->current == Scenario::Parking,
        "자율 주행 상태로 5초 넘게 정차하면 주차로 전환합니다");

  planning::ScenarioTrajectory empty_parking;
  empty_parking.stamp_s = 8.1;
  selector.onParkingTrajectory(empty_parking, 8.1);
  ScenarioPosition along_lot{false, false, true, true};
  selection = selector.tick(8.1, along_lot);
  check(selection && selection->current == Scenario::Parking,
        "빈 주차 궤적의 지속 시간을 측정합니다");
  selection = selector.tick(11.1, along_lot);
  check(selection && selection->current == Scenario::Parking,
        "빈 주차 궤적은 3초를 초과해야 합니다");
  selection = selector.tick(11.2, along_lot);
  check(selection && selection->current == Scenario::LaneDriving,
        "빈 주차 궤적과 차로 정렬이 지속되면 차로 주행으로 돌아갑니다");
  selector.onMap(true);
  selector.onRoute("route-b");
  check(selector.currentScenario() == Scenario::Empty,
        "지도를 다시 받은 뒤에는 같은 경로 ID도 초기화합니다");
}
