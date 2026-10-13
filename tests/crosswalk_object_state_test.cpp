#include "planning/crosswalk_object_state.hpp"

#include <cstdlib>
#include <iostream>

namespace {
void check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}
}

int main() {
  using planning::CrosswalkCollisionState;
  using planning::CrosswalkObjectObservation;
  using planning::CrosswalkObjectStateTracker;

  CrosswalkObjectStateTracker tracker;
  CrosswalkObjectObservation observed;
  observed.now_s = 0.0;
  observed.speed_mps = 0.0;
  observed.distance_to_crosswalk_m = 2.0;
  observed.ego_yielding = true;
  observed.away_from_attention_area = true;
  observed.collision_times_s = std::make_pair(5.0, 1.0);
  tracker.beginFrame();
  check(tracker.update("pedestrian", observed) == CrosswalkCollisionState::Yield,
        "정지 직후 객체에는 양보합니다");
  tracker.endFrame();
  observed.now_s = 0.5;
  tracker.beginFrame();
  check(tracker.update("pedestrian", observed) == CrosswalkCollisionState::Yield,
        "의도 판정 시간이 지나기 전에는 양보합니다");
  tracker.endFrame();
  observed.now_s = 0.8;
  tracker.beginFrame();
  check(tracker.update("pedestrian", observed) == CrosswalkCollisionState::Ignore,
        "정지 객체가 주의 영역 밖에서 기다리면 무시합니다");
  tracker.endFrame();
  observed.now_s = 1.0;
  observed.speed_mps = 1.0;
  tracker.beginFrame();
  check(tracker.update("pedestrian", observed) == CrosswalkCollisionState::Yield,
        "객체가 다시 움직이면 충돌 상태를 재평가합니다");
  tracker.endFrame();

  CrosswalkObjectObservation vehicle;
  vehicle.now_s = 1.0;
  vehicle.speed_mps = 0.0;
  vehicle.crosswalk_has_traffic_light = true;
  vehicle.away_from_attention_area = true;
  vehicle.collision_times_s = std::make_pair(2.0, 2.0);
  tracker.beginFrame();
  check(tracker.update("new", vehicle) == CrosswalkCollisionState::Ignore,
        "신호등이 있는 횡단보도의 새 외부 객체를 무시합니다");
  vehicle.speed_mps = 2.0;
  check(tracker.update("new", vehicle) == CrosswalkCollisionState::Yield,
        "이동 객체의 충돌 정보가 있으면 다시 양보를 판정합니다");
  vehicle.crosswalk_has_traffic_light = false;
  vehicle.away_from_attention_area = false;
  vehicle.bicycle_or_motorcycle = true;
  vehicle.ego_passage_direction_rad = 0.0;
  vehicle.object_passage_direction_rad = 0.1;
  check(tracker.update("bicycle", vehicle) == CrosswalkCollisionState::Ignore,
        "차로와 나란히 지나는 자전거를 무시합니다");
  tracker.endFrame();
  check(!tracker.state("pedestrian") && tracker.size() == 2,
        "이번 프레임에 보이지 않은 객체 상태를 삭제합니다");

  const std::vector<planning::CrosswalkPedestrianSignal> signals{
      {2.0, {{planning::TrafficLampColor::Red,
              planning::TrafficLampShape::Circle}}}};
  check(planning::pedestriansHaveRedSignal(signals, 4.0),
        "최신 빨간 원형 신호에서는 횡단보도 객체 판단을 건너뜁니다");
  check(!planning::pedestriansHaveRedSignal(signals, 5.1),
        "시간 초과 신호는 사용하지 않습니다");
  check(!planning::pedestriansHaveRedSignal(
            {{2.0, {{planning::TrafficLampColor::Red,
                     planning::TrafficLampShape::LeftArrow}}}}, 4.0),
        "빨간 화살표만으로 보행자 적색을 판정하지 않습니다");

  observed.now_s = 4.0;
  tracker.beginFrame(signals, 4.0);
  check(tracker.update("new", observed) == CrosswalkCollisionState::Ignore,
        "보행자 적색 신호에서는 충돌 객체를 무시합니다");
  tracker.endFrame();
  check(!tracker.state("new"),
        "보행자 적색 신호에서 건너뛴 객체 상태를 삭제합니다");
  tracker.beginFrame();
  check(tracker.update("new", observed) == CrosswalkCollisionState::Yield,
        "적색 신호가 사라지면 객체 충돌을 다시 판단합니다");
  tracker.endFrame();
}
