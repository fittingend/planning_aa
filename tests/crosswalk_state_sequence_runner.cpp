#include "planning/crosswalk_object_state.hpp"

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

int main() {
  std::size_t count = 0;
  std::cin >> count;
  std::string line;
  std::getline(std::cin, line);
  planning::CrosswalkObjectStateTracker tracker;
  for (std::size_t i = 0; i < count; ++i) {
    std::string id;
    double now, speed_x, speed_y, ttc, ttv;
    if (!std::getline(std::cin, line)) return 2;
    std::istringstream fields(line);
    fields >> id >> now >> speed_x >> speed_y >> ttc >> ttv;
    if (!fields) return 2;
    int ego_yielding = 0;
    int away_from_attention_area = 0;
    double distance_to_crosswalk_m = 2.0;
    if (fields >> ego_yielding >> away_from_attention_area >>
        distance_to_crosswalk_m) {
      // 선택 입력은 정지 의도 판정이 필요한 연속 재생에서 사용합니다.
    }
    planning::CrosswalkObjectObservation observation;
    observation.now_s = now;
    observation.speed_mps = std::hypot(speed_x, speed_y);
    observation.distance_to_crosswalk_m = distance_to_crosswalk_m;
    observation.ego_yielding = ego_yielding != 0;
    observation.away_from_attention_area = away_from_attention_area != 0;
    observation.collision_times_s = std::make_pair(ttc, ttv);
    tracker.beginFrame();
    const auto state = tracker.update(id, observation);
    tracker.endFrame();
    std::cout << static_cast<int>(state) << '\n';
  }
}
