#include "planning/intersection_collision_core.hpp"

#include <cstdlib>
#include <iostream>

namespace {

void check(bool valid, const char *message) {
  if (!valid) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

}  // 익명 네임스페이스

int main() {
  planning::IntersectionCollisionState state;
  check(!state.update(false, 0.0), "초기 안전 상태입니다");
  check(state.update(true, 0.1), "충돌이 나타나면 즉시 정지합니다");
  check(state.update(true, 0.3), "연속 충돌은 정지를 유지합니다");
  check(state.update(false, 0.6), "첫 안전 판단에서 유지 시간을 시작합니다");
  check(state.update(false, 1.1), "0.5초 경계에서는 정지를 유지합니다");
  check(!state.update(false, 1.11), "0.5초를 넘으면 통과합니다");
  check(state.update(true, 1.2), "새 충돌이 나타나면 다시 정지합니다");
  check(state.update(false, 1.3), "안전 판단에서도 일시 정지를 유지합니다");
  check(state.update(true, 1.4), "충돌 재발은 안전 전환 시간을 초기화합니다");
  check(state.update(false, 1.5), "다시 안전 판단을 시작합니다");
  check(state.update(false, 2.0), "새 경계에서 정지를 유지합니다");
  check(!state.update(false, 2.01), "새 경계가 지난 뒤 통과합니다");
}
