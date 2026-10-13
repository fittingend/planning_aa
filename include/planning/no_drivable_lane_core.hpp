// 주행 금지 차로 진입 전 정지와 모듈 상태를 관리합니다.

#pragma once

#include "planning/speed_bump_core.hpp"

namespace planning {

enum class NoDrivableLaneState { Init, Approaching, Inside, Stopped };

struct NoDrivableLaneResult {
  Trajectory trajectory;
  bool stopped = false;
  NoDrivableLaneState state = NoDrivableLaneState::Init;
};

class NoDrivableLaneCore {
 public:
  explicit NoDrivableLaneCore(std::vector<SpeedBumpPoint> polygon);

  NoDrivableLaneResult apply(
      const Trajectory &path, double ego_x, double ego_y,
      bool vehicle_stopped);
  NoDrivableLaneState state() const noexcept { return state_; }

 private:
  std::vector<SpeedBumpPoint> polygon_;
  NoDrivableLaneState state_ = NoDrivableLaneState::Init;
};

}  // planning 네임스페이스
