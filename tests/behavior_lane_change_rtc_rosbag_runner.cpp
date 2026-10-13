// SPDX-License-Identifier: Apache-2.0
// 기록된 RTC 상태 배열에서 승인·강제 명령을 조립합니다.

#include "planning/behavior_lane_change_safety.hpp"

#include <iostream>
#include <vector>

int main() {
  std::size_t frame_count = 0;
  if (!(std::cin >> frame_count)) return 2;
  for (std::size_t frame = 0; frame < frame_count; ++frame) {
    std::size_t count = 0;
    if (!(std::cin >> count)) return 3;
    std::vector<planning::BehaviorLaneChangeRtcChannel> channels;
    for (std::size_t index = 0; index < count; ++index) {
      planning::BehaviorLaneChangeRtcStatus status;
      int state = 0;
      if (!(std::cin >> status.safe >> status.requested >>
            status.command_activated >> status.auto_mode >> state) ||
          state < 0 || state > 4) return 4;
      status.state = static_cast<planning::BehaviorLaneChangeRtcState>(state);
      channels.push_back(
          planning::behaviorLaneChangeRtcChannelFromStatus(status));
    }
    const auto rtc = planning::assembleBehaviorLaneChangeRtcSignals(
        channels, true);
    std::cout << rtc.activated << ' ' << rtc.force_activated << ' '
              << rtc.force_deactivated << '\n';
  }
  return 0;
}
