// Copyright 2020 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 횡단보도 객체별 양보·통과·무시 상태를 유지합니다.

#pragma once

#include "planning/crosswalk_core.hpp"
#include "planning/traffic_light_signal.hpp"

#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace planning {

struct CrosswalkObjectStateParameters {
  double stopped_speed_mps = 0.25;
  double vehicle_cross_angle_threshold_rad = 0.5;
  bool ignore_new_away_object_at_traffic_light = true;
  std::vector<double> distance_to_crosswalk_m{1.0, 5.0};
  std::vector<double> no_intention_timeout_s{1.0, 0.0};
  CrosswalkPassParameters pass;
};

struct CrosswalkObjectObservation {
  double now_s = 0.0;
  double speed_mps = 0.0;
  double distance_to_crosswalk_m = 0.0;
  bool ego_yielding = false;
  bool away_from_attention_area = false;
  bool crosswalk_has_traffic_light = false;
  bool bicycle_or_motorcycle = false;
  std::optional<double> ego_passage_direction_rad;
  std::optional<double> object_passage_direction_rad;
  std::optional<std::pair<double, double>> collision_times_s;
};

struct CrosswalkPedestrianSignal {
  double stamp_s = 0.0;
  std::vector<TrafficLampElement> elements;
};

bool pedestriansHaveRedSignal(
    const std::vector<CrosswalkPedestrianSignal> &signals,
    double now_s, double timeout_s = 3.0);

class CrosswalkObjectStateTracker {
 public:
  explicit CrosswalkObjectStateTracker(CrosswalkObjectStateParameters parameters = {});

  void beginFrame();
  void beginFrame(
      const std::vector<CrosswalkPedestrianSignal> &signals,
      double now_s, double timeout_s = 3.0);
  CrosswalkCollisionState update(
      const std::string &id, const CrosswalkObjectObservation &observation);
  void endFrame();
  std::optional<CrosswalkCollisionState> state(const std::string &id) const;
  std::size_t size() const noexcept { return objects_.size(); }

 private:
  struct ObjectState {
    CrosswalkCollisionState collision_state = CrosswalkCollisionState::Yield;
    std::optional<double> stopped_since_s;
  };

  CrosswalkObjectStateParameters parameters_;
  std::unordered_map<std::string, ObjectState> objects_;
  std::unordered_set<std::string> seen_;
  bool ignore_crosswalk_ = false;
};

}  // planning 네임스페이스
