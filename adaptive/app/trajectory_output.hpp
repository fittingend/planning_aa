// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "oss/type/autoware_planning_msgs/msg/impl_type_trajectory.h"
#include "planning/planning_final_pipeline_core.hpp"

#include <optional>
#include <string>

namespace adaptive {

// Convert only a newly validated final result. An invalid frame must never
// resurrect a previous trajectory as a fresh control command.
std::optional<oss::type::autoware_planning_msgs::msg::Trajectory>
makeTrajectoryEvent(const planning::PlanningFinalResult &result,
                    double stamp_s, const std::string &frame_id);

// Scenario selector publishes its selected lane/parking trajectory before smoothing.
std::optional<oss::type::autoware_planning_msgs::msg::Trajectory>
makeSelectedTrajectoryEvent(const planning::PlanningFinalResult &result,
                            double stamp_s, const std::string &frame_id);

}  // namespace adaptive
