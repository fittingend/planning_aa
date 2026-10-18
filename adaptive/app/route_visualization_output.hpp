// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "planning/trajectory.hpp"

#include <string>

namespace adaptive {

// Sends the full planning path to an RViz-only ROS adapter. The control
// trajectory remains the local, velocity-smoothed trajectory.
class RouteVisualizationOutput {
 public:
  explicit RouteVisualizationOutput(const std::string &target);
  ~RouteVisualizationOutput();
  RouteVisualizationOutput(const RouteVisualizationOutput &) = delete;
  RouteVisualizationOutput &operator=(const RouteVisualizationOutput &) = delete;

  bool send(const planning::Trajectory &path) const;

 private:
  int socket_ = -1;
  unsigned int address_ = 0;
};

}  // namespace adaptive
