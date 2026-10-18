// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "planning/planning_validator_core.hpp"

#include <cstdint>
#include <string>

namespace adaptive {

// Sends AA's own validator results to the ROS diagnostic adapter. A missing
// receiver never changes planning or trajectory publication.
class DiagnosticOutput {
 public:
  explicit DiagnosticOutput(const std::string &target);
  ~DiagnosticOutput();
  DiagnosticOutput(const DiagnosticOutput &) = delete;
  DiagnosticOutput &operator=(const DiagnosticOutput &) = delete;

  bool enabled() const noexcept { return socket_ >= 0; }
  bool send(const planning::ValidationOutcome &outcome, double stamp_s) const;

 private:
  int socket_ = -1;
  std::uint32_t address_ = 0;
};

}  // namespace adaptive
