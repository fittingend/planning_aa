// SPDX-License-Identifier: Apache-2.0
#include "ara/core/initialization.h"
#include "ara/exec/execution_client.h"
#include "ara/log/logger.h"
#include "planning/planning_final_pipeline_core.hpp"
#include "planning/planning_route_coordinator.hpp"

#include <csignal>
#include <cstdlib>

int main() {
  sigset_t shutdown_signals;
  sigemptyset(&shutdown_signals);
  sigaddset(&shutdown_signals, SIGINT);
  sigaddset(&shutdown_signals, SIGTERM);
  if (sigprocmask(SIG_BLOCK, &shutdown_signals, nullptr) != 0) {
    return EXIT_FAILURE;
  }

  if (!ara::core::Initialize().HasValue()) {
    return EXIT_FAILURE;
  }

  auto &log = ara::log::CreateLogger("PLAA", "Planning AA bootstrap");
  // Construct the stateful cores before reporting Running. Transport adapters
  // will supply their inputs and publish their outputs in a later integration.
  planning::PlanningRouteCoordinator routes;
  planning::PlanningFinalPipelineCore final_pipeline;
  final_pipeline.reset();
  (void)routes;

  ara::exec::ExecutionClient execution;
  if (!execution.ReportExecutionState(ara::exec::ExecutionState::kRunning)
           .HasValue()) {
    log.LogError() << "Failed to report Running to Execution Management";
    ara::core::Deinitialize();
    return EXIT_FAILURE;
  }

  log.LogInfo() << "PlanningAA initialized; communication adapters are pending";
  int signal_number = 0;
  const int wait_result = sigwait(&shutdown_signals, &signal_number);
  if (wait_result != 0) {
    log.LogError() << "Failed while waiting for shutdown";
  }
  if (!ara::core::Deinitialize().HasValue()) {
    return EXIT_FAILURE;
  }
  return wait_result == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
