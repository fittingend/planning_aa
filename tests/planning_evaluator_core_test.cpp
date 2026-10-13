// SPDX-License-Identifier: Apache-2.0

#include "planning/planning_evaluator_core.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void check(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}

planning::TrajectoryPoint point(double x, double y, float velocity) {
  planning::TrajectoryPoint result{};
  result.x = x;
  result.y = y;
  result.orientation_w = 1.0;
  result.longitudinal_velocity_mps = velocity;
  return result;
}
}  // namespace

int main() {
  using namespace planning;
  try {
    PlanningEvaluatorCore current_image_case;
    PlanningEvaluatorInput synthetic;
    synthetic.ego = point(0.0, 0.0, 0.0f);
    synthetic.trajectory = Trajectory{point(0.0, 0.0, 1.0f),
                                      point(1.0, 0.0, 1.0f),
                                      point(2.0, 0.0, 0.0f)};
    const auto synthetic_result = current_image_case.tick(synthetic);
    check(synthetic_result.metrics.at("length").mean == 2.0 &&
              synthetic_result.metrics.at("duration").mean == 2.0 &&
              synthetic_result.metrics.at("point_interval").mean == 1.0 &&
              std::abs(synthetic_result.metrics.at("velocity").mean -
                       2.0 / 3.0) < 1e-12 &&
              synthetic_result.metrics.at("obstacle_distance").mean ==
                  std::numeric_limits<double>::max(),
          "current-image three-point synthetic metrics must match");
    check(synthetic_result.metrics.size() == 9,
          "first current-image metric array must have 27 scalar entries");
    const auto synthetic_second = current_image_case.tick(synthetic);
    check(synthetic_second.metrics.size() == 13 &&
              synthetic_second.metrics.at(
                  "lateral_trajectory_displacement_lookahead").mean == 0.0 &&
              synthetic_second.metrics.at("stability_frechet").mean == 0.0,
          "second current-image metric array must include 39 scalar entries");
    synthetic.trajectory = Trajectory{point(0.0, 1.0, 1.0f),
                                      point(1.0, 1.0, 1.0f),
                                      point(2.0, 1.0, 0.0f)};
    const auto shifted_synthetic = current_image_case.tick(synthetic);
    for (const auto *name : {
             "lateral_trajectory_displacement_local",
             "lateral_trajectory_displacement_lookahead",
             "stability", "stability_frechet"}) {
      const auto &stat = shifted_synthetic.metrics.at(name);
      check(stat.minimum == 1.0 && stat.maximum == 1.0 &&
                stat.mean == 1.0,
            "stationary one-meter path shift must match current-image metric");
    }

    PlanningEvaluatorCore kinematic_image_case;
    PlanningEvaluatorInput moving_synthetic;
    moving_synthetic.ego = point(0.0, 0.0, 2.0f);
    moving_synthetic.ego_speed_mps = 2.0;
    moving_synthetic.ego_acceleration_mps2 = 0.5;
    moving_synthetic.acceleration_stamp_s = 1.0;
    moving_synthetic.trajectory = Trajectory{
        point(0.0, 0.0, 1.0f), point(1.0, 0.0, 1.0f),
        point(2.0, 0.0, 0.0f)};
    const auto moving_first = kinematic_image_case.tick(moving_synthetic);
    check(moving_first.metrics.size() * 3 +
              moving_first.scalar_metrics.size() == 30 &&
              moving_first.scalar_metrics.at("kinematic_state/vel") == 2.0 &&
              moving_first.scalar_metrics.at("kinematic_state/acc") == 0.5 &&
              moving_first.scalar_metrics.at("kinematic_state/jerk") == 0.0,
          "current-image moving synthetic first array must have 30 entries");
    moving_synthetic.acceleration_stamp_s = 1.1;
    const auto moving_second = kinematic_image_case.tick(moving_synthetic);
    check(moving_second.metrics.size() * 3 +
              moving_second.scalar_metrics.size() == 39 &&
              moving_second.metrics.count(
                  "lateral_trajectory_displacement_lookahead") == 0,
          "short path must omit the moving lookahead displacement metric");

    PlanningEvaluatorCore modified_goal_image_case;
    PlanningEvaluatorInput goal_synthetic;
    goal_synthetic.ego = point(0.0, 0.0, 0.0f);
    goal_synthetic.trajectory = Trajectory{
        point(0.0, 0.0, 1.0f), point(1.0, 0.0, 1.0f),
        point(2.0, 0.0, 0.0f)};
    goal_synthetic.modified_goal = BehaviorGoalPose{3.0, 4.0, 0.0};
    const auto goal_first = modified_goal_image_case.tick(goal_synthetic);
    check(goal_first.metrics.size() * 3 == 36 &&
              goal_first.metrics.at(
                  "modified_goal_longitudinal_deviation").mean == 3.0 &&
              goal_first.metrics.at(
                  "modified_goal_lateral_deviation").mean == 4.0 &&
              goal_first.metrics.at(
                  "modified_goal_yaw_deviation").mean == 0.0,
          "current-image modified goal first array must have 36 entries");
    check(modified_goal_image_case.tick(goal_synthetic).metrics.size() * 3 == 48,
          "current-image modified goal second array must have 48 entries");

    PlanningEvaluatorCore obstacle_image_case;
    PlanningEvaluatorInput obstacle_synthetic;
    obstacle_synthetic.ego = point(0.0, 0.0, 0.0f);
    obstacle_synthetic.trajectory = Trajectory{
        point(0.0, 0.0, 1.0f), point(1.0, 0.0, 1.0f),
        point(2.0, 0.0, 0.0f)};
    obstacle_synthetic.object_centers =
        std::vector<std::pair<double, double>>{{1.1, 0.0}};
    const auto obstacle_first = obstacle_image_case.tick(obstacle_synthetic);
    check(obstacle_first.metrics.size() * 3 == 30 &&
              std::abs(obstacle_first.metrics.at(
                  "obstacle_distance").mean - 0.7) < 1e-12 &&
              obstacle_first.metrics.at("obstacle_ttc").mean == 1.0,
          "current-image single obstacle first array must have 30 entries");
    check(obstacle_image_case.tick(obstacle_synthetic).metrics.size() * 3 == 42,
          "current-image single obstacle second array must have 42 entries");

    PlanningEvaluatorCore evaluator;
    PlanningEvaluatorInput input;
    input.ego = point(0.0, 0.0, 2.0f);
    input.ego_speed_mps = 2.0;
    input.ego_acceleration_mps2 = 0.5;
    input.acceleration_stamp_s = 1.0;
    input.trajectory = Trajectory{point(0.0, 0.0, 2.0f),
                                  point(2.0, 0.0, 2.0f),
                                  point(4.0, 0.0, 2.0f)};
    input.reference_trajectory = *input.trajectory;
    input.object_centers = std::vector<std::pair<double, double>>{{4.0, 0.0}};
    input.ego_lane_id = 42;
    input.ego_lane_arc_m = 10.0;
    input.ego_lane_lateral_m = 0.1;
    auto first = evaluator.tick(input);
    check(first.metrics.at("length").mean == 4.0, "trajectory length");
    check(first.metrics.at("duration").mean == 2.0, "trajectory duration");
    check(first.metrics.at("point_interval").count == 2, "point interval count");
    check(first.metrics.at("obstacle_ttc").mean == 2.0, "obstacle TTC");
    check(first.metrics.at("lateral_deviation").maximum == 0.0,
          "reference deviation");
    check(first.scalar_metrics.at("ego_lane_info/lane_id") == 42.0,
          "lane context");
    check(first.metrics.count("stability_frechet") == 0,
          "stability needs previous trajectory");

    input.trajectory = Trajectory{point(0.0, 1.0, 2.0f),
                                  point(2.0, 1.0, 2.0f),
                                  point(4.0, 1.0, 2.0f)};
    input.ego_acceleration_mps2 = 1.0;
    input.acceleration_stamp_s = 2.0;
    auto second = evaluator.tick(input);
    check(std::abs(second.metrics.at("stability_frechet").mean - 1.0) < 1e-9,
          "previous trajectory comparison");
    check(std::abs(second.scalar_metrics.at("kinematic_state/jerk") - 0.5) < 1e-9,
          "acceleration callback state");
    evaluator.reset();
    check(evaluator.tick(input).metrics.count("stability_frechet") == 0,
          "reset previous trajectory");

    evaluator.reset();
    input.trajectory = Trajectory{point(0.0, 0.0, 2.0f),
                                  point(10.0, 0.0, 2.0f),
                                  point(20.0, 0.0, 2.0f)};
    evaluator.tick(input);
    input.trajectory = Trajectory{point(0.0, 1.0, 2.0f),
                                  point(10.0, 1.0, 2.0f),
                                  point(20.0, 1.0, 2.0f)};
    const auto lookahead = evaluator.tick(input);
    check(lookahead.metrics.count(
              "lateral_trajectory_displacement_lookahead") == 1 &&
              std::abs(lookahead.metrics.at(
                  "lateral_trajectory_displacement_lookahead").mean -
                       1.0) < 1e-9,
          "long trajectory should produce sampled lookahead displacement");
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
