#include "planning/behavior_velocity_perception_bridge.hpp"

#include <cstdlib>

int main() {
  planning::BehaviorVelocityMapInput input;
  planning::PlanningPerceivedObject car;
  car.id = "차량";
  car.classification = 1;
  car.x = 5.0;
  car.length_m = 4.0;
  car.width_m = 2.0;
  car.velocity_x_mps = 1.0;
  car.predicted_paths = {{{{6.0, 0.0, 0.0}}, 1.0, 0.9}};
  planning::PlanningPerceivedObject pedestrian;
  pedestrian.id = "보행자";
  pedestrian.classification = 7;
  pedestrian.x = 6.0;
  pedestrian.length_m = 0.5;
  pedestrian.width_m = 0.5;
  planning::applyBehaviorVelocityPerceptionBatch(input, {car, pedestrian});
  if (input.crosswalk_objects.size() != 2 ||
      input.crosswalk_stuck_vehicles.size() != 1 ||
      input.intersection_stuck_vehicles.size() != 1 ||
      input.intersection_predicted_vehicles.size() != 1 ||
      input.no_stopping_vehicles.size() != 1 ||
      input.occlusion_spot_vehicles.size() != 1 ||
      input.dynamic_stop_objects.size() != 2 ||
      input.dynamic_stop_objects.front().type !=
          planning::DynamicObjectType::Car ||
      input.blind_spot_map_objects.size() != 2 ||
      input.run_out_objects.size() != 2 ||
      input.crosswalk_objects.front().predicted.predicted_paths.size() != 1)
    return EXIT_FAILURE;
  planning::applyBehaviorVelocityPerceptionBatch(input, {});
  return input.crosswalk_objects.empty() && input.run_out_objects.empty() &&
                 input.occlusion_spot_vehicles.empty() &&
                 input.dynamic_stop_objects.empty() &&
                 input.intersection_predicted_vehicles.empty()
      ? EXIT_SUCCESS : EXIT_FAILURE;
}
