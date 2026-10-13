# K-City 기본 Planning 출력 관측 (10개 원본 기록)

`0개 기록`은 이 기록들에서 메시지가 없다는 뜻이며 노드 비활성의 증거는 아닙니다. 서비스 호출은 SQLite topic 집계로 확인할 수 없습니다. endpoint 기준은 `decision_planning_contract_matrix.md`입니다.

QoS는 rosbag metadata의 publisher 제공 프로필이다. 표기의 순서는 reliability/durability/depth이며 R=reliable, B=best effort, T=transient local, V=volatile이다.

| 출력 | 메시지가 있는 기록 | 총 메시지 | 기록된 ROS 형식 | 기록 QoS |
| --- | ---: | ---: | --- | --- |
| `mission route` (`/planning/mission_planning/route`) | 10/10 | 10 | `autoware_planning_msgs/msg/LaneletRoute` | `R/T/1` |
| `mission state` (`/planning/mission_planning/state`) | 10/10 | 15 | `tier4_planning_msgs/msg/RouteState` | `R/T/1` |
| `mission route marker` (`/planning/mission_planning/route_marker`) | 10/10 | 10 | `visualization_msgs/msg/MarkerArray` | `R/T/1` |
| `main route` (`/planning/mission_planning/route_selector/main/route`) | 10/10 | 10 | `autoware_planning_msgs/msg/LaneletRoute` | `R/T/1` |
| `main state` (`/planning/mission_planning/route_selector/main/state`) | 10/10 | 15 | `tier4_planning_msgs/msg/RouteState` | `R/T/1` |
| `MRM route` (`/planning/mission_planning/route_selector/mrm/route`) | 0/10 | 0 | `autoware_planning_msgs/msg/LaneletRoute` | `R/T/1` |
| `MRM state` (`/planning/mission_planning/route_selector/mrm/state`) | 10/10 | 10 | `tier4_planning_msgs/msg/RouteState` | `R/T/1` |
| `goal echo` (`/planning/mission_planning/echo_back_goal_pose`) | 10/10 | 10 | `geometry_msgs/msg/PoseStamped` | `R/T/1` |
| `scenario` (`/planning/scenario_planning/scenario`) | 10/10 | 6105 | `tier4_planning_msgs/msg/Scenario` | `R/V/1` |
| `external max velocity` (`/planning/scenario_planning/max_velocity`) | 4/10 | 601 | `tier4_planning_msgs/msg/VelocityLimit` | `R/V/1` |
| `current max velocity` (`/planning/scenario_planning/current_max_velocity`) | 10/10 | 592 | `tier4_planning_msgs/msg/VelocityLimit` | `R/T/1` |
| `behavior path lane ids` (`/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id`) | 10/10 | 6143 | `tier4_planning_msgs/msg/PathWithLaneId` | `R/V/1` |
| `turn indicators` (`/planning/turn_indicators_cmd`) | 10/10 | 6093 | `autoware_vehicle_msgs/msg/TurnIndicatorsCommand` | `R/V/1` |
| `hazard lights` (`/planning/hazard_lights_cmd`) | 10/10 | 6208 | `autoware_vehicle_msgs/msg/HazardLightsCommand` | `R/V/1` |
| `modified goal` (`/planning/scenario_planning/modified_goal`) | 0/10 | 0 | `autoware_planning_msgs/msg/PoseWithUuidStamped` | `R/T/1` |
| `reroute availability` (`/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/output/is_reroute_available`) | 10/10 | 6078 | `tier4_planning_msgs/msg/RerouteAvailability` | `R/V/1` |
| `behavior velocity path` (`/planning/scenario_planning/lane_driving/behavior_planning/path`) | 10/10 | 6144 | `autoware_planning_msgs/msg/Path` | `R/V/1` |
| `infrastructure commands` (`/planning/scenario_planning/status/infrastructure_commands`) | 10/10 | 59347 | `tier4_v2x_msgs/msg/InfrastructureCommandArray` | `R/V/1` |
| `smoothed path` (`/planning/scenario_planning/lane_driving/motion_planning/path_smoother/path`) | 10/10 | 6048 | `autoware_planning_msgs/msg/Path` | `R/V/1` |
| `elastic band trajectory` (`/planning/scenario_planning/lane_driving/motion_planning/elastic_band_smoother/output/traj`) | 10/10 | 6045 | `autoware_planning_msgs/msg/Trajectory` | `R/V/1` |
| `optimized trajectory` (`/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory`) | 10/10 | 6101 | `autoware_planning_msgs/msg/Trajectory` | `R/V/1` |
| `motion velocity trajectory` (`/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory`) | 10/10 | 6136 | `autoware_planning_msgs/msg/Trajectory` | `R/V/1` |
| `motion velocity metrics` (`/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/metrics`) | 0/10 | 0 | `없음` | `없음` |
| `lane driving trajectory` (`/planning/scenario_planning/lane_driving/trajectory`) | 10/10 | 6095 | `autoware_planning_msgs/msg/Trajectory` | `R/V/1` |
| `parking occupancy` (`/planning/scenario_planning/parking/costmap_generator/occupancy_grid`) | 0/10 | 0 | `nav_msgs/msg/OccupancyGrid` | `R/V/1` |
| `parking grid map` (`/planning/scenario_planning/parking/costmap_generator/grid_map`) | 0/10 | 0 | `grid_map_msgs/msg/GridMap` | `R/V/1` |
| `parking trajectory` (`/planning/scenario_planning/parking/trajectory`) | 0/10 | 0 | `autoware_planning_msgs/msg/Trajectory` | `R/T/1` |
| `parking complete` (`/planning/scenario_planning/parking/is_completed`) | 10/10 | 6126 | `std_msgs/msg/Bool` | `R/T/1` |
| `selected trajectory` (`/planning/scenario_planning/scenario_selector/trajectory`) | 10/10 | 6170 | `autoware_planning_msgs/msg/Trajectory` | `R/V/1` |
| `velocity smoother trajectory` (`/planning/scenario_planning/velocity_smoother/trajectory`) | 10/10 | 6092 | `autoware_planning_msgs/msg/Trajectory` | `R/V/1` |
| `distance to stopline` (`/planning/scenario_planning/velocity_smoother/distance_to_stopline`) | 10/10 | 6026 | `tier4_debug_msgs/msg/Float32Stamped` | `R/V/1` |
| `validated trajectory` (`/planning/scenario_planning/trajectory`) | 10/10 | 6062 | `autoware_planning_msgs/msg/Trajectory` | `R/V/1` |
| `validation status` (`/planning/planning_validator/validation_status`) | 10/10 | 6085 | `autoware_planning_validator/msg/PlanningValidatorStatus` | `R/V/1` |
| `evaluator metrics` (`/planning/planning_evaluator/metrics`) | 10/10 | 6140 | `diagnostic_msgs/msg/DiagnosticArray` | `R/V/1` |
| `remaining distance time` (`/planning/mission_remaining_distance_time`) | 10/10 | 6194 | `autoware_internal_msgs/msg/MissionRemainingDistanceTime` | `R/V/10` |
| `stop reasons` (`/planning/scenario_planning/status/stop_reasons`) | 10/10 | 20681 | `tier4_planning_msgs/msg/StopReasonArray` | `R/V/1` |
| `velocity limit candidates` (`/planning/scenario_planning/max_velocity_candidates`) | 8/10 | 600 | `tier4_planning_msgs/msg/VelocityLimit` | `R/T/1` |
| `clear velocity limit` (`/planning/scenario_planning/clear_velocity_limit`) | 8/10 | 17 | `tier4_planning_msgs/msg/VelocityLimitClearCommand` | `R/T/1` |

## 입력 기록

- `hpc/rosbag_blocking/rosbag_20251231_023908_0.db3`
- `hpc/rosbag_cutinout/rosbag_20251231_024435_0.db3`
- `hpc/rosbag_pedestrian2/rosbag_20251231_021250_0.db3`
- `hpc/rosbag_rotary/rosbag_20251231_022924_0.db3`
- `hpc/rosbag_runout/rosbag_20251231_013654_0.db3`
- `hpc/rosbag_scurve/rosbag_20251231_022311_0.db3`
- `hpc/rosbag_stopline2/rosbag_20251231_014354_0.db3`
- `ipc/rosbag_cutinout/rosbag_20251231_133116_0.db3`
- `ipc/rosbag_pedestrian/rosbag_20251231_132433_0.db3`
- `ipc/rosbag_stopline/rosbag_20251231_132036_0.db3`

읽을 수 없어 제외한 파일:

- `ipc/rosbag_cutinout/rosbag_20251231_132840_0.db3`: disk I/O error
