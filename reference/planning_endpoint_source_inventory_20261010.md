# 기준 Planning 정적 출력·서비스 생성 코드 목록

근거: `source-api-evidence.json`. 기본 Planning 계산 노드 17개, publisher 생성 코드 99개, service 생성 코드 13개. 테스트·별도 보조 노드·선택되지 않은 알고리즘은 제외했다. 동적 이름·플러그인/RTC·실행 graph·최종 remap은 별도 검증 대상이다.

| 노드 | API | 코드의 topic/service 식 | 근거 파일:줄 |
| --- | --- | --- | --- |
| behavior_path_planner | create_publisher | `~/output/path` | `behavior_path_planner_node.cpp:51` |
| behavior_path_planner | create_publisher | `~/output/turn_indicators_cmd` | `behavior_path_planner_node.cpp:53` |
| behavior_path_planner | create_publisher | `~/output/hazard_lights_cmd` | `behavior_path_planner_node.cpp:54` |
| behavior_path_planner | create_publisher | `~/output/modified_goal` | `behavior_path_planner_node.cpp:57` |
| behavior_path_planner | create_publisher | `~/output/is_reroute_available` | `behavior_path_planner_node.cpp:59` |
| behavior_path_planner | create_publisher | `~/debug/avoidance_debug_message_array` | `behavior_path_planner_node.cpp:62` |
| behavior_path_planner | create_publisher | `~/debug/turn_signal_info` | `behavior_path_planner_node.cpp:67` |
| behavior_path_planner | create_publisher | `~/debug/bound` | `behavior_path_planner_node.cpp:69` |
| behavior_path_planner | create_publisher | `동적 이름: create_publisher<Path>(path_candidate_name_space + manager->name(), 1));` | `behavior_path_planner_node.cpp:103` |
| behavior_path_planner | create_publisher | `동적 이름: create_publisher<Path>(path_reference_name_space + manager->name(), 1));` | `behavior_path_planner_node.cpp:105` |
| behavior_velocity_planner | create_service | `~/service/load_plugin` | `node.cpp:73` |
| behavior_velocity_planner | create_service | `~/service/unload_plugin` | `node.cpp:75` |
| behavior_velocity_planner | create_publisher | `~/output/path` | `node.cpp:83` |
| behavior_velocity_planner | create_publisher | `~/debug/path` | `node.cpp:84` |
| costmap_generator | create_publisher | `~/output/grid_map` | `costmap_generator.cpp:172` |
| costmap_generator | create_publisher | `~/output/occupancy_grid` | `costmap_generator.cpp:174` |
| costmap_generator | create_publisher | `processing_time` | `costmap_generator.cpp:176` |
| costmap_generator | create_publisher | `~/debug/processing_time_ms` | `costmap_generator.cpp:179` |
| elastic_band_smoother | create_publisher | `~/debug/eb_traj` | `elastic_band.cpp:178` |
| elastic_band_smoother | create_publisher | `~/debug/eb_fixed_traj` | `elastic_band.cpp:179` |
| elastic_band_smoother | create_publisher | `~/output/traj` | `elastic_band_smoother.cpp:79` |
| elastic_band_smoother | create_publisher | `~/output/path` | `elastic_band_smoother.cpp:80` |
| elastic_band_smoother | create_publisher | `~/debug/extended_traj` | `elastic_band_smoother.cpp:87` |
| elastic_band_smoother | create_publisher | `~/debug/calculation_time` | `elastic_band_smoother.cpp:88` |
| elastic_band_smoother | create_publisher | `~/debug/processing_time_ms` | `elastic_band_smoother.cpp:90` |
| external_velocity_limit_selector | create_publisher | `output/external_velocity_limit` | `external_velocity_limit_selector_node.cpp:136` |
| external_velocity_limit_selector | create_publisher | `output/debug` | `external_velocity_limit_selector_node.cpp:138` |
| freespace_planner | create_publisher | `~/output/trajectory` | `freespace_planner_node.cpp:94` |
| freespace_planner | create_publisher | `~/debug/pose_array` | `freespace_planner_node.cpp:95` |
| freespace_planner | create_publisher | `~/debug/partial_pose_array` | `freespace_planner_node.cpp:96` |
| freespace_planner | create_publisher | `is_completed` | `freespace_planner_node.cpp:97` |
| freespace_planner | create_publisher | `~/debug/processing_time_ms` | `freespace_planner_node.cpp:98` |
| goal_pose_visualizer | create_publisher | `output/goal_pose` | `goal_pose_visualizer.cpp:25` |
| mission_planner | create_publisher | `~/debug/goal_footprint` | `default_planner.cpp:54` |
| mission_planner | create_publisher | `~/debug/route_marker` | `mission_planner.cpp:68` |
| mission_planner | create_service | `~/clear_route` | `mission_planner.cpp:73` |
| mission_planner | create_service | `~/set_lanelet_route` | `mission_planner.cpp:75` |
| mission_planner | create_service | `~/set_waypoint_route` | `mission_planner.cpp:78` |
| mission_planner | create_publisher | `~/route` | `mission_planner.cpp:81` |
| mission_planner | create_publisher | `~/state` | `mission_planner.cpp:82` |
| mission_planner | create_publisher | `~/debug/processing_time_ms` | `mission_planner.cpp:91` |
| motion_velocity_planner | create_service | `~/service/load_plugin` | `node.cpp:76` |
| motion_velocity_planner | create_service | `~/service/unload_plugin` | `node.cpp:78` |
| motion_velocity_planner | create_publisher | `~/output/trajectory` | `node.cpp:84` |
| motion_velocity_planner | create_publisher | `~/debug/processing_time_ms` | `node.cpp:86` |
| motion_velocity_planner | create_publisher | `~/debug/markers` | `node.cpp:89` |
| motion_velocity_planner | create_publisher | `~/metrics` | `node.cpp:90` |
| obstacle_cruise_planner | create_publisher | `~/output/stop_speed_exceeded` | `planner_interface.hpp:62` |
| obstacle_cruise_planner | create_publisher | `~/metrics` | `planner_interface.hpp:63` |
| obstacle_cruise_planner | create_publisher | `~/output/trajectory` | `node.cpp:510` |
| obstacle_cruise_planner | create_publisher | `~/output/velocity_limit` | `node.cpp:512` |
| obstacle_cruise_planner | create_publisher | `~/output/clear_velocity_limit` | `node.cpp:513` |
| obstacle_cruise_planner | create_publisher | `~/debug/processing_time_ms` | `node.cpp:517` |
| obstacle_cruise_planner | create_publisher | `~/virtual_wall/cruise` | `node.cpp:518` |
| obstacle_cruise_planner | create_publisher | `~/virtual_wall/stop` | `node.cpp:519` |
| obstacle_cruise_planner | create_publisher | `~/virtual_wall/slow_down` | `node.cpp:520` |
| obstacle_cruise_planner | create_publisher | `~/debug/marker` | `node.cpp:521` |
| obstacle_cruise_planner | create_publisher | `~/debug/stop_planning_info` | `node.cpp:523` |
| obstacle_cruise_planner | create_publisher | `~/debug/cruise_planning_info` | `node.cpp:525` |
| obstacle_cruise_planner | create_publisher | `~/debug/slow_down_planning_info` | `node.cpp:527` |
| path_optimizer | create_publisher | `~/debug/mpt_fixed_traj` | `mpt_optimizer.cpp:424` |
| path_optimizer | create_publisher | `~/debug/mpt_ref_traj` | `mpt_optimizer.cpp:425` |
| path_optimizer | create_publisher | `~/debug/mpt_traj` | `mpt_optimizer.cpp:426` |
| path_optimizer | create_publisher | `~/output/path` | `node.cpp:96` |
| path_optimizer | create_publisher | `~/virtual_wall` | `node.cpp:97` |
| path_optimizer | create_publisher | `~/debug/extended_traj` | `node.cpp:104` |
| path_optimizer | create_publisher | `~/debug/marker` | `node.cpp:105` |
| path_optimizer | create_publisher | `~/debug/calculation_time` | `node.cpp:106` |
| path_optimizer | create_publisher | `~/debug/processing_time_ms` | `node.cpp:108` |
| path_optimizer | create_publisher | `~/debug/processing_time_detail_ms` | `node.cpp:110` |
| planning_evaluator | create_publisher | `~/debug/processing_time_ms` | `planning_evaluator_node.cpp:62` |
| planning_evaluator | create_publisher | `~/metrics` | `planning_evaluator_node.cpp:66` |
| planning_validator | create_publisher | `~/debug/marker` | `debug_marker.cpp:30` |
| planning_validator | create_publisher | `~/virtual_wall` | `debug_marker.cpp:33` |
| planning_validator | create_publisher | `~/output/trajectory` | `planning_validator.cpp:38` |
| planning_validator | create_publisher | `~/output/validation_status` | `planning_validator.cpp:39` |
| planning_validator | create_publisher | `~/output/markers` | `planning_validator.cpp:40` |
| planning_validator | create_publisher | `~/debug/processing_time_ms` | `planning_validator.cpp:41` |
| remaining_distance_time_calculator | create_publisher | `~/output/mission_remaining_distance_time` | `remaining_distance_time_calculator_node.cpp:64` |
| route_selector | create_service | `~/main/clear_route` | `route_selector.cpp:97` |
| route_selector | create_service | `~/main/set_waypoint_route` | `route_selector.cpp:100` |
| route_selector | create_service | `~/main/set_lanelet_route` | `route_selector.cpp:103` |
| route_selector | create_publisher | `~/main/state` | `route_selector.cpp:106` |
| route_selector | create_publisher | `~/main/route` | `route_selector.cpp:107` |
| route_selector | create_service | `~/mrm/clear_route` | `route_selector.cpp:110` |
| route_selector | create_service | `~/mrm/set_waypoint_route` | `route_selector.cpp:112` |
| route_selector | create_service | `~/mrm/set_lanelet_route` | `route_selector.cpp:115` |
| route_selector | create_publisher | `~/mrm/state` | `route_selector.cpp:118` |
| route_selector | create_publisher | `~/mrm/route` | `route_selector.cpp:119` |
| route_selector | create_publisher | `~/debug/processing_time_ms` | `route_selector.cpp:141` |
| scenario_selector | create_publisher | `output/scenario` | `node.cpp:482` |
| scenario_selector | create_publisher | `output/trajectory` | `node.cpp:483` |
| scenario_selector | create_publisher | `~/debug/processing_time_ms` | `node.cpp:493` |
| velocity_smoother | create_publisher | `~/debug/processing_time_detail_ms` | `node.cpp:53` |
| velocity_smoother | create_publisher | `~/output/trajectory` | `node.cpp:62` |
| velocity_smoother | create_publisher | `~/virtual_wall` | `node.cpp:63` |
| velocity_smoother | create_publisher | `~/output/current_velocity_limit_mps` | `node.cpp:64` |
| velocity_smoother | create_publisher | `~/distance_to_stopline` | `node.cpp:66` |
| velocity_smoother | create_publisher | `~/closest_velocity` | `node.cpp:76` |
| velocity_smoother | create_publisher | `~/closest_acceleration` | `node.cpp:77` |
| velocity_smoother | create_publisher | `~/closest_jerk` | `node.cpp:78` |
| velocity_smoother | create_publisher | `~/closest_max_velocity` | `node.cpp:79` |
| velocity_smoother | create_publisher | `~/debug/processing_time_ms` | `node.cpp:80` |
| velocity_smoother | create_publisher | `~/debug/trajectory_raw` | `node.cpp:81` |
| velocity_smoother | create_publisher | `~/debug/trajectory_external_velocity_limited` | `node.cpp:83` |
| velocity_smoother | create_publisher | `~/debug/trajectory_lateral_acc_filtered` | `node.cpp:85` |
| velocity_smoother | create_publisher | `~/debug/trajectory_steering_rate_limited` | `node.cpp:87` |
| velocity_smoother | create_publisher | `~/debug/trajectory_time_resampled` | `node.cpp:88` |
| velocity_smoother | create_publisher | `~/debug/forward_filtered_trajectory` | `node.cpp:114` |
| velocity_smoother | create_publisher | `~/debug/backward_filtered_trajectory` | `node.cpp:116` |
| velocity_smoother | create_publisher | `~/debug/merged_filtered_trajectory` | `node.cpp:118` |
| velocity_smoother | create_publisher | `~/closest_merged_velocity` | `node.cpp:120` |

## 기준 launch에서 선택하지 않은 알고리즘

`obstacle_cruise_planner`는 `pid_base`를 사용한다. 아래 `optimization_base` 내부 생성 위치는 활성 출력 개수에 포함하지 않았다.

| 노드 | API | 코드의 topic/service 식 | 근거 파일:줄 |
| --- | --- | --- | --- |
| obstacle_cruise_planner | create_subscription | `/planning/scenario_planning/trajectory` | `optimization_based_planner.cpp:51` |
| obstacle_cruise_planner | create_publisher | `~/optimized_sv_trajectory` | `optimization_based_planner.cpp:101` |
| obstacle_cruise_planner | create_publisher | `~/optimized_st_graph` | `optimization_based_planner.cpp:102` |
| obstacle_cruise_planner | create_publisher | `~/boundary` | `optimization_based_planner.cpp:103` |
| obstacle_cruise_planner | create_publisher | `~/debug/wall_marker` | `optimization_based_planner.cpp:104` |
