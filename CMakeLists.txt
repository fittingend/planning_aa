cmake_minimum_required(VERSION 3.16)
project(decision_ap VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED)

add_library(planning_core STATIC
  src/planning_evaluator/planning_evaluator_core.cpp
  src/behavior_path_planner/behavior_dynamic_drivable_area.cpp
  src/costmap_generator/costmap_generator_core.cpp
  src/freespace_planner/freespace_planner_core.cpp
  src/remaining_distance_time_calculator/remaining_distance_time_core.cpp
  src/mission_planner/mission_planner_core.cpp
  src/mission_planner/planning_route_coordinator.cpp
  src/mission_planner/mission_reroute_safety.cpp
  src/mission_planner/route_selector_core.cpp
  src/scenario_selector/scenario_selector.cpp
  src/pipeline/planning_final_pipeline_core.cpp
  src/behavior_velocity_planner/modules/stop_line_core.cpp
  src/behavior_velocity_planner/crosswalk/crosswalk_core.cpp
  src/behavior_velocity_planner/modules/walkway_core.cpp
  src/behavior_velocity_planner/intersection/intersection_stuck_core.cpp
  src/behavior_velocity_planner/intersection/merge_from_private_core.cpp
  src/behavior_velocity_planner/intersection/intersection_collision_core.cpp
  src/behavior_velocity_planner/intersection/intersection_occlusion_core.cpp
  src/behavior_velocity_planner/modules/detection_area_core.cpp
  src/behavior_velocity_planner/modules/speed_bump_core.cpp
  src/behavior_velocity_planner/modules/no_drivable_lane_core.cpp
  src/behavior_velocity_planner/run_out/run_out_state.cpp
  src/behavior_velocity_planner/run_out/run_out_collision.cpp
  src/behavior_velocity_planner/run_out/run_out_obstacle_creator.cpp
  src/behavior_velocity_planner/modules/trajectory_stop.cpp
  src/behavior_velocity_planner/modules/virtual_traffic_light_core.cpp
  src/behavior_velocity_planner/modules/blind_spot_core.cpp
  src/behavior_velocity_planner/crosswalk/crosswalk_collision.cpp
  src/behavior_velocity_planner/crosswalk/crosswalk_braking.cpp
  src/behavior_velocity_planner/crosswalk/crosswalk_object_state.cpp
  src/behavior_velocity_planner/crosswalk/crosswalk_occlusion.cpp
  src/behavior_velocity_planner/modules/occlusion_spot_core.cpp
  src/behavior_velocity_planner/modules/occlusion_path_resample.cpp
  src/behavior_velocity_planner/traffic_light/traffic_light_core.cpp
  src/behavior_velocity_planner/traffic_light/traffic_light_signal.cpp
  src/behavior_path_planner/behavior_path_postprocess.cpp
  src/behavior_path_planner/behavior_path_shift.cpp
  src/behavior_path_planner/behavior_path_lane_safety.cpp
  src/behavior_path_planner/behavior_path_object_safety.cpp
  src/behavior_path_planner/behavior_path_prediction.cpp
  src/behavior_path_planner/behavior_lane_change_safety.cpp
  src/behavior_path_planner/behavior_candidate_callback.cpp
  src/behavior_velocity_planner/core/behavior_velocity_perception_bridge.cpp
  src/pipeline/planning_pipeline_cruise_bridge.cpp
  src/behavior_path_planner/behavior_lane_change_route.cpp
  src/behavior_path_planner/behavior_static_avoidance.cpp
  src/behavior_path_planner/behavior_route_action.cpp
  src/behavior_path_planner/behavior_frenet_sampling.cpp
  src/behavior_path_planner/behavior_dynamic_avoidance.cpp
  src/behavior_path_planner/behavior_pull_over_core.cpp
  src/behavior_path_planner/behavior_freespace_pull_over.cpp
  src/behavior_path_planner/behavior_freespace_rrtstar.cpp
  lib/autoware_reeds_shepp/src/reeds_shepp.cpp
  lib/autoware_reeds_shepp/src/rrtstar_core.cpp
  src/behavior_path_planner/behavior_goal_refinement.cpp
  src/behavior_path_planner/behavior_goal_safety.cpp
  src/path_smoother/elastic_band_core.cpp
  src/path_optimizer/path_optimizer_model.cpp
  src/path_optimizer/path_optimizer_qp.cpp
  src/path_optimizer/path_optimizer_solution.cpp
  src/path_optimizer/path_optimizer_reference_geometry.cpp
  src/path_optimizer/path_optimizer_bounds.cpp
  src/path_optimizer/path_optimizer_reference_data.cpp
  src/path_optimizer/path_optimizer_core.cpp
  src/path_optimizer/path_optimizer_reset.cpp
  src/motion_velocity_planner/motion_velocity_core.cpp
  src/obstacle_cruise_planner/obstacle_cruise_stop_core.cpp
  src/obstacle_cruise_planner/obstacle_cruise_optimization.cpp
  src/obstacle_cruise_planner/obstacle_cruise_slowdown_core.cpp
  src/obstacle_cruise_planner/obstacle_cruise_pid_core.cpp
  src/obstacle_cruise_planner/obstacle_cruise_candidate.cpp
  src/obstacle_cruise_planner/obstacle_cruise_collision.cpp
  src/obstacle_cruise_planner/obstacle_cruise_pointcloud.cpp
  src/adapters/pointcloud2_adapter.cpp
  src/motion_velocity_planner/dynamic_obstacle_stop_core.cpp
  src/motion_velocity_planner/obstacle_velocity_limiter_core.cpp
  src/motion_velocity_planner/obstacle_velocity_limiter_bicycle.cpp
  src/motion_velocity_planner/obstacle_velocity_limiter_pointcloud.cpp
  src/motion_velocity_planner/out_of_lane_core.cpp
  src/motion_velocity_planner/out_of_lane_geometry.cpp
  src/motion_velocity_planner/out_of_lane_stop_line.cpp
  src/motion_velocity_planner/out_of_lane_object_filter.cpp
  src/external_velocity_limit_selector/velocity_limit_selector.cpp
  src/velocity_smoother/jerk_filter.cpp
  src/velocity_smoother/jerk_resample.cpp
  src/velocity_smoother/jerk_smoother.cpp
  src/velocity_smoother/jerk_node_policy.cpp
  src/velocity_smoother/jerk_path_limits.cpp
  src/velocity_smoother/jerk_external_limit.cpp
  src/velocity_smoother/analytical_velocity_smoother.cpp
  src/velocity_smoother/trajectory_precheck.cpp
  src/planning_validator/planning_validator_core.cpp
  src/planning_validator/planning_validator_diagnostics.cpp)
target_sources(planning_core PRIVATE src/pipeline/planning_pipeline_core.cpp)
target_sources(planning_core PRIVATE src/velocity_smoother/jerk_qp_problem.cpp src/velocity_smoother/jerk_qp_solver.cpp)
set(PROXSUITE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/proxsuite-devel")
option(PLANNING_USE_PROXQP "제공된 ProxSuite 소스를 사용해 ProxQP로 풉니다" ON)
set(PLANNING_HAS_PROXQP OFF)
if(PLANNING_USE_PROXQP AND
   EXISTS "${PROXSUITE_SOURCE_DIR}/include/proxsuite/proxqp/sparse/sparse.hpp")
  set(PLANNING_HAS_PROXQP ON)
  file(READ "${PROXSUITE_SOURCE_DIR}/package.xml" PROXSUITE_PACKAGE_XML)
  string(REGEX MATCH "<version>([0-9]+)\\.([0-9]+)\\.([0-9]+)</version>"
    PROXSUITE_VERSION_MATCH "${PROXSUITE_PACKAGE_XML}")
  if(NOT PROXSUITE_VERSION_MATCH)
    message(FATAL_ERROR "ProxSuite 버전을 package.xml에서 읽을 수 없습니다")
  endif()
  set(PROXSUITE_VERSION_MAJOR "${CMAKE_MATCH_1}")
  set(PROXSUITE_VERSION_MINOR "${CMAKE_MATCH_2}")
  set(PROXSUITE_VERSION_PATCH "${CMAKE_MATCH_3}")
  configure_file(cmake/proxsuite_config.hpp.in
    "${CMAKE_CURRENT_BINARY_DIR}/generated/proxsuite/config.hpp" @ONLY)
  target_sources(planning_core PRIVATE src/velocity_smoother/jerk_proxqp_solver.cpp)
  target_include_directories(planning_core PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/generated" "${PROXSUITE_SOURCE_DIR}/include")
  target_compile_definitions(planning_core PRIVATE PLANNING_HAS_PROXQP=1)
endif()
target_include_directories(planning_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_include_directories(planning_core PRIVATE ${Boost_INCLUDE_DIRS}
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/autoware_reeds_shepp/include")
target_link_libraries(planning_core PUBLIC Eigen3::Eigen)
target_compile_features(planning_core PUBLIC cxx_std_17)
target_compile_options(planning_core PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS planning_core EXPORT planning_core_targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/planning DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(PROGRAMS ros/goal_pose_visualizer.py
  DESTINATION ${CMAKE_INSTALL_BINDIR}
  RENAME planning_aa_goal_pose_visualizer)
install(EXPORT planning_core_targets
  FILE planning_core_targets.cmake
  NAMESPACE planning::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/planning_core)
configure_package_config_file(
  cmake/planning_core_config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/planning_core-config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/planning_core)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/planning_core-config-version.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/planning_core-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/planning_core-config-version.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/planning_core)

option(PLANNING_BUILD_GRID_ADAPTER
  "OpenCV가 있으면 점유 격자 장애물 어댑터를 빌드합니다" ON)
if(PLANNING_BUILD_GRID_ADAPTER)
  find_package(OpenCV QUIET COMPONENTS core imgproc)
else()
  set(OpenCV_FOUND OFF)
endif()
if(OpenCV_FOUND)
  add_library(planning_grid_adapter STATIC
    src/motion_velocity_planner/obstacle_velocity_limiter_grid.cpp)
  target_include_directories(planning_grid_adapter PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(planning_grid_adapter PUBLIC
    planning_core ${OpenCV_LIBS})
  target_compile_features(planning_grid_adapter PUBLIC cxx_std_17)
  target_compile_options(planning_grid_adapter PRIVATE -Wall -Wextra -Wpedantic)
endif()

option(PLANNING_BUILD_OSM_ADAPTER
  "TinyXML2가 있으면 OSM 지도 어댑터를 빌드합니다" ON)
if(PLANNING_BUILD_OSM_ADAPTER)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(TINYXML2 QUIET IMPORTED_TARGET tinyxml2)
  endif()
else()
  set(TINYXML2_FOUND OFF)
endif()
if(TINYXML2_FOUND)
  add_library(planning_osm_adapter STATIC src/adapters/lanelet_osm_adapter.cpp)
  target_include_directories(planning_osm_adapter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(planning_osm_adapter PUBLIC planning_core PkgConfig::TINYXML2)
  target_compile_features(planning_osm_adapter PUBLIC cxx_std_17)
  target_compile_options(planning_osm_adapter PRIVATE -Wall -Wextra -Wpedantic)
endif()

set(ROSLESS_LANELET2_SOURCE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/Rosless-Lanelet2"
  CACHE PATH "저장소에 포함된 Rosless-Lanelet2 소스 위치")
set(PLANNING_LANELET_BIN_DEFAULT OFF)
if(EXISTS "${ROSLESS_LANELET2_SOURCE_DIR}/lanelet2_core/CMakeLists.txt" AND
   EXISTS "${ROSLESS_LANELET2_SOURCE_DIR}/lanelet2_io/include/lanelet2_io/io_handlers/Serialize.h")
  find_package(Boost QUIET COMPONENTS serialization)
  if(Boost_FOUND)
    set(PLANNING_LANELET_BIN_DEFAULT ON)
  endif()
endif()
option(PLANNING_BUILD_LANELET_BIN_ADAPTER
  "저장소의 Rosless-Lanelet2 소스로 이진 지도 어댑터를 빌드합니다"
  ${PLANNING_LANELET_BIN_DEFAULT})
include(CTest)
if(PLANNING_BUILD_LANELET_BIN_ADAPTER)
  if(NOT EXISTS "${ROSLESS_LANELET2_SOURCE_DIR}/lanelet2_core/CMakeLists.txt" OR
     NOT EXISTS "${ROSLESS_LANELET2_SOURCE_DIR}/lanelet2_io/include/lanelet2_io/io_handlers/Serialize.h")
    message(FATAL_ERROR "Rosless-Lanelet2 소스를 찾을 수 없습니다")
  endif()
  if(NOT Boost_FOUND)
    find_package(Boost REQUIRED COMPONENTS serialization)
  endif()
  add_subdirectory("${ROSLESS_LANELET2_SOURCE_DIR}/lanelet2_core"
    "${CMAKE_CURRENT_BINARY_DIR}/lanelet2_core" EXCLUDE_FROM_ALL)
  add_subdirectory("${ROSLESS_LANELET2_SOURCE_DIR}/lanelet2_traffic_rules"
    "${CMAKE_CURRENT_BINARY_DIR}/lanelet2_traffic_rules" EXCLUDE_FROM_ALL)
  add_subdirectory("${ROSLESS_LANELET2_SOURCE_DIR}/lanelet2_routing"
    "${CMAKE_CURRENT_BINARY_DIR}/lanelet2_routing" EXCLUDE_FROM_ALL)
  add_library(planning_lanelet_bin_adapter STATIC
    src/costmap_generator/costmap_map_adapter.cpp
    src/planning_evaluator/planning_evaluator_map_adapter.cpp
    src/adapters/lanelet_bin_adapter.cpp
    src/mission_planner/mission_route_map_adapter.cpp
    src/remaining_distance_time_calculator/remaining_distance_map_adapter.cpp
    src/behavior_velocity_planner/core/behavior_velocity_map_planner.cpp
    src/pipeline/planning_pipeline_map_adapter.cpp)
  target_include_directories(planning_lanelet_bin_adapter PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_include_directories(planning_lanelet_bin_adapter PRIVATE
    "${ROSLESS_LANELET2_SOURCE_DIR}/lanelet2_io/include")
  target_link_libraries(planning_lanelet_bin_adapter PUBLIC
    planning_core lanelet2_core lanelet2_routing Boost::serialization)
  target_compile_features(planning_lanelet_bin_adapter PUBLIC cxx_std_17)
  target_compile_options(planning_lanelet_bin_adapter PRIVATE
    -Wall -Wextra -Wpedantic)
  if(BUILD_TESTING)
  add_executable(blind_spot_rosbag_runner tests/blind_spot_rosbag_runner.cpp)
  target_link_libraries(blind_spot_rosbag_runner PRIVATE
    planning_lanelet_bin_adapter)
  target_compile_options(blind_spot_rosbag_runner PRIVATE
    -Wall -Wextra -Wpedantic)
  add_executable(stop_line_rosbag_runner tests/stop_line_rosbag_runner.cpp)
  target_link_libraries(stop_line_rosbag_runner PRIVATE
    planning_lanelet_bin_adapter)
  target_compile_options(stop_line_rosbag_runner PRIVATE
    -Wall -Wextra -Wpedantic)
  add_executable(traffic_light_rosbag_runner
    tests/traffic_light_rosbag_runner.cpp)
  target_link_libraries(traffic_light_rosbag_runner PRIVATE
    planning_lanelet_bin_adapter)
  target_compile_options(traffic_light_rosbag_runner PRIVATE
    -Wall -Wextra -Wpedantic)
  add_executable(crosswalk_rosbag_runner tests/crosswalk_rosbag_runner.cpp)
  target_link_libraries(crosswalk_rosbag_runner PRIVATE
    planning_lanelet_bin_adapter)
  target_compile_options(crosswalk_rosbag_runner PRIVATE
    -Wall -Wextra -Wpedantic)
  add_executable(behavior_goal_rosbag_runner
    tests/behavior_goal_rosbag_runner.cpp)
  target_link_libraries(behavior_goal_rosbag_runner PRIVATE
    planning_lanelet_bin_adapter)
  target_compile_options(behavior_goal_rosbag_runner PRIVATE
    -Wall -Wextra -Wpedantic)
  add_executable(behavior_path_lane_rosbag_runner
    tests/behavior_path_lane_rosbag_runner.cpp)
  target_link_libraries(behavior_path_lane_rosbag_runner PRIVATE
    planning_lanelet_bin_adapter)
  target_compile_options(behavior_path_lane_rosbag_runner PRIVATE
    -Wall -Wextra -Wpedantic)
  add_executable(behavior_start_centerline_probe
    tests/behavior_start_centerline_probe.cpp)
  target_link_libraries(behavior_start_centerline_probe PRIVATE
    planning_lanelet_bin_adapter)
  target_compile_options(behavior_start_centerline_probe PRIVATE
    -Wall -Wextra -Wpedantic)
  add_executable(route_reference_geometry_probe
    tests/route_reference_geometry_probe.cpp)
  target_link_libraries(route_reference_geometry_probe PRIVATE
    planning_lanelet_bin_adapter)
  target_compile_options(route_reference_geometry_probe PRIVATE
    -Wall -Wextra -Wpedantic)
  add_executable(behavior_start_candidate_rosbag_runner
    tests/behavior_start_candidate_rosbag_runner.cpp)
  target_link_libraries(behavior_start_candidate_rosbag_runner PRIVATE
    planning_lanelet_bin_adapter)
  target_compile_options(behavior_start_candidate_rosbag_runner PRIVATE
    -Wall -Wextra -Wpedantic)
  endif()
endif()

if(BUILD_TESTING)
  add_executable(costmap_generator_core_test
    tests/costmap_generator_core_test.cpp)
  target_link_libraries(costmap_generator_core_test PRIVATE planning_core)
  add_test(NAME costmap_generator_core COMMAND costmap_generator_core_test)
  add_executable(freespace_planner_core_test
    tests/freespace_planner_core_test.cpp)
  target_link_libraries(freespace_planner_core_test PRIVATE planning_core)
  add_test(NAME freespace_planner_core COMMAND freespace_planner_core_test)
  add_executable(freespace_planner_reference_runner
    tests/freespace_planner_reference_runner.cpp)
  target_link_libraries(freespace_planner_reference_runner PRIVATE planning_core)
  add_executable(planning_evaluator_core_test
    tests/planning_evaluator_core_test.cpp)
  target_link_libraries(planning_evaluator_core_test PRIVATE planning_core)
  add_test(NAME planning_evaluator_core COMMAND planning_evaluator_core_test)
  add_executable(planning_evaluator_rosbag_runner
    tests/planning_evaluator_rosbag_runner.cpp)
  target_link_libraries(planning_evaluator_rosbag_runner PRIVATE planning_core)
  add_executable(behavior_dynamic_drivable_area_test
    tests/behavior_dynamic_drivable_area_test.cpp)
  target_link_libraries(behavior_dynamic_drivable_area_test PRIVATE planning_core)
  add_test(NAME behavior_dynamic_drivable_area
    COMMAND behavior_dynamic_drivable_area_test)
  add_executable(remaining_distance_time_core_test
    tests/remaining_distance_time_core_test.cpp)
  target_link_libraries(remaining_distance_time_core_test PRIVATE planning_core)
  add_test(NAME remaining_distance_time_core
    COMMAND remaining_distance_time_core_test)
  add_executable(mission_planner_core_test tests/mission_planner_core_test.cpp)
  target_link_libraries(mission_planner_core_test PRIVATE planning_core)
  add_test(NAME mission_planner_core COMMAND mission_planner_core_test)
  add_executable(planning_route_coordinator_test
    tests/planning_route_coordinator_test.cpp)
  target_link_libraries(planning_route_coordinator_test PRIVATE planning_core)
  add_test(NAME planning_route_coordinator COMMAND planning_route_coordinator_test)
  add_executable(planning_final_route_state_test
    tests/planning_final_route_state_test.cpp)
  target_link_libraries(planning_final_route_state_test PRIVATE planning_core)
  add_test(NAME planning_final_route_state COMMAND planning_final_route_state_test)
  add_executable(mission_reroute_safety_test
    tests/mission_reroute_safety_test.cpp)
  target_link_libraries(mission_reroute_safety_test PRIVATE planning_core)
  add_test(NAME mission_reroute_safety
    COMMAND mission_reroute_safety_test)
  add_executable(route_selector_core_test tests/route_selector_core_test.cpp)
  target_link_libraries(route_selector_core_test PRIVATE planning_core)
  add_test(NAME route_selector_core COMMAND route_selector_core_test)
  if(OpenCV_FOUND)
    add_executable(obstacle_velocity_limiter_grid_test
      tests/obstacle_velocity_limiter_grid_test.cpp)
    target_link_libraries(obstacle_velocity_limiter_grid_test PRIVATE
      planning_grid_adapter)
    target_compile_options(obstacle_velocity_limiter_grid_test PRIVATE
      -Wall -Wextra -Wpedantic)
    add_test(NAME obstacle_velocity_limiter_grid
      COMMAND obstacle_velocity_limiter_grid_test)
  endif()
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(NAME freespace_planner_synthetic_reference_replay
    COMMAND Python3::Interpreter
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/freespace_planner_reference_replay.py
      $<TARGET_FILE:freespace_planner_reference_runner>
      ${CMAKE_CURRENT_SOURCE_DIR}/reference/freespace_planner_synthetic_reference.json
      --completion-reference
      ${CMAKE_CURRENT_SOURCE_DIR}/reference/freespace_planner_synthetic_completion_reference.json)
  add_test(NAME freespace_planner_synthetic_obstacle_reference_replay
    COMMAND Python3::Interpreter
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/freespace_planner_reference_replay.py
      $<TARGET_FILE:freespace_planner_reference_runner>
      ${CMAKE_CURRENT_SOURCE_DIR}/reference/freespace_planner_synthetic_obstacle_segment_reference.json
      --check-next-prefix 3
      --maximum-full-path-position-error 0.27
      --maximum-full-path-endpoint-error 0.09)
  add_test(NAME freespace_planner_generated_grid_no_adapt_reference_replay
    COMMAND Python3::Interpreter
      ${CMAKE_CURRENT_SOURCE_DIR}/reference/freespace_generated_grid_diagnostic.py
      $<TARGET_FILE:freespace_planner_reference_runner>
      ${CMAKE_CURRENT_SOURCE_DIR}/reference/freespace_planner_generated_grid_no_adapt_reference.json
      ${CMAKE_CURRENT_SOURCE_DIR}/reference/costmap_generator_reference_scurve_synthetic.json.gz
      --expect-match)
  add_test(NAME freespace_planner_generated_grid_terminal_reference_replay
    COMMAND Python3::Interpreter
      ${CMAKE_CURRENT_SOURCE_DIR}/reference/freespace_generated_grid_diagnostic.py
      $<TARGET_FILE:freespace_planner_reference_runner>
      ${CMAKE_CURRENT_SOURCE_DIR}/reference/freespace_planner_generated_grid_reference.json
      ${CMAKE_CURRENT_SOURCE_DIR}/reference/costmap_generator_reference_scurve_synthetic.json.gz
      --expect-terminal-match --maximum-position-error 0.12)
  if(PLANNING_BUILD_LANELET_BIN_ADAPTER)
    add_executable(planning_generated_costmap_test
      tests/planning_generated_costmap_test.cpp)
    target_link_libraries(planning_generated_costmap_test PRIVATE
      planning_lanelet_bin_adapter)
    add_test(NAME planning_generated_costmap
      COMMAND planning_generated_costmap_test
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_parking_lot_synthetic_inner.bin)
    add_executable(costmap_map_adapter_test
      tests/costmap_map_adapter_test.cpp)
    target_link_libraries(costmap_map_adapter_test PRIVATE
      planning_lanelet_bin_adapter)
    add_test(NAME costmap_map_adapter
      COMMAND costmap_map_adapter_test
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_parking_lot_synthetic_inner.bin)
    add_executable(costmap_generator_reference_runner
      tests/costmap_generator_reference_runner.cpp)
    target_link_libraries(costmap_generator_reference_runner PRIVATE
      planning_lanelet_bin_adapter)
    add_executable(mission_route_map_adapter_test
      tests/mission_route_map_adapter_test.cpp)
    target_link_libraries(mission_route_map_adapter_test PRIVATE
      planning_lanelet_bin_adapter)
    add_executable(planning_evaluator_lane_reference_runner
      tests/planning_evaluator_lane_reference_runner.cpp)
    target_link_libraries(planning_evaluator_lane_reference_runner PRIVATE
      planning_lanelet_bin_adapter)
    add_test(NAME mission_route_map_adapter
      COMMAND mission_route_map_adapter_test
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin)
    add_executable(remaining_distance_rosbag_runner
      tests/remaining_distance_rosbag_runner.cpp)
    target_link_libraries(remaining_distance_rosbag_runner PRIVATE
      planning_lanelet_bin_adapter)
    if(EXISTS "${PLANNING_KCITY_IPC_STOPLINE_BAG}")
      add_test(NAME remaining_distance_ipc_stopline_rosbag_replay
        COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_CURRENT_SOURCE_DIR}/tests/remaining_distance_rosbag_replay.py
          $<TARGET_FILE:remaining_distance_rosbag_runner>
          - ${PLANNING_KCITY_IPC_STOPLINE_BAG}
          --maximum-distance-error-m 0.08
          --maximum-frames-above-1cm 2)
    endif()
    if(EXISTS "${PLANNING_KCITY_SCURVE_BAG}")
      add_test(NAME remaining_distance_hpc_scurve_rosbag_replay
        COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_CURRENT_SOURCE_DIR}/tests/remaining_distance_rosbag_replay.py
          $<TARGET_FILE:remaining_distance_rosbag_runner>
          - ${PLANNING_KCITY_SCURVE_BAG}
          --maximum-distance-error-m 0.03
          --maximum-frames-above-1cm 1)
    endif()
    add_executable(lanelet_bin_adapter_test tests/lanelet_bin_adapter_test.cpp)
    add_executable(planning_route_action_map_test
      tests/planning_route_action_map_test.cpp)
    target_link_libraries(planning_route_action_map_test PRIVATE
      planning_lanelet_bin_adapter)
    target_include_directories(planning_route_action_map_test PRIVATE
      "${ROSLESS_LANELET2_SOURCE_DIR}/lanelet2_io/include")
    target_compile_options(planning_route_action_map_test PRIVATE
      -Wall -Wextra -Wpedantic)
    add_test(NAME planning_route_action_map
      COMMAND planning_route_action_map_test)
    target_link_libraries(lanelet_bin_adapter_test PRIVATE
      planning_lanelet_bin_adapter)
    target_include_directories(lanelet_bin_adapter_test PRIVATE
      "${ROSLESS_LANELET2_SOURCE_DIR}/lanelet2_io/include")
    target_compile_options(lanelet_bin_adapter_test PRIVATE
      -Wall -Wextra -Wpedantic)
    add_executable(planning_pipeline_map_reference_runner
      tests/planning_pipeline_map_reference_runner.cpp)
    add_executable(planning_pipeline_rosbag_runner
      tests/planning_pipeline_rosbag_runner.cpp)
    target_link_libraries(planning_pipeline_rosbag_runner PRIVATE
      planning_lanelet_bin_adapter)
    target_link_libraries(planning_pipeline_map_reference_runner PRIVATE
      planning_lanelet_bin_adapter)
    target_compile_options(planning_pipeline_map_reference_runner PRIVATE
      -Wall -Wextra -Wpedantic)
    add_test(NAME planning_pipeline_map_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/planning_pipeline_map_reference_replay.py
        $<TARGET_FILE:planning_pipeline_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_stop_sign_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/behavior_velocity_map_combined_stopline.json)
    add_executable(static_obstacle_velocity_rosbag_runner
      tests/static_obstacle_velocity_rosbag_runner.cpp)
    target_link_libraries(static_obstacle_velocity_rosbag_runner PRIVATE
      planning_lanelet_bin_adapter)
    target_compile_options(static_obstacle_velocity_rosbag_runner PRIVATE
      -Wall -Wextra -Wpedantic)
    add_executable(out_of_lane_rosbag_runner
      tests/out_of_lane_rosbag_runner.cpp)
    target_link_libraries(out_of_lane_rosbag_runner PRIVATE
      planning_lanelet_bin_adapter)
    target_compile_options(out_of_lane_rosbag_runner PRIVATE
      -Wall -Wextra -Wpedantic)
    add_test(NAME lanelet_bin_adapter
      COMMAND lanelet_bin_adapter_test
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_stop_sign_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_signal_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_intersection_signal_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_intersection_control_synthetic_inner.bin)

    add_executable(scenario_position_reference_runner
      tests/scenario_position_reference_runner.cpp)
    target_link_libraries(scenario_position_reference_runner PRIVATE
      planning_lanelet_bin_adapter)
    target_compile_options(scenario_position_reference_runner PRIVATE
      -Wall -Wextra -Wpedantic)
    add_test(NAME scenario_position_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/scenario_position_reference_replay.py
        $<TARGET_FILE:scenario_position_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/scenario_position_reference_pedestrian.json)
    foreach(PLANNING_SCENARIO_CASE stopline cutinout)
      add_test(NAME scenario_position_${PLANNING_SCENARIO_CASE}_reference_replay
        COMMAND Python3::Interpreter
          ${CMAKE_CURRENT_SOURCE_DIR}/tests/scenario_position_reference_replay.py
          $<TARGET_FILE:scenario_position_reference_runner>
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/scenario_position_reference_${PLANNING_SCENARIO_CASE}.json)
    endforeach()

    add_executable(scenario_parking_reference_runner
      tests/scenario_parking_reference_runner.cpp)
    target_link_libraries(scenario_parking_reference_runner PRIVATE
      planning_lanelet_bin_adapter)
    target_compile_options(scenario_parking_reference_runner PRIVATE
      -Wall -Wextra -Wpedantic)
    add_test(NAME scenario_parking_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/scenario_parking_reference_replay.py
        $<TARGET_FILE:scenario_parking_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_parking_lot_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/scenario_position_reference_stopline.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/scenario_selector_reference_synthetic_parking.json)
    add_test(NAME scenario_parking_timeout_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/scenario_parking_timeout_reference_replay.py
        $<TARGET_FILE:scenario_parking_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_parking_lot_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/scenario_position_reference_stopline.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/scenario_selector_reference_parking_timeout.json)
    add_test(NAME scenario_parking_empty_timeout_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/scenario_parking_empty_timeout_reference_replay.py
        $<TARGET_FILE:scenario_parking_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_parking_lot_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/scenario_position_reference_stopline.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/scenario_selector_reference_parking_empty_timeout.json)

    add_test(NAME detection_area_map_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/detection_area_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_detection_area_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/detection_area_clear_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/detection_area_stop_reference.json)
    add_test(NAME no_stopping_area_map_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/no_stopping_area_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_no_stopping_area_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_stopping_area_clear_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_stopping_area_stuck_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_stopping_area_fast_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_stopping_area_bicycle_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_stopping_area_motorcycle_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_stopping_area_footprint_edge_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_stopping_area_footprint_outside_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_stopping_area_rotated_footprint_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_stopping_area_offpath_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_stopping_area_start_inside_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_stopping_area_ego_past_stop_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_stopping_area_high_speed_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_stopping_area_brake_limit_fast_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_stopping_area_brake_limit_slow_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_stopping_area_downstream_stop_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_stopping_area_downstream_stop31_reference.json)
    add_test(NAME no_stopping_area_auto_line_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/no_stopping_area_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_no_stopping_area_auto_line_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_stopping_area_auto_line_clear_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_stopping_area_auto_line_stuck_reference.json)
    add_test(NAME private_merge_map_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/private_merge_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_private_merge_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/private_merge_initial_reference.json)
    add_test(NAME private_merge_normal_road_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/private_merge_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/private_merge_normal_road_reference.json)
    add_test(NAME private_merge_continues_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/private_merge_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_private_continue_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/private_merge_continues_reference.json)
    add_test(NAME private_merge_state_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/private_merge_state_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_private_merge_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/private_merge_state_reference.json)
    add_test(NAME speed_bump_map_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/speed_bump_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_speed_bump_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/speed_bump_reference.json)
    add_test(NAME speed_bump_override_map_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/speed_bump_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_speed_bump_override_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/speed_bump_override_reference.json)
    add_test(NAME speed_bump_inside_map_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/speed_bump_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_speed_bump_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/speed_bump_inside_reference.json)
    add_test(NAME speed_bump_truncated_map_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/speed_bump_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_speed_bump_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/speed_bump_truncated_reference.json)
    add_test(NAME speed_bump_default_height_map_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/speed_bump_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_speed_bump_default_height_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/speed_bump_default_height_reference.json)
    add_test(NAME no_drivable_lane_map_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/no_drivable_lane_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_no_drivable_lane_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_drivable_lane_reference.json)
    add_test(NAME no_drivable_lane_inside_map_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/no_drivable_lane_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_no_drivable_lane_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_drivable_lane_inside_reference.json)
    add_test(NAME no_drivable_lane_inside_stopped_map_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/no_drivable_lane_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_no_drivable_lane_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_drivable_lane_inside_stopped_reference.json)
    add_test(NAME no_drivable_lane_ego_inside_map_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/no_drivable_lane_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_no_drivable_lane_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/no_drivable_lane_ego_inside_reference.json)

    add_executable(behavior_velocity_map_reference_runner
      tests/behavior_velocity_map_reference_runner.cpp)
    add_executable(mission_route_reference_runner
      tests/mission_route_reference_runner.cpp)
    target_link_libraries(mission_route_reference_runner PRIVATE
      planning_lanelet_bin_adapter)
    target_compile_options(mission_route_reference_runner PRIVATE
      -Wall -Wextra -Wpedantic)
    add_test(NAME mission_route_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_reference.json)
    add_test(NAME mission_route_pedestrian_bag_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_pedestrian_reference.json)
    add_test(NAME mission_route_pedestrian_lane_change_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_pedestrian_0_25_reference.json)
    add_test(NAME mission_route_pedestrian_late_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_pedestrian_0_6_reference.json)
    add_test(NAME mission_route_pedestrian_start_right_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_pedestrian_start_offset_0_reference.json)
    add_test(NAME mission_route_pedestrian_start_left_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_pedestrian_start_offset_1_reference.json)
    add_test(NAME mission_route_cutinout_bag_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_cutinout_reference.json)
    add_test(NAME mission_route_cutinout_second_time_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_cutinout_0_5_reference.json)
    add_test(NAME mission_route_same_lane_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_same_lane_reference.json)
    add_test(NAME mission_route_adjacent_left_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_early_goal_offset_0_reference.json)
    add_test(NAME mission_route_adjacent_right_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_early_goal_offset_1_reference.json)
    add_test(NAME mission_route_turn_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_turn_reference.json)
    add_test(NAME mission_route_waypoint_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_waypoint_reference.json)
    add_test(NAME mission_route_adjacent_left_waypoint_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_adjacent_waypoint_0_reference.json)
    add_test(NAME mission_route_adjacent_right_waypoint_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_adjacent_waypoint_1_reference.json)
    add_test(NAME mission_route_cross_lanes_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_cross_lanes_reference.json)
    add_test(NAME mission_route_offset_start_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_offset_start_reference.json)
    add_test(NAME mission_route_wrong_goal_yaw_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_wrong_goal_yaw_reference.json)
    add_test(NAME mission_route_far_start_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_far_start_reference.json)
    add_test(NAME mission_route_no_drivable_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_route_no_drivable_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_no_drivable_reference.json)
    add_test(NAME mission_route_goal_height_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_goal_height_reference.json)
    add_test(NAME mission_route_goal_footprint_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_goal_edge_north_reference.json)
    add_test(NAME mission_route_goal_next_lane_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_end_extended_reference.json)
    add_test(NAME mission_route_parking_lot_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_parking_lot_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_parking_edge_reference.json)
    add_test(NAME mission_route_parking_space_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_parking_space_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_parking_space_reference.json)
    add_test(NAME mission_route_shoulder_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_shoulder_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_shoulder_reference.json)
    add_test(NAME mission_route_shoulder_wrong_yaw_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_shoulder_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_shoulder_wrong_yaw_reference.json)
    add_test(NAME mission_route_waypoint_offset_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_waypoint_offset_reference.json)
    add_test(NAME mission_route_two_waypoints_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_two_waypoints_reference.json)
    add_test(NAME mission_route_reverse_waypoints_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_reverse_waypoints_reference.json)
    add_test(NAME mission_route_start_left_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_start_left_far_reference.json)
    add_test(NAME mission_route_start_right_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_start_right_far_reference.json)
    add_test(NAME mission_route_drivable_alternative_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_drivable_alternative_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_drivable_alternative_reference.json)
    add_test(NAME mission_route_drivable_alternative_disabled_reference_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mission_route_reference_replay.py
        $<TARGET_FILE:mission_route_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_drivable_alternative_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/mission_planner_route_drivable_alternative_disabled_reference.json)
    add_executable(intersection_stuck_reference_runner
      tests/intersection_stuck_reference_runner.cpp)
    target_link_libraries(intersection_stuck_reference_runner PRIVATE
      planning_lanelet_bin_adapter)
    target_compile_options(intersection_stuck_reference_runner PRIVATE
      -Wall -Wextra -Wpedantic)
    add_test(NAME intersection_stuck_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/intersection_stuck_reference_replay.py
        $<TARGET_FILE:intersection_stuck_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stopline_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stopline_stuck_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stuck_unknown_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stuck_next_boundary_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stuck_next_near_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stuck_next_far_reference.json)
    add_test(NAME intersection_stuck_map_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/intersection_stuck_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stopline_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stopline_stuck_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stuck_unknown_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stuck_next_boundary_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stuck_next_near_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stuck_next_far_reference.json)
    add_test(NAME intersection_pedestrian_stuck_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/intersection_pedestrian_stuck_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_pedestrian_stuck_detected_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_pedestrian_stuck_outside_reference.json)
    add_executable(intersection_collision_reference_runner
      tests/intersection_collision_reference_runner.cpp)
    add_executable(intersection_collision_state_runner
      tests/intersection_collision_state_runner.cpp)
    target_link_libraries(intersection_collision_state_runner PRIVATE planning_core)
    target_compile_options(intersection_collision_state_runner PRIVATE
      -Wall -Wextra -Wpedantic)
    add_test(NAME intersection_collision_state_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/intersection_collision_state_replay.py
        $<TARGET_FILE:intersection_collision_state_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_collision_sequence_reference.json)
    target_link_libraries(intersection_collision_reference_runner PRIVATE
      planning_lanelet_bin_adapter)
    target_compile_options(intersection_collision_reference_runner PRIVATE
      -Wall -Wextra -Wpedantic)
    add_test(NAME intersection_collision_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/intersection_collision_reference_replay.py
        $<TARGET_FILE:intersection_collision_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stopline_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stopline_collision_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stopline_collision_early_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stopline_collision_late_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stopline_collision_slow_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stopline_collision_fast_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stopline_collision_ego_lane_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stopline_collision_ego_approach_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_collision_unknown_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_collision_low_confidence_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_collision_two_paths_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_collision_two_paths_low_reference.json)
    add_test(NAME intersection_pedestrian_second_turn_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/intersection_collision_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_pedestrian_second_turn_reference.json)
    add_test(NAME intersection_collision_map_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/intersection_collision_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stopline_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stopline_collision_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stopline_collision_early_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stopline_collision_late_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stopline_collision_slow_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stopline_collision_fast_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stopline_collision_ego_lane_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_stopline_collision_ego_approach_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_collision_unknown_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_collision_low_confidence_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_collision_two_paths_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_collision_two_paths_low_reference.json)
    add_test(NAME intersection_collision_signal_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/intersection_collision_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_intersection_signal_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_collision_signal_linked_red_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_collision_signal_linked_green_reference.json)
    add_test(NAME intersection_collision_control_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/intersection_collision_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_intersection_control_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_collision_control_red_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_collision_control_green_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_collision_control_edge_green_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_collision_control_near_green_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_collision_control_near_yellow_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_collision_control_yellow_at_line_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_collision_control_far_green_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_collision_control_yellow_reference.json)
    add_test(NAME intersection_occlusion_signal_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/intersection_occlusion_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_intersection_signal_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_free_green_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_unknown_green_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_patch_green_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_offlane_green_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_adjacent_lane_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_small_patch_green_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_mid_patch_green_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_boundary34_patch_green_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_boundary_patch_green_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_far_north_green_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_north60_green_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_north62_green_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_north64_green_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_distant_patch_green_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_far_patch_green_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_unknown_red_reference.json)
    add_test(NAME intersection_occlusion_parked_shadow_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/intersection_occlusion_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_intersection_signal_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_parked_shadow_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_north_parked_clear_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_north_moving_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_north_yellow_parked_reference.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_north_yellow_moving_reference.json)
    add_test(NAME intersection_occlusion_no_signal_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/intersection_occlusion_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_unknown_reference.json)
    add_test(NAME intersection_occlusion_unsignalized_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/intersection_occlusion_map_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_intersection_no_signal_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_no_signal_unknown_reference.json)
    add_test(NAME intersection_occlusion_sequence_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/intersection_occlusion_sequence_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_intersection_signal_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_clear_sequence_reference.json)
    add_test(NAME intersection_occlusion_static_release_sequence
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/intersection_occlusion_release_sequence_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_intersection_signal_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_unknown_green_reference.json)
    add_test(NAME intersection_occlusion_signal_transition_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/intersection_occlusion_sequence_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_intersection_signal_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/intersection_occlusion_green_yellow_sequence_reference.json)
    target_link_libraries(behavior_velocity_map_reference_runner PRIVATE
      planning_lanelet_bin_adapter)
    target_compile_options(behavior_velocity_map_reference_runner PRIVATE
      -Wall -Wextra -Wpedantic)
    add_test(NAME behavior_velocity_map_combined_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/behavior_velocity_map_reference_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_stop_sign_synthetic_inner.bin
        combined
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/behavior_velocity_map_combined_stopline.json)
    add_test(NAME behavior_velocity_map_crosswalk_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/behavior_velocity_map_reference_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        crosswalk
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/behavior_velocity_map_crosswalk_default.json)
    add_test(NAME behavior_velocity_map_walkway_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/behavior_velocity_map_reference_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_walkway_synthetic_inner.bin
        walkway
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/walkway_reference.json)
    add_test(NAME behavior_velocity_map_walkway_stopline_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/behavior_velocity_map_reference_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_walkway_stopline_synthetic_inner.bin
        walkway
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/walkway_stopline_reference.json)
    add_test(NAME behavior_velocity_map_signal_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/behavior_velocity_map_signal_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_signal_synthetic_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/behavior_velocity_map_crosswalk_default.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_signal_green.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_signal_red.json)
    add_test(NAME behavior_velocity_map_occlusion_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/behavior_velocity_map_occlusion_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/behavior_velocity_map_crosswalk_default.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_occlusion_rtc_clear.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_occlusion_rtc_unknown.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_occlusion_patch.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_occlusion_masked.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_occlusion_slow_object.json)
    add_test(NAME behavior_velocity_map_stuck_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/behavior_velocity_map_stuck_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/behavior_velocity_map_crosswalk_default.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_stuck_car.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_stuck_car_enabled.json
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_stuck_car_fast.json)
    add_test(NAME behavior_velocity_map_sequence_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/behavior_velocity_map_sequence_replay.py
        $<TARGET_FILE:behavior_velocity_map_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/kcity_crosswalk_bag_map_inner.bin
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_sequence_reference_state_transition_50.json)
  endif()
  set(PLANNING_KCITY_MAP
    "/home/sujin/Documents/01_GIT/01_OSS/251231_KCity/KCity_map_251231/lanelet2_map.osm"
    CACHE FILEPATH "K-City Lanelet2 지도 경로")
  set(PLANNING_KCITY_ROTARY_BAG
    "/home/sujin/Documents/01_GIT/01_OSS/251231_KCity/hpc/rosbag_rotary/rosbag_20251231_022924_0.db3"
    CACHE FILEPATH "K-City 회전교차로 rosbag 경로")
  set(PLANNING_KCITY_BLOCKING_BAG
    "/home/sujin/Documents/01_GIT/01_OSS/251231_KCity/hpc/rosbag_blocking/rosbag_20251231_023908_0.db3"
    CACHE FILEPATH "K-City 차단 rosbag 경로")
  set(PLANNING_KCITY_RUNOUT_BAG
    "/home/sujin/Documents/01_GIT/01_OSS/251231_KCity/hpc/rosbag_runout/rosbag_20251231_013654_0.db3"
    CACHE FILEPATH "K-City 돌발진입 rosbag 경로")
  set(PLANNING_KCITY_IPC_STOPLINE_BAG
    "/home/sujin/Documents/01_GIT/01_OSS/251231_KCity/ipc/rosbag_stopline/rosbag_20251231_132036_0.db3"
    CACHE FILEPATH "K-City IPC 정지선 rosbag 경로")
  if(NOT EXISTS "${PLANNING_KCITY_IPC_STOPLINE_BAG}")
    if(EXISTS "/home/sujin/Documents/01_GIT/01_OSS/251231_KCity_ori/ipc/rosbag_stopline/rosbag_20251231_132036_0.db3")
      set(PLANNING_KCITY_IPC_STOPLINE_BAG
        "/home/sujin/Documents/01_GIT/01_OSS/251231_KCity_ori/ipc/rosbag_stopline/rosbag_20251231_132036_0.db3"
        CACHE FILEPATH "K-City IPC 정지선 rosbag 경로" FORCE)
    elseif(EXISTS "/home/sujin/Documents/01_GIT/01_OSS/251231_KCity/ipc/rosbag_stopline/rosbag_20251231_132036_0.db3")
      set(PLANNING_KCITY_IPC_STOPLINE_BAG
        "/home/sujin/Documents/01_GIT/01_OSS/251231_KCity/ipc/rosbag_stopline/rosbag_20251231_132036_0.db3"
        CACHE FILEPATH "K-City IPC 정지선 rosbag 경로" FORCE)
    endif()
  endif()
  set(PLANNING_KCITY_SCURVE_BAG
    "/home/sujin/Documents/01_GIT/01_OSS/251231_KCity/hpc/rosbag_scurve/rosbag_20251231_022311_0.db3"
    CACHE FILEPATH "K-City S자 rosbag 경로")
  get_filename_component(PLANNING_KCITY_AVAILABLE_BAG_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../251231_KCity_ori" ABSOLUTE)
  foreach(_kcity_bag IN ITEMS
      ROTARY:rosbag_rotary/rosbag_20251231_022924_0.db3
      BLOCKING:rosbag_blocking/rosbag_20251231_023908_0.db3
      RUNOUT:rosbag_runout/rosbag_20251231_013654_0.db3
      SCURVE:rosbag_scurve/rosbag_20251231_022311_0.db3)
    string(REPLACE ":" ";" _kcity_parts "${_kcity_bag}")
    list(GET _kcity_parts 0 _kcity_name)
    list(GET _kcity_parts 1 _kcity_relative_path)
    set(_kcity_variable "PLANNING_KCITY_${_kcity_name}_BAG")
    if(NOT EXISTS "${${_kcity_variable}}" AND
        EXISTS "${PLANNING_KCITY_AVAILABLE_BAG_ROOT}/hpc/${_kcity_relative_path}")
      set(${_kcity_variable}
        "${PLANNING_KCITY_AVAILABLE_BAG_ROOT}/hpc/${_kcity_relative_path}"
        CACHE FILEPATH "K-City rosbag 경로" FORCE)
    endif()
  endforeach()
  if(PLANNING_BUILD_LANELET_BIN_ADAPTER AND
      EXISTS "${PLANNING_KCITY_SCURVE_BAG}")
    add_test(NAME planning_evaluator_scurve_lane_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/planning_evaluator_lane_reference_replay.py
        $<TARGET_FILE:planning_evaluator_lane_reference_runner>
        ${PLANNING_KCITY_SCURVE_BAG}
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/planning_evaluator_current_image_lane_graph_reference.json)
    add_test(NAME planning_evaluator_scurve_lane_left_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/planning_evaluator_lane_reference_replay.py
        $<TARGET_FILE:planning_evaluator_lane_reference_runner>
        ${PLANNING_KCITY_SCURVE_BAG}
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/planning_evaluator_current_image_lane_xminus3_reference.json)
    add_test(NAME planning_evaluator_scurve_lane_right_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/planning_evaluator_lane_reference_replay.py
        $<TARGET_FILE:planning_evaluator_lane_reference_runner>
        ${PLANNING_KCITY_SCURVE_BAG}
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/planning_evaluator_current_image_lane_xplus3_reference.json)
    add_test(NAME costmap_generator_scurve_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/costmap_generator_reference_replay.py
        $<TARGET_FILE:costmap_generator_reference_runner>
        ${PLANNING_KCITY_SCURVE_BAG}
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/costmap_generator_reference_scurve_synthetic.json.gz)
    add_test(NAME costmap_generator_scurve_object_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/costmap_generator_reference_replay.py
        $<TARGET_FILE:costmap_generator_reference_runner>
        ${PLANNING_KCITY_SCURVE_BAG}
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/costmap_generator_reference_scurve_object_on_road.json.gz)
    add_test(NAME costmap_generator_scurve_box_probability025_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/costmap_generator_reference_replay.py
        $<TARGET_FILE:costmap_generator_reference_runner>
        ${PLANNING_KCITY_SCURVE_BAG}
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/costmap_generator_reference_scurve_box_probability025.json.gz)
    foreach(probability_case IN ITEMS 0 1)
      add_test(NAME costmap_generator_scurve_box_probability${probability_case}_reference_replay
        COMMAND Python3::Interpreter
          ${CMAKE_CURRENT_SOURCE_DIR}/tests/costmap_generator_reference_replay.py
          $<TARGET_FILE:costmap_generator_reference_runner>
          ${PLANNING_KCITY_SCURVE_BAG}
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/costmap_generator_reference_scurve_box_probability${probability_case}.json.gz)
    endforeach()
    add_test(NAME costmap_generator_scurve_cylinder_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/costmap_generator_reference_replay.py
        $<TARGET_FILE:costmap_generator_reference_runner>
        ${PLANNING_KCITY_SCURVE_BAG}
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/costmap_generator_reference_scurve_cylinder.json.gz)
    add_test(NAME costmap_generator_scurve_polygon_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/costmap_generator_reference_replay.py
        $<TARGET_FILE:costmap_generator_reference_runner>
        ${PLANNING_KCITY_SCURVE_BAG}
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/costmap_generator_reference_scurve_polygon.json.gz)
    add_test(NAME costmap_generator_scurve_polygon_offset5_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/costmap_generator_reference_replay.py
        $<TARGET_FILE:costmap_generator_reference_runner>
        ${PLANNING_KCITY_SCURVE_BAG}
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/costmap_generator_reference_scurve_polygon_offset5.json.gz)
    add_test(NAME costmap_generator_scurve_polygon_yaw30_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/costmap_generator_reference_replay.py
        $<TARGET_FILE:costmap_generator_reference_runner>
        ${PLANNING_KCITY_SCURVE_BAG}
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/costmap_generator_reference_scurve_polygon_yaw30.json.gz)
    add_test(NAME costmap_generator_scurve_polygon_asymmetric_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/costmap_generator_reference_replay.py
        $<TARGET_FILE:costmap_generator_reference_runner>
        ${PLANNING_KCITY_SCURVE_BAG}
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/costmap_generator_reference_scurve_polygon_asymmetric.json.gz)
    add_test(NAME costmap_generator_scurve_point_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/costmap_generator_reference_replay.py
        $<TARGET_FILE:costmap_generator_reference_runner>
        ${PLANNING_KCITY_SCURVE_BAG}
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/costmap_generator_reference_scurve_point.json.gz)
    add_test(NAME costmap_generator_scurve_point_z0_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/costmap_generator_reference_replay.py
        $<TARGET_FILE:costmap_generator_reference_runner>
        ${PLANNING_KCITY_SCURVE_BAG}
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/costmap_generator_reference_scurve_point_z0.json.gz)
    add_test(NAME costmap_generator_scurve_point_z2p5_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/costmap_generator_reference_replay.py
        $<TARGET_FILE:costmap_generator_reference_runner>
        ${PLANNING_KCITY_SCURVE_BAG}
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/costmap_generator_reference_scurve_point_z2p5.json.gz)
    foreach(height_case IN ITEMS zminus0p1 z2p6)
      add_test(NAME costmap_generator_scurve_point_${height_case}_reference_replay
        COMMAND Python3::Interpreter
          ${CMAKE_CURRENT_SOURCE_DIR}/tests/costmap_generator_reference_replay.py
          $<TARGET_FILE:costmap_generator_reference_runner>
          ${PLANNING_KCITY_SCURVE_BAG}
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/costmap_generator_reference_scurve_point_${height_case}.json.gz)
    endforeach()
    add_test(NAME costmap_generator_scurve_two_points_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/costmap_generator_reference_replay.py
        $<TARGET_FILE:costmap_generator_reference_runner>
        ${PLANNING_KCITY_SCURVE_BAG}
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/costmap_generator_reference_scurve_two_points.json.gz)
    add_test(NAME costmap_generator_scurve_two_points_same_cell_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/costmap_generator_reference_replay.py
        $<TARGET_FILE:costmap_generator_reference_runner>
        ${PLANNING_KCITY_SCURVE_BAG}
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/costmap_generator_reference_scurve_two_points_same_cell.json.gz)
    add_test(NAME costmap_generator_scurve_object_point_overlap_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/costmap_generator_reference_replay.py
        $<TARGET_FILE:costmap_generator_reference_runner>
        ${PLANNING_KCITY_SCURVE_BAG}
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/costmap_generator_reference_scurve_object_point_overlap.json.gz)
  endif()
  if(EXISTS "${PLANNING_KCITY_IPC_STOPLINE_BAG}")
    add_test(NAME planning_evaluator_ipc_stopline_rosbag_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/planning_evaluator_rosbag_replay.py
        $<TARGET_FILE:planning_evaluator_rosbag_runner>
        ${PLANNING_KCITY_IPC_STOPLINE_BAG}
        --maximum-basic-mismatches 2
        --maximum-extended-mismatches 3)
    if(PLANNING_BUILD_LANELET_BIN_ADAPTER)
      add_test(NAME planning_pipeline_recorded_inputs_rosbag_replay
        COMMAND Python3::Interpreter
          ${CMAKE_CURRENT_SOURCE_DIR}/tests/planning_pipeline_rosbag_replay.py
          $<TARGET_FILE:planning_pipeline_rosbag_runner>
          ${PLANNING_KCITY_IPC_STOPLINE_BAG} --frames 10000
          --with-occupancy-grid --expected-complete-frames 574)
      add_test(NAME planning_pipeline_route_inputs_rosbag_replay
        COMMAND Python3::Interpreter
          ${CMAKE_CURRENT_SOURCE_DIR}/tests/planning_pipeline_rosbag_replay.py
          $<TARGET_FILE:planning_pipeline_rosbag_runner>
          ${PLANNING_KCITY_IPC_STOPLINE_BAG} --from-route --frames 5
          --smoother-node-max-velocity 11.11111068725586
          --expected-complete-frames 5)
      add_test(NAME planning_pipeline_route_callback_clock_rosbag_replay
        COMMAND Python3::Interpreter
          ${CMAKE_CURRENT_SOURCE_DIR}/tests/planning_pipeline_rosbag_replay.py
          $<TARGET_FILE:planning_pipeline_rosbag_runner>
          ${PLANNING_KCITY_IPC_STOPLINE_BAG} --from-route
          --callback-clock recorded-behavior-receipt --frames 5
          --smoother-node-max-velocity 11.11111068725586
          --assert-route-behavior-reference
          --assert-behavior-velocity-after-warmup
          --assert-final-speed-after-warmup
          --expected-complete-frames 5)
    endif()
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_test(NAME pipeline_ipc_stopline_clock_audit
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/pipeline_rosbag_clock_audit.py
        ${PLANNING_KCITY_IPC_STOPLINE_BAG}
        --assert-ipc-stopline-reference)
  endif()
  add_executable(stop_line_core_test tests/stop_line_core_test.cpp)
  add_executable(walkway_core_test tests/walkway_core_test.cpp)
  target_link_libraries(walkway_core_test PRIVATE planning_core)
  target_compile_options(walkway_core_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME walkway_core COMMAND walkway_core_test)
  add_executable(intersection_stuck_core_test tests/intersection_stuck_core_test.cpp)
  target_link_libraries(intersection_stuck_core_test PRIVATE planning_core)
  target_compile_options(intersection_stuck_core_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME intersection_stuck_core COMMAND intersection_stuck_core_test)
  add_executable(merge_from_private_core_test tests/merge_from_private_core_test.cpp)
  target_link_libraries(merge_from_private_core_test PRIVATE planning_core)
  target_compile_options(merge_from_private_core_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME merge_from_private_core COMMAND merge_from_private_core_test)
  add_executable(intersection_occlusion_core_test
    tests/intersection_occlusion_core_test.cpp)
  target_link_libraries(intersection_occlusion_core_test PRIVATE planning_core)
  target_compile_options(intersection_occlusion_core_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME intersection_occlusion_core COMMAND intersection_occlusion_core_test)
  add_executable(intersection_collision_state_test
    tests/intersection_collision_state_test.cpp)
  target_link_libraries(intersection_collision_state_test PRIVATE planning_core)
  target_compile_options(intersection_collision_state_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME intersection_collision_state COMMAND intersection_collision_state_test)
  target_link_libraries(stop_line_core_test PRIVATE planning_core)
  target_compile_options(stop_line_core_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME stop_line_core COMMAND stop_line_core_test)

  add_executable(traffic_light_core_test tests/traffic_light_core_test.cpp)
  target_link_libraries(traffic_light_core_test PRIVATE planning_core)
  target_compile_options(traffic_light_core_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME traffic_light_core COMMAND traffic_light_core_test)

  add_executable(crosswalk_core_test tests/crosswalk_core_test.cpp)
  target_link_libraries(crosswalk_core_test PRIVATE planning_core)
  target_compile_options(crosswalk_core_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME crosswalk_core COMMAND crosswalk_core_test)

  add_executable(crosswalk_object_state_test tests/crosswalk_object_state_test.cpp)
  target_link_libraries(crosswalk_object_state_test PRIVATE planning_core)
  target_compile_options(crosswalk_object_state_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME crosswalk_object_state COMMAND crosswalk_object_state_test)

  add_executable(crosswalk_state_sequence_runner
    tests/crosswalk_state_sequence_runner.cpp)
  target_link_libraries(crosswalk_state_sequence_runner PRIVATE planning_core)
  target_compile_options(crosswalk_state_sequence_runner PRIVATE -Wall -Wextra -Wpedantic)

  add_executable(crosswalk_occlusion_test tests/crosswalk_occlusion_test.cpp)
  target_link_libraries(crosswalk_occlusion_test PRIVATE planning_core)
  target_compile_options(crosswalk_occlusion_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME crosswalk_occlusion COMMAND crosswalk_occlusion_test)

  if(TINYXML2_FOUND)
    add_executable(lanelet_osm_adapter_test tests/lanelet_osm_adapter_test.cpp)
    target_link_libraries(lanelet_osm_adapter_test PRIVATE planning_osm_adapter)
    target_compile_options(lanelet_osm_adapter_test PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME lanelet_osm_adapter
      COMMAND lanelet_osm_adapter_test
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/traffic_light_map.osm)

    add_executable(crosswalk_reference_runner tests/crosswalk_reference_runner.cpp)
    target_link_libraries(crosswalk_reference_runner PRIVATE planning_osm_adapter)
    target_compile_options(crosswalk_reference_runner PRIVATE -Wall -Wextra -Wpedantic)
    if(EXISTS "${PLANNING_KCITY_MAP}")
      add_test(NAME crosswalk_pedestrian_default_stop_reference
        COMMAND Python3::Interpreter
          ${CMAKE_CURRENT_SOURCE_DIR}/tests/crosswalk_reference_replay.py
          $<TARGET_FILE:crosswalk_reference_runner>
          ${PLANNING_KCITY_MAP}
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_pedestrian_040.json
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_pedestrian_040_empty.json
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_pedestrian_040_detailed.json
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_pedestrian_shifted_y_minus2.json)

      add_executable(crosswalk_occlusion_reference_runner
        tests/crosswalk_occlusion_reference_runner.cpp)
      target_link_libraries(crosswalk_occlusion_reference_runner PRIVATE planning_osm_adapter)
      target_compile_options(crosswalk_occlusion_reference_runner PRIVATE -Wall -Wextra -Wpedantic)
      add_test(NAME crosswalk_occlusion_reference
        COMMAND Python3::Interpreter
          ${CMAKE_CURRENT_SOURCE_DIR}/tests/crosswalk_occlusion_reference_replay.py
          $<TARGET_FILE:crosswalk_occlusion_reference_runner>
          ${PLANNING_KCITY_MAP}
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_occlusion_rtc_clear.json
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_occlusion_rtc_unknown.json)
      add_test(NAME crosswalk_occlusion_mask_reference
        COMMAND Python3::Interpreter
          ${CMAKE_CURRENT_SOURCE_DIR}/tests/crosswalk_occlusion_mask_reference_replay.py
          $<TARGET_FILE:crosswalk_occlusion_reference_runner>
          ${PLANNING_KCITY_MAP}
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_occlusion_patch.json
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_occlusion_masked.json
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_occlusion_slow_object.json
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_occlusion_rtc_clear.json)
      add_test(NAME crosswalk_signal_reference
        COMMAND Python3::Interpreter
          ${CMAKE_CURRENT_SOURCE_DIR}/tests/crosswalk_signal_reference_replay.py
          $<TARGET_FILE:crosswalk_reference_runner>
          ${PLANNING_KCITY_MAP}
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_signal_green.json
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_signal_red.json
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_occlusion_rtc_clear.json
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_pedestrian_shifted_y_minus2.json)
      add_executable(crosswalk_stuck_reference_runner
        tests/crosswalk_stuck_reference_runner.cpp)
      target_link_libraries(crosswalk_stuck_reference_runner PRIVATE planning_osm_adapter)
      target_compile_options(crosswalk_stuck_reference_runner PRIVATE -Wall -Wextra -Wpedantic)
      add_test(NAME crosswalk_stuck_reference
        COMMAND Python3::Interpreter
          ${CMAKE_CURRENT_SOURCE_DIR}/tests/crosswalk_stuck_reference_replay.py
          $<TARGET_FILE:crosswalk_stuck_reference_runner>
          ${PLANNING_KCITY_MAP}
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_stuck_car.json
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_stuck_car_enabled.json
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_stuck_car_fast.json
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_planner_reference_occlusion_rtc_clear.json)
      add_test(NAME crosswalk_sequence_reference
        COMMAND Python3::Interpreter
          ${CMAKE_CURRENT_SOURCE_DIR}/tests/crosswalk_sequence_reference_replay.py
          $<TARGET_FILE:crosswalk_reference_runner>
          $<TARGET_FILE:crosswalk_state_sequence_runner>
          ${PLANNING_KCITY_MAP}
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_sequence_reference_pedestrian_20.json)
      add_test(NAME crosswalk_state_transition_reference
        COMMAND Python3::Interpreter
          ${CMAKE_CURRENT_SOURCE_DIR}/tests/crosswalk_state_transition_reference_replay.py
          $<TARGET_FILE:crosswalk_reference_runner>
          $<TARGET_FILE:crosswalk_state_sequence_runner>
          ${PLANNING_KCITY_MAP}
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/crosswalk_sequence_reference_state_transition_50.json)
    endif()
  endif()

  add_executable(scenario_selector_test tests/scenario_selector_test.cpp)
  target_link_libraries(scenario_selector_test PRIVATE planning_core)
  target_compile_options(scenario_selector_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME scenario_selector COMMAND scenario_selector_test)

  add_executable(detection_area_state_test tests/detection_area_state_test.cpp)
  target_link_libraries(detection_area_state_test PRIVATE planning_core)
  target_compile_options(detection_area_state_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME detection_area_state COMMAND detection_area_state_test)

  add_executable(velocity_limit_selector_test tests/velocity_limit_selector_test.cpp)
  target_link_libraries(velocity_limit_selector_test PRIVATE planning_core)
  target_compile_options(velocity_limit_selector_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME velocity_limit_selector COMMAND velocity_limit_selector_test)

  add_executable(jerk_filter_test tests/jerk_filter_test.cpp)
  target_link_libraries(jerk_filter_test PRIVATE planning_core)
  target_compile_options(jerk_filter_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME jerk_filter COMMAND jerk_filter_test)
  add_executable(analytical_velocity_smoother_test
    tests/analytical_velocity_smoother_test.cpp)
  target_link_libraries(analytical_velocity_smoother_test PRIVATE planning_core)
  target_compile_options(analytical_velocity_smoother_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME analytical_velocity_smoother
    COMMAND analytical_velocity_smoother_test)

  add_executable(jerk_resample_test tests/jerk_resample_test.cpp)
  target_link_libraries(jerk_resample_test PRIVATE planning_core)
  target_compile_options(jerk_resample_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME jerk_resample COMMAND jerk_resample_test)

  add_executable(elastic_band_core_test tests/elastic_band_core_test.cpp)
  target_link_libraries(elastic_band_core_test PRIVATE planning_core)
  target_compile_options(elastic_band_core_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME elastic_band_core COMMAND elastic_band_core_test)

  add_executable(path_optimizer_model_test tests/path_optimizer_model_test.cpp)
  target_link_libraries(path_optimizer_model_test PRIVATE planning_core)
  target_compile_options(path_optimizer_model_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME path_optimizer_model COMMAND path_optimizer_model_test)

  add_executable(path_optimizer_qp_test tests/path_optimizer_qp_test.cpp)
  target_link_libraries(path_optimizer_qp_test PRIVATE planning_core)
  target_compile_options(path_optimizer_qp_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME path_optimizer_qp COMMAND path_optimizer_qp_test)

  add_executable(path_optimizer_qp_solve_test tests/path_optimizer_qp_solve_test.cpp)
  target_link_libraries(path_optimizer_qp_solve_test PRIVATE planning_core)
  target_compile_options(path_optimizer_qp_solve_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME path_optimizer_qp_solve COMMAND path_optimizer_qp_solve_test)

  add_executable(path_optimizer_solution_test tests/path_optimizer_solution_test.cpp)
  target_link_libraries(path_optimizer_solution_test PRIVATE planning_core)
  target_compile_options(path_optimizer_solution_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME path_optimizer_solution COMMAND path_optimizer_solution_test)

  add_executable(path_optimizer_postprocess_runner
    tests/path_optimizer_postprocess_runner.cpp)
  target_link_libraries(path_optimizer_postprocess_runner PRIVATE planning_core)
  target_compile_options(path_optimizer_postprocess_runner PRIVATE -Wall -Wextra -Wpedantic)

  add_executable(path_optimizer_reference_geometry_test
    tests/path_optimizer_reference_geometry_test.cpp)
  target_link_libraries(path_optimizer_reference_geometry_test PRIVATE planning_core)
  target_compile_options(path_optimizer_reference_geometry_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME path_optimizer_reference_geometry
    COMMAND path_optimizer_reference_geometry_test)

  add_executable(path_optimizer_reference_geometry_runner
    tests/path_optimizer_reference_geometry_runner.cpp)
  target_link_libraries(path_optimizer_reference_geometry_runner PRIVATE planning_core)
  target_compile_options(path_optimizer_reference_geometry_runner PRIVATE
    -Wall -Wextra -Wpedantic)

  add_executable(path_optimizer_bounds_test tests/path_optimizer_bounds_test.cpp)
  target_link_libraries(path_optimizer_bounds_test PRIVATE planning_core)
  target_compile_options(path_optimizer_bounds_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME path_optimizer_bounds COMMAND path_optimizer_bounds_test)

  add_executable(path_optimizer_reference_data_test
    tests/path_optimizer_reference_data_test.cpp)
  target_link_libraries(path_optimizer_reference_data_test PRIVATE planning_core)
  target_compile_options(path_optimizer_reference_data_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME path_optimizer_reference_data
    COMMAND path_optimizer_reference_data_test)

  add_executable(path_optimizer_core_test tests/path_optimizer_core_test.cpp)
  target_link_libraries(path_optimizer_core_test PRIVATE planning_core)
  target_compile_options(path_optimizer_core_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME path_optimizer_core COMMAND path_optimizer_core_test)

  add_executable(path_optimizer_core_rosbag_runner
    tests/path_optimizer_core_rosbag_runner.cpp)
  target_link_libraries(path_optimizer_core_rosbag_runner PRIVATE planning_core)
  target_compile_options(path_optimizer_core_rosbag_runner PRIVATE
    -Wall -Wextra -Wpedantic)
  add_executable(eb_optimizer_chain_runner
    tests/eb_optimizer_chain_runner.cpp)
  target_link_libraries(eb_optimizer_chain_runner PRIVATE planning_core)
  target_compile_options(eb_optimizer_chain_runner PRIVATE
    -Wall -Wextra -Wpedantic)
  if(PLANNING_USE_PROXQP AND EXISTS "${PLANNING_KCITY_IPC_STOPLINE_BAG}")
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_test(NAME eb_optimizer_ipc_stopline_chain_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/eb_optimizer_chain_rosbag_replay.py
        $<TARGET_FILE:eb_optimizer_chain_runner>
        ${PLANNING_KCITY_IPC_STOPLINE_BAG}
        --frames 100 --compare-after 30 --assert-ipc-stopline-chain)
    add_test(NAME eb_optimizer_ipc_stopline_full_chain_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/eb_optimizer_chain_rosbag_replay.py
        $<TARGET_FILE:eb_optimizer_chain_runner>
        ${PLANNING_KCITY_IPC_STOPLINE_BAG}
        --frames 571 --compare-after 30 --assert-ipc-stopline-full-chain)
    add_test(NAME eb_optimizer_ipc_stopline_state_at_smooth_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/eb_optimizer_chain_rosbag_replay.py
        $<TARGET_FILE:eb_optimizer_chain_runner>
        ${PLANNING_KCITY_IPC_STOPLINE_BAG}
        --frames 100 --compare-after 30 --state-at-smooth
        --assert-ipc-stopline-state-at-smooth)
    add_test(NAME eb_optimizer_ipc_stopline_state_at_smooth_full_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/eb_optimizer_chain_rosbag_replay.py
        $<TARGET_FILE:eb_optimizer_chain_runner>
        ${PLANNING_KCITY_IPC_STOPLINE_BAG}
        --frames 571 --compare-after 30 --state-at-smooth
        --assert-ipc-stopline-state-at-smooth-full)
  endif()
  if(EXISTS "${PLANNING_KCITY_IPC_STOPLINE_BAG}")
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_test(NAME path_optimizer_ipc_stopline_continuous_rosbag_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/path_optimizer_core_rosbag_replay.py
        $<TARGET_FILE:path_optimizer_core_rosbag_runner>
        ${PLANNING_KCITY_IPC_STOPLINE_BAG}
        --frames 10000 --compare-after 30
        --assert-ipc-stopline-reference)
  endif()

  add_executable(motion_velocity_core_test tests/motion_velocity_core_test.cpp)
  target_link_libraries(motion_velocity_core_test PRIVATE planning_core)
  target_compile_options(motion_velocity_core_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME motion_velocity_core COMMAND motion_velocity_core_test)

  add_executable(obstacle_cruise_stop_core_test
    tests/obstacle_cruise_stop_core_test.cpp)
  target_link_libraries(obstacle_cruise_stop_core_test PRIVATE planning_core)
  target_compile_options(obstacle_cruise_stop_core_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME obstacle_cruise_stop_core
    COMMAND obstacle_cruise_stop_core_test)

  add_executable(obstacle_cruise_slowdown_core_test
    tests/obstacle_cruise_slowdown_core_test.cpp)
  target_link_libraries(obstacle_cruise_slowdown_core_test PRIVATE planning_core)
  target_compile_options(obstacle_cruise_slowdown_core_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME obstacle_cruise_slowdown_core
    COMMAND obstacle_cruise_slowdown_core_test)
  add_executable(obstacle_cruise_pid_core_test
    tests/obstacle_cruise_pid_core_test.cpp)
  target_link_libraries(obstacle_cruise_pid_core_test PRIVATE planning_core)
  target_compile_options(obstacle_cruise_pid_core_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME obstacle_cruise_pid_core
    COMMAND obstacle_cruise_pid_core_test)
  add_executable(obstacle_cruise_candidate_test
    tests/obstacle_cruise_candidate_test.cpp)
  target_link_libraries(obstacle_cruise_candidate_test PRIVATE planning_core)
  target_compile_options(obstacle_cruise_candidate_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME obstacle_cruise_candidate
    COMMAND obstacle_cruise_candidate_test)
  add_executable(obstacle_cruise_collision_test
    tests/obstacle_cruise_collision_test.cpp)
  target_link_libraries(obstacle_cruise_collision_test PRIVATE planning_core)
  target_compile_options(obstacle_cruise_collision_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME obstacle_cruise_collision
    COMMAND obstacle_cruise_collision_test)
  add_executable(obstacle_cruise_optimization_test
    tests/obstacle_cruise_optimization_test.cpp)
  target_link_libraries(obstacle_cruise_optimization_test PRIVATE planning_core)
  target_compile_options(obstacle_cruise_optimization_test PRIVATE
    -Wall -Wextra -Wpedantic)
  if(PLANNING_HAS_PROXQP)
    target_compile_definitions(obstacle_cruise_optimization_test PRIVATE
      PLANNING_HAS_PROXQP=1)
  endif()
  add_test(NAME obstacle_cruise_optimization
    COMMAND obstacle_cruise_optimization_test)
  add_executable(obstacle_cruise_optimization_rosbag_runner
    tests/obstacle_cruise_optimization_rosbag_runner.cpp)
  target_link_libraries(obstacle_cruise_optimization_rosbag_runner PRIVATE
    planning_core)
  target_compile_options(obstacle_cruise_optimization_rosbag_runner PRIVATE
    -Wall -Wextra -Wpedantic)
  add_executable(obstacle_cruise_pointcloud_test
    tests/obstacle_cruise_pointcloud_test.cpp)
  target_link_libraries(obstacle_cruise_pointcloud_test PRIVATE planning_core)
  target_compile_options(obstacle_cruise_pointcloud_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME obstacle_cruise_pointcloud
    COMMAND obstacle_cruise_pointcloud_test)
  add_executable(obstacle_cruise_projection_rosbag_runner
    tests/obstacle_cruise_projection_rosbag_runner.cpp)
  target_link_libraries(obstacle_cruise_projection_rosbag_runner PRIVATE planning_core)
  target_compile_options(obstacle_cruise_projection_rosbag_runner PRIVATE
    -Wall -Wextra -Wpedantic)
  add_executable(obstacle_cruise_motion_input_audit_runner
    tests/obstacle_cruise_motion_input_audit_runner.cpp)
  target_link_libraries(obstacle_cruise_motion_input_audit_runner PRIVATE planning_core)
  target_compile_options(obstacle_cruise_motion_input_audit_runner PRIVATE
    -Wall -Wextra -Wpedantic)
  add_executable(obstacle_cruise_stop_candidate_rosbag_runner
    tests/obstacle_cruise_stop_candidate_rosbag_runner.cpp)
  target_link_libraries(obstacle_cruise_stop_candidate_rosbag_runner PRIVATE planning_core)
  target_compile_options(obstacle_cruise_stop_candidate_rosbag_runner PRIVATE
    -Wall -Wextra -Wpedantic)
  add_executable(dynamic_obstacle_stop_rosbag_runner
    tests/dynamic_obstacle_stop_rosbag_runner.cpp)
  target_link_libraries(dynamic_obstacle_stop_rosbag_runner PRIVATE planning_core)
  target_compile_options(dynamic_obstacle_stop_rosbag_runner PRIVATE
    -Wall -Wextra -Wpedantic)

  add_executable(dynamic_obstacle_stop_core_test
    tests/dynamic_obstacle_stop_core_test.cpp)
  target_link_libraries(dynamic_obstacle_stop_core_test PRIVATE planning_core)
  target_compile_options(dynamic_obstacle_stop_core_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME dynamic_obstacle_stop_core
    COMMAND dynamic_obstacle_stop_core_test)

  add_executable(obstacle_velocity_limiter_core_test
    tests/obstacle_velocity_limiter_core_test.cpp)
  target_link_libraries(obstacle_velocity_limiter_core_test PRIVATE planning_core)
  target_compile_options(obstacle_velocity_limiter_core_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME obstacle_velocity_limiter_core
    COMMAND obstacle_velocity_limiter_core_test)

  add_executable(out_of_lane_core_test tests/out_of_lane_core_test.cpp)
  target_link_libraries(out_of_lane_core_test PRIVATE planning_core)
  target_compile_options(out_of_lane_core_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME out_of_lane_core COMMAND out_of_lane_core_test)

  add_executable(obstacle_cruise_stop_rosbag_runner
    tests/obstacle_cruise_stop_rosbag_runner.cpp)
  target_link_libraries(obstacle_cruise_stop_rosbag_runner PRIVATE planning_core)
  target_compile_options(obstacle_cruise_stop_rosbag_runner PRIVATE
    -Wall -Wextra -Wpedantic)

  add_executable(obstacle_cruise_pid_rosbag_runner
    tests/obstacle_cruise_pid_rosbag_runner.cpp)
  target_link_libraries(obstacle_cruise_pid_rosbag_runner PRIVATE planning_core)
  target_compile_options(obstacle_cruise_pid_rosbag_runner PRIVATE
    -Wall -Wextra -Wpedantic)
  add_executable(obstacle_cruise_slowdown_rosbag_runner
    tests/obstacle_cruise_slowdown_rosbag_runner.cpp)
  target_link_libraries(obstacle_cruise_slowdown_rosbag_runner PRIVATE
    planning_core)
  target_compile_options(obstacle_cruise_slowdown_rosbag_runner PRIVATE
    -Wall -Wextra -Wpedantic)
  if(EXISTS "${PLANNING_KCITY_SCURVE_BAG}")
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_test(NAME obstacle_cruise_slowdown_scurve_rosbag_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/obstacle_cruise_slowdown_rosbag_replay.py
        $<TARGET_FILE:obstacle_cruise_slowdown_rosbag_runner>
        ${PLANNING_KCITY_SCURVE_BAG}
        --assert-hpc-scurve-reference)
  endif()

  add_executable(elastic_band_rosbag_runner tests/elastic_band_rosbag_runner.cpp)
  target_link_libraries(elastic_band_rosbag_runner PRIVATE planning_core)
  target_compile_options(elastic_band_rosbag_runner PRIVATE -Wall -Wextra -Wpedantic)

  add_executable(elastic_band_sequence_runner tests/elastic_band_sequence_runner.cpp)
  target_link_libraries(elastic_band_sequence_runner PRIVATE planning_core)
  target_compile_options(elastic_band_sequence_runner PRIVATE -Wall -Wextra -Wpedantic)

  add_executable(trajectory_gate_test tests/trajectory_gate_test.cpp)
  target_link_libraries(trajectory_gate_test PRIVATE planning_core)
  target_compile_options(trajectory_gate_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME trajectory_gate COMMAND trajectory_gate_test)

  add_executable(trajectory_precheck_test tests/trajectory_precheck_test.cpp)
  target_link_libraries(trajectory_precheck_test PRIVATE planning_core)
  target_compile_options(trajectory_precheck_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME trajectory_precheck COMMAND trajectory_precheck_test)

  add_executable(trajectory_ego_checks_test tests/trajectory_ego_checks_test.cpp)
  target_link_libraries(trajectory_ego_checks_test PRIVATE planning_core)
  target_compile_options(trajectory_ego_checks_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME trajectory_ego_checks COMMAND trajectory_ego_checks_test)

  add_executable(planning_validator_core_test tests/planning_validator_core_test.cpp)
  target_link_libraries(planning_validator_core_test PRIVATE planning_core)
  target_compile_options(planning_validator_core_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME planning_validator_core COMMAND planning_validator_core_test)
  add_executable(planning_pipeline_core_test
    tests/planning_pipeline_core_test.cpp)
  add_executable(occlusion_spot_core_test
    tests/occlusion_spot_core_test.cpp)
  target_link_libraries(occlusion_spot_core_test PRIVATE planning_core)
  add_test(NAME occlusion_spot_core COMMAND occlusion_spot_core_test)
  target_link_libraries(planning_pipeline_core_test PRIVATE planning_core)
  target_compile_options(planning_pipeline_core_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME planning_pipeline_core
    COMMAND planning_pipeline_core_test)
  add_executable(planning_pipeline_cruise_bridge_test
    tests/planning_pipeline_cruise_bridge_test.cpp)
  target_link_libraries(planning_pipeline_cruise_bridge_test PRIVATE
    planning_core)
  target_compile_options(planning_pipeline_cruise_bridge_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME planning_pipeline_cruise_bridge
    COMMAND planning_pipeline_cruise_bridge_test)
  add_executable(planning_validator_diagnostics_test
    tests/planning_validator_diagnostics_test.cpp)
  target_link_libraries(planning_validator_diagnostics_test PRIVATE planning_core)
  target_compile_options(planning_validator_diagnostics_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME planning_validator_diagnostics
    COMMAND planning_validator_diagnostics_test)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_executable(planning_validator_diagnostics_reference_runner
    tests/planning_validator_diagnostics_reference_runner.cpp)
  target_link_libraries(planning_validator_diagnostics_reference_runner PRIVATE planning_core)
  target_compile_options(planning_validator_diagnostics_reference_runner PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME planning_validator_diagnostics_reference_replay
    COMMAND Python3::Interpreter
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/planning_validator_diagnostics_reference_replay.py
      $<TARGET_FILE:planning_validator_diagnostics_reference_runner>
      ${CMAKE_CURRENT_SOURCE_DIR}/reference/validator_diagnostics_reference.json)

  add_executable(validator_reference_runner tests/validator_reference_runner.cpp)
  target_link_libraries(validator_reference_runner PRIVATE planning_core)
  target_compile_options(validator_reference_runner PRIVATE -Wall -Wextra -Wpedantic)
  foreach(case_name synthetic stopline pedestrian cutinout)
    add_test(NAME validator_reference_${case_name}
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/validator_reference_replay.py
        $<TARGET_FILE:validator_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/validator_reference_${case_name}.json)
  endforeach()
  add_test(NAME validator_reference_stopline_sequence20
    COMMAND Python3::Interpreter
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/validator_reference_replay.py
      $<TARGET_FILE:validator_reference_runner>
      ${CMAKE_CURRENT_SOURCE_DIR}/reference/validator_reference_stopline_sequence20.json.gz)
  add_test(NAME validator_reference_stopline_recovery20
    COMMAND Python3::Interpreter
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/validator_reference_replay.py
      $<TARGET_FILE:validator_reference_runner>
      ${CMAKE_CURRENT_SOURCE_DIR}/reference/validator_reference_stopline_recovery20.json.gz)
  add_test(NAME validator_reference_pedestrian_sequence50
    COMMAND Python3::Interpreter
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/validator_reference_replay.py
      $<TARGET_FILE:validator_reference_runner>
      ${CMAKE_CURRENT_SOURCE_DIR}/reference/validator_reference_pedestrian_sequence50.json.gz)
  add_test(NAME validator_reference_pedestrian_late_sequence100
    COMMAND Python3::Interpreter
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/validator_reference_replay.py
      $<TARGET_FILE:validator_reference_runner>
      ${CMAKE_CURRENT_SOURCE_DIR}/reference/validator_reference_pedestrian_late_sequence100.json.gz)
  add_test(NAME validator_reference_stopline_long_recovery300
    COMMAND Python3::Interpreter
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/validator_reference_replay.py
      $<TARGET_FILE:validator_reference_runner>
      ${CMAKE_CURRENT_SOURCE_DIR}/reference/validator_reference_stopline_long_recovery300.json.gz)
  add_test(NAME validator_reference_cutinout_sequence250
    COMMAND Python3::Interpreter
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/validator_reference_replay.py
      $<TARGET_FILE:validator_reference_runner>
      ${CMAKE_CURRENT_SOURCE_DIR}/reference/validator_reference_cutinout_sequence250.json.gz)

  add_executable(traffic_light_reference_runner tests/traffic_light_reference_runner.cpp)
  target_link_libraries(traffic_light_reference_runner PRIVATE planning_core)
  target_compile_options(traffic_light_reference_runner PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME traffic_light_stopline_reference_replay
    COMMAND Python3::Interpreter
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/traffic_light_reference_replay.py
      $<TARGET_FILE:traffic_light_reference_runner>
      ${CMAKE_CURRENT_SOURCE_DIR}/reference/traffic_light_planner_reference_stopline.json)

  add_executable(stop_line_reference_runner tests/stop_line_reference_runner.cpp)
  target_link_libraries(stop_line_reference_runner PRIVATE planning_core)
  target_compile_options(stop_line_reference_runner PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME stop_line_synthetic_reference_replay
    COMMAND Python3::Interpreter
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/stop_line_reference_replay.py
      $<TARGET_FILE:stop_line_reference_runner>
      ${CMAKE_CURRENT_SOURCE_DIR}/reference/stop_line_planner_reference_synthetic_stop_sign.json)
  add_test(NAME stop_line_synthetic_near_reference_replay
    COMMAND Python3::Interpreter
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/stop_line_reference_replay.py
      $<TARGET_FILE:stop_line_reference_runner>
      ${CMAKE_CURRENT_SOURCE_DIR}/reference/stop_line_planner_reference_synthetic_stop_sign_near.json)

  add_executable(stop_line_state_reference_runner
    tests/stop_line_state_reference_runner.cpp)
  target_link_libraries(stop_line_state_reference_runner PRIVATE planning_core)
  target_compile_options(stop_line_state_reference_runner PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME stop_line_state_reference_replay
    COMMAND Python3::Interpreter
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/stop_line_state_reference_replay.py
      $<TARGET_FILE:stop_line_state_reference_runner>
      ${CMAKE_CURRENT_SOURCE_DIR}/reference/stop_line_state_reference_synthetic_7.json)

  add_executable(jerk_qp_problem_test tests/jerk_qp_problem_test.cpp)
  target_link_libraries(jerk_qp_problem_test PRIVATE planning_core)
  target_compile_options(jerk_qp_problem_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME jerk_qp_problem COMMAND jerk_qp_problem_test)

  add_executable(jerk_qp_solver_test tests/jerk_qp_solver_test.cpp)
  target_link_libraries(jerk_qp_solver_test PRIVATE planning_core)
  if(PLANNING_HAS_PROXQP)
    target_compile_definitions(jerk_qp_solver_test PRIVATE PLANNING_HAS_PROXQP=1)
  endif()
  target_compile_options(jerk_qp_solver_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME jerk_qp_solver COMMAND jerk_qp_solver_test)

  add_executable(jerk_smoother_test tests/jerk_smoother_test.cpp)
  target_link_libraries(jerk_smoother_test PRIVATE planning_core)
  target_compile_options(jerk_smoother_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME jerk_smoother COMMAND jerk_smoother_test)
  add_executable(jerk_smoother_benchmark tests/jerk_smoother_benchmark.cpp)
  target_link_libraries(jerk_smoother_benchmark PRIVATE planning_core)
  target_compile_options(jerk_smoother_benchmark PRIVATE -Wall -Wextra -Wpedantic)
  add_executable(jerk_external_limit_plan_runner
    tests/jerk_external_limit_plan_runner.cpp)
  target_link_libraries(jerk_external_limit_plan_runner PRIVATE planning_core)
  target_compile_options(jerk_external_limit_plan_runner PRIVATE -Wall -Wextra -Wpedantic)
  if(NOT PLANNING_HAS_PROXQP)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_test(NAME admm_rosbag_guard
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/admm_rosbag_guard.py
        $<TARGET_FILE:jerk_smoother_benchmark>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference)
  endif()

  add_executable(jerk_node_policy_test tests/jerk_node_policy_test.cpp)
  target_link_libraries(jerk_node_policy_test PRIVATE planning_core)
  target_compile_options(jerk_node_policy_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME jerk_node_policy COMMAND jerk_node_policy_test)

  add_executable(run_out_state_test tests/run_out_state_test.cpp)
  target_link_libraries(run_out_state_test PRIVATE planning_core)
  target_compile_options(run_out_state_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME run_out_state COMMAND run_out_state_test)

  add_executable(run_out_collision_test tests/run_out_collision_test.cpp)
  target_link_libraries(run_out_collision_test PRIVATE planning_core)
  target_compile_options(run_out_collision_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME run_out_collision COMMAND run_out_collision_test)

  add_executable(pointcloud2_adapter_test tests/pointcloud2_adapter_test.cpp)
  target_link_libraries(pointcloud2_adapter_test PRIVATE planning_core)
  target_compile_options(pointcloud2_adapter_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME pointcloud2_adapter COMMAND pointcloud2_adapter_test)

  add_executable(pointcloud2_rosbag_runner
    tests/pointcloud2_rosbag_runner.cpp)
  target_link_libraries(pointcloud2_rosbag_runner PRIVATE planning_core)
  target_compile_options(pointcloud2_rosbag_runner PRIVATE
    -Wall -Wextra -Wpedantic)

  if(TARGET planning_lanelet_bin_adapter)
    add_executable(run_out_collision_rosbag_runner
      tests/run_out_collision_rosbag_runner.cpp)
    target_link_libraries(run_out_collision_rosbag_runner PRIVATE
      planning_core planning_lanelet_bin_adapter)
    target_compile_options(run_out_collision_rosbag_runner PRIVATE
      -Wall -Wextra -Wpedantic)
    if(EXISTS "${PLANNING_KCITY_RUNOUT_BAG}")
      find_package(Python3 COMPONENTS Interpreter REQUIRED)
      add_test(NAME run_out_analytical_collision_rosbag_replay
        COMMAND Python3::Interpreter
          ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_out_collision_rosbag_replay.py
          $<TARGET_FILE:run_out_collision_rosbag_runner>
          ${PLANNING_KCITY_RUNOUT_BAG}
          --analytical-full --assert-hpc-runout-reference)
    endif()
  endif()

  add_executable(virtual_traffic_light_core_test
    tests/virtual_traffic_light_core_test.cpp)
  target_link_libraries(virtual_traffic_light_core_test PRIVATE planning_core)
  target_compile_options(virtual_traffic_light_core_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME virtual_traffic_light_core COMMAND virtual_traffic_light_core_test)
  add_executable(blind_spot_core_test tests/blind_spot_core_test.cpp)
  target_link_libraries(blind_spot_core_test PRIVATE planning_core)
  target_compile_options(blind_spot_core_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME blind_spot_core COMMAND blind_spot_core_test)

  add_executable(behavior_path_shift_test tests/behavior_path_shift_test.cpp)
  target_link_libraries(behavior_path_shift_test PRIVATE planning_core)
  target_compile_options(behavior_path_shift_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME behavior_path_shift COMMAND behavior_path_shift_test)
  add_executable(behavior_lane_change_route_test
    tests/behavior_lane_change_route_test.cpp)
  target_link_libraries(behavior_lane_change_route_test PRIVATE planning_core)
  target_compile_options(behavior_lane_change_route_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME behavior_lane_change_route
    COMMAND behavior_lane_change_route_test)
  add_executable(behavior_path_lane_safety_test
    tests/behavior_path_lane_safety_test.cpp)
  target_link_libraries(behavior_path_lane_safety_test PRIVATE planning_core)
  target_compile_options(behavior_path_lane_safety_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME behavior_path_lane_safety
    COMMAND behavior_path_lane_safety_test)
  if(TARGET behavior_start_centerline_probe AND
     EXISTS "${PLANNING_KCITY_ROTARY_BAG}")
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_test(NAME behavior_start_candidate_map_rosbag_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/behavior_start_candidate_map_replay.py
        $<TARGET_FILE:behavior_start_centerline_probe>
        ${PLANNING_KCITY_ROTARY_BAG})
  endif()
  add_executable(behavior_path_object_safety_test
    tests/behavior_path_object_safety_test.cpp)
  target_link_libraries(behavior_path_object_safety_test PRIVATE planning_core)
  target_compile_options(behavior_path_object_safety_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME behavior_path_object_safety
    COMMAND behavior_path_object_safety_test)
  add_executable(behavior_path_prediction_test
    tests/behavior_path_prediction_test.cpp)
  target_link_libraries(behavior_path_prediction_test PRIVATE planning_core)
  target_compile_options(behavior_path_prediction_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME behavior_path_prediction
    COMMAND behavior_path_prediction_test)
  add_executable(behavior_lane_change_safety_test
    tests/behavior_lane_change_safety_test.cpp)
  target_link_libraries(behavior_lane_change_safety_test PRIVATE planning_core)
  target_compile_options(behavior_lane_change_safety_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME behavior_lane_change_safety
    COMMAND behavior_lane_change_safety_test)
  add_executable(behavior_lane_change_rtc_rosbag_runner
    tests/behavior_lane_change_rtc_rosbag_runner.cpp)
  target_link_libraries(behavior_lane_change_rtc_rosbag_runner PRIVATE
    planning_core)
  add_executable(behavior_candidate_callback_test
    tests/behavior_candidate_callback_test.cpp)
  target_link_libraries(behavior_candidate_callback_test PRIVATE planning_core)
  add_test(NAME behavior_candidate_callback
    COMMAND behavior_candidate_callback_test)
  add_executable(behavior_velocity_perception_bridge_test
    tests/behavior_velocity_perception_bridge_test.cpp)
  target_link_libraries(behavior_velocity_perception_bridge_test PRIVATE
    planning_core)
  add_test(NAME behavior_velocity_perception_bridge
    COMMAND behavior_velocity_perception_bridge_test)
  add_executable(behavior_path_object_safety_rosbag_runner
    tests/behavior_path_object_safety_rosbag_runner.cpp)
  target_link_libraries(behavior_path_object_safety_rosbag_runner
    PRIVATE planning_core)
  target_compile_options(behavior_path_object_safety_rosbag_runner PRIVATE
    -Wall -Wextra -Wpedantic)
  if(EXISTS "${PLANNING_KCITY_BLOCKING_BAG}")
    add_test(NAME behavior_lane_change_rtc_rosbag_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/behavior_lane_change_rtc_rosbag_replay.py
        $<TARGET_FILE:behavior_lane_change_rtc_rosbag_runner>
        ${PLANNING_KCITY_BLOCKING_BAG})
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_test(NAME behavior_path_object_rejection_rosbag_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/behavior_path_object_safety_rosbag_replay.py
        $<TARGET_FILE:behavior_path_object_safety_rosbag_runner>
        ${PLANNING_KCITY_BLOCKING_BAG}
        --assert-rejection)
  endif()
  add_executable(behavior_static_avoidance_test
    tests/behavior_static_avoidance_test.cpp)
  add_executable(behavior_route_action_test
    tests/behavior_route_action_test.cpp)
  add_executable(behavior_frenet_sampling_test
    tests/behavior_frenet_sampling_test.cpp)
  add_executable(behavior_dynamic_avoidance_test
    tests/behavior_dynamic_avoidance_test.cpp)
  target_link_libraries(behavior_dynamic_avoidance_test PRIVATE planning_core)
  target_compile_options(behavior_dynamic_avoidance_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME behavior_dynamic_avoidance
    COMMAND behavior_dynamic_avoidance_test)
  target_link_libraries(behavior_frenet_sampling_test PRIVATE planning_core)
  target_compile_options(behavior_frenet_sampling_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME behavior_frenet_sampling
    COMMAND behavior_frenet_sampling_test)
  target_link_libraries(behavior_route_action_test PRIVATE planning_core)
  target_compile_options(behavior_route_action_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME behavior_route_action COMMAND behavior_route_action_test)
  target_link_libraries(behavior_static_avoidance_test PRIVATE planning_core)
  target_compile_options(behavior_static_avoidance_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME behavior_static_avoidance
    COMMAND behavior_static_avoidance_test)
  add_executable(behavior_goal_refinement_test
    tests/behavior_goal_refinement_test.cpp)
  target_link_libraries(behavior_goal_refinement_test PRIVATE planning_core)
  target_compile_options(behavior_goal_refinement_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME behavior_goal_refinement
    COMMAND behavior_goal_refinement_test)
  add_executable(behavior_goal_safety_test
    tests/behavior_goal_safety_test.cpp)
  target_link_libraries(behavior_goal_safety_test PRIVATE planning_core)
  target_compile_options(behavior_goal_safety_test PRIVATE
    -Wall -Wextra -Wpedantic)
  add_test(NAME behavior_goal_safety COMMAND behavior_goal_safety_test)

  if(PLANNING_HAS_PROXQP)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_executable(smoother_reference_runner tests/smoother_reference_runner.cpp)
    target_link_libraries(smoother_reference_runner PRIVATE planning_core)
    target_compile_options(smoother_reference_runner PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME smoother_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/smoother_reference_replay.py
        $<TARGET_FILE:smoother_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/smoother_reference.json)
    add_test(NAME smoother_stopline_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/smoother_reference_replay.py
        $<TARGET_FILE:smoother_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/smoother_reference_stopline.json)
    add_test(NAME smoother_pedestrian_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/smoother_reference_replay.py
        $<TARGET_FILE:smoother_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/smoother_reference_pedestrian.json)
    add_test(NAME smoother_cutinout_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/smoother_reference_replay.py
        $<TARGET_FILE:smoother_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/smoother_reference_cutinout.json)
    add_test(NAME smoother_curve_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/smoother_reference_replay.py
        $<TARGET_FILE:smoother_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/smoother_reference_curve.json)
    add_test(NAME smoother_reverse_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/smoother_reference_replay.py
        $<TARGET_FILE:smoother_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/smoother_reference_reverse.json)
    add_test(NAME smoother_sequence_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/smoother_reference_replay.py
        $<TARGET_FILE:smoother_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/smoother_reference_sequence.json)
    add_test(NAME smoother_high_speed_sequence4_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/smoother_sequence_reference_replay.py
        $<TARGET_FILE:smoother_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/smoother_reference_sequence4.json.gz)
    add_test(NAME smoother_high_speed_sequence10_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/smoother_sequence_reference_replay.py
        $<TARGET_FILE:smoother_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/smoother_reference_sequence10_single.json.gz)
    add_test(NAME smoother_curve_sequence_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/smoother_reference_replay.py
        $<TARGET_FILE:smoother_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/smoother_reference_curve_sequence.json)
    add_test(NAME smoother_external_limit_reference_replay
      COMMAND Python3::Interpreter
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/smoother_reference_replay.py
        $<TARGET_FILE:smoother_reference_runner>
        ${CMAKE_CURRENT_SOURCE_DIR}/reference/smoother_reference_external_limit.json)

    add_executable(smoother_preprocess_runner tests/smoother_preprocess_runner.cpp)
    target_link_libraries(smoother_preprocess_runner PRIVATE planning_core)
    target_compile_options(smoother_preprocess_runner PRIVATE -Wall -Wextra -Wpedantic)
    foreach(case_name smoother stopline pedestrian cutinout curve reverse sequence curve_sequence external_limit)
      if(case_name STREQUAL "smoother")
        set(reference_file "smoother_reference.json")
      else()
        set(reference_file "smoother_reference_${case_name}.json")
      endif()
      add_test(NAME smoother_preprocess_${case_name}
        COMMAND Python3::Interpreter
          ${CMAKE_CURRENT_SOURCE_DIR}/tests/smoother_preprocess_replay.py
          $<TARGET_FILE:smoother_preprocess_runner>
          ${CMAKE_CURRENT_SOURCE_DIR}/reference/${reference_file})
    endforeach()
  endif()
endif()
