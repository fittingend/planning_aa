cmake_minimum_required(VERSION 3.16)
project(planning_map_bin_patch LANGUAGES CXX)
include(GNUInstallDirs)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ROSLESS_LANELET2_SOURCE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/Rosless-Lanelet2"
  CACHE PATH "저장소에 포함된 Rosless-Lanelet2 소스 위치")
if(NOT EXISTS "${ROSLESS_LANELET2_SOURCE_DIR}/lanelet2_io/CMakeLists.txt")
  message(FATAL_ERROR "Rosless-Lanelet2 소스를 찾을 수 없습니다: ${ROSLESS_LANELET2_SOURCE_DIR}")
endif()

list(APPEND CMAKE_MODULE_PATH "${ROSLESS_LANELET2_SOURCE_DIR}/cmake")
find_package(Boost REQUIRED COMPONENTS serialization filesystem)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PugiXml REQUIRED)

add_subdirectory("${ROSLESS_LANELET2_SOURCE_DIR}/lanelet2_core"
  "${CMAKE_CURRENT_BINARY_DIR}/lanelet2_core")
add_subdirectory("${ROSLESS_LANELET2_SOURCE_DIR}/lanelet2_io"
  "${CMAKE_CURRENT_BINARY_DIR}/lanelet2_io")

add_executable(crosswalk_signal_map_patch main.cpp)
target_compile_features(crosswalk_signal_map_patch PRIVATE cxx_std_17)
target_link_libraries(crosswalk_signal_map_patch PRIVATE
  lanelet2_io)

add_executable(stop_sign_map_patch stop_sign_main.cpp)
target_compile_features(stop_sign_map_patch PRIVATE cxx_std_17)
target_link_libraries(stop_sign_map_patch PRIVATE
  lanelet2_io)

add_executable(walkway_map_patch walkway_main.cpp)
target_compile_features(walkway_map_patch PRIVATE cxx_std_17)
target_link_libraries(walkway_map_patch PRIVATE
  lanelet2_io Boost::serialization)

add_executable(intersection_signal_map_patch intersection_signal_main.cpp)
target_compile_features(intersection_signal_map_patch PRIVATE cxx_std_17)
target_link_libraries(intersection_signal_map_patch PRIVATE
  lanelet2_io Boost::serialization)

add_executable(intersection_control_map_patch intersection_control_main.cpp)
target_compile_features(intersection_control_map_patch PRIVATE cxx_std_17)
target_link_libraries(intersection_control_map_patch PRIVATE
  lanelet2_io Boost::serialization)

add_executable(parking_lot_map_patch parking_lot_main.cpp)
target_compile_features(parking_lot_map_patch PRIVATE cxx_std_17)
target_link_libraries(parking_lot_map_patch PRIVATE
  lanelet2_io Boost::serialization)

add_executable(intersection_no_signal_map_patch intersection_no_signal_main.cpp)
target_compile_features(intersection_no_signal_map_patch PRIVATE cxx_std_17)
target_link_libraries(intersection_no_signal_map_patch PRIVATE
  lanelet2_io Boost::serialization)

add_executable(lane_scan_map_patch lane_scan_main.cpp)
target_compile_features(lane_scan_map_patch PRIVATE cxx_std_17)
target_link_libraries(lane_scan_map_patch PRIVATE
  lanelet2_io Boost::serialization)

add_executable(detection_area_map_patch detection_area_main.cpp)
target_compile_features(detection_area_map_patch PRIVATE cxx_std_17)
target_link_libraries(detection_area_map_patch PRIVATE
  lanelet2_io Boost::serialization)

add_executable(speed_bump_map_patch speed_bump_main.cpp)
target_compile_features(speed_bump_map_patch PRIVATE cxx_std_17)
target_link_libraries(speed_bump_map_patch PRIVATE
  lanelet2_io Boost::serialization)

add_executable(no_drivable_lane_map_patch no_drivable_lane_main.cpp)
target_compile_features(no_drivable_lane_map_patch PRIVATE cxx_std_17)
target_link_libraries(no_drivable_lane_map_patch PRIVATE
  lanelet2_io Boost::serialization)

add_executable(parking_space_map_patch parking_space_main.cpp)
target_compile_features(parking_space_map_patch PRIVATE cxx_std_17)
target_link_libraries(parking_space_map_patch PRIVATE
  lanelet2_io Boost::serialization)

add_executable(shoulder_map_patch shoulder_main.cpp)
target_compile_features(shoulder_map_patch PRIVATE cxx_std_17)
target_link_libraries(shoulder_map_patch PRIVATE
  lanelet2_io Boost::serialization)

add_executable(drivable_alternative_map_patch drivable_alternative_main.cpp)
target_compile_features(drivable_alternative_map_patch PRIVATE cxx_std_17)
target_link_libraries(drivable_alternative_map_patch PRIVATE
  lanelet2_io Boost::serialization)

add_executable(no_stopping_area_map_patch no_stopping_area_main.cpp)
target_compile_features(no_stopping_area_map_patch PRIVATE cxx_std_17)
target_link_libraries(no_stopping_area_map_patch PRIVATE
  lanelet2_io Boost::serialization)

add_executable(private_merge_map_patch private_merge_main.cpp)
target_compile_features(private_merge_map_patch PRIVATE cxx_std_17)
target_link_libraries(private_merge_map_patch PRIVATE
  lanelet2_io Boost::serialization)
