cmake_minimum_required(VERSION 3.16)
project(planning_core_standalone_example LANGUAGES CXX)

find_package(planning_core CONFIG REQUIRED)
add_executable(planning_core_standalone_example main.cpp)
target_link_libraries(planning_core_standalone_example
  PRIVATE planning::planning_core)
