cmake_minimum_required(VERSION 3.22)
project(ExtraLib CXX)

message(STATUS "PROXSUITE_GIT_REPOSITORY: $ENV{PROXSUITE_GIT_REPOSITORY}")
message(STATUS "PROXSUITE_GIT_TAG: $ENV{PROXSUITE_GIT_TAG}")

include(FetchContent)
FetchContent_Declare(
  proxsuite-c++
  GIT_REPOSITORY $ENV{PROXSUITE_GIT_REPOSITORY}
  GIT_TAG $ENV{PROXSUITE_GIT_TAG}
  GIT_SHALLOW ON
)
FetchContent_MakeAvailable(proxsuite-c++)

add_executable(run-proxqp ${CMAKE_CURRENT_LIST_DIR}/../src/run-proxqp.cpp)
target_link_libraries(run-proxqp PUBLIC proxsuite)
