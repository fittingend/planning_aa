cmake_minimum_required(VERSION 3.20)
project(planning_aa_adaptive VERSION 0.1.0 LANGUAGES CXX)

if(NOT PARA_SDK AND DEFINED ENV{PARA_SDK})
  set(PARA_SDK "$ENV{PARA_SDK}")
endif()
if(NOT EXISTS "${PARA_SDK}/share/cmake/Modules/PARA/Para.cmake")
  message(FATAL_ERROR "Set PARA_SDK to the PARA R20-11 x86_64 SDK directory")
endif()

list(APPEND CMAKE_MODULE_PATH "${PARA_SDK}/share/cmake/Modules")
include(PARA/Para)

# Install the platform manifests alongside the SDK runtime supplied at build time.
set(PARA_PLATFORM TRUE)
set(PARA_APP_NAME EM)
set(PARA_APP_VERSION 1.0.0)
set(PARA_APP_GEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/platform")
Para_App()
unset(PARA_PLATFORM)

# The existing ROS-free planning library remains the calculation implementation.
set(BUILD_TESTING OFF CACHE BOOL "No core tests in the runtime image" FORCE)
set(PLANNING_BUILD_GRID_ADAPTER OFF CACHE BOOL "No grid adapter in bootstrap" FORCE)
set(PLANNING_BUILD_OSM_ADAPTER OFF CACHE BOOL "No OSM adapter in bootstrap" FORCE)
set(PLANNING_BUILD_LANELET_BIN_ADAPTER OFF CACHE BOOL "No LaneletBin adapter in bootstrap" FORCE)
set(PLANNING_USE_PROXQP OFF CACHE BOOL "No ProxQP in bootstrap" FORCE)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/.." planning-core)

set(PARA_APP_NAME PlanningAA)
set(PARA_APP_VERSION 0.1.0)
set(PARA_APP_GEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/app")
Para_App()

add_executable(PlanningAA app/main.cpp)
target_compile_features(PlanningAA PRIVATE cxx_std_17)
target_link_libraries(PlanningAA PRIVATE planning_core)
Para_Target(PlanningAA PARA_LIBS_PRIVATE core log exec
  PARA_GEN_DIR "${PARA_APP_GEN_DIR}" OUTPUT_NAME PlanningAA)
