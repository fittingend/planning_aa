cmake_minimum_required(VERSION 3.20)
project(planning_aa_adaptive VERSION 0.1.0 LANGUAGES CXX)

if(NOT PARA_SDK AND DEFINED ENV{PARA_SDK})
  set(PARA_SDK "$ENV{PARA_SDK}")
endif()
if(NOT PARA_SDK)
  set(PARA_SDK "${CMAKE_CURRENT_SOURCE_DIR}/vendor/para-sdk/x86_64")
endif()
if(NOT EXISTS "${PARA_SDK}/share/cmake/Modules/PARA/Para.cmake")
  message(FATAL_ERROR "PARA SDK missing from adaptive/vendor/para-sdk/x86_64; set PARA_SDK to override")
endif()

list(APPEND CMAKE_MODULE_PATH "${PARA_SDK}/share/cmake/Modules")
include(PARA/Para)

set(OSS_AP_DEFAULT "${CMAKE_CURRENT_SOURCE_DIR}/vendor/oss-ap")
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../oss-ap-planning-bridge/src/Ecu_Machine/Exe_missionplanner/include/oss/srv/MP2RS/rs_si_missionplanner_skeleton.h")
  set(OSS_AP_DEFAULT "${CMAKE_CURRENT_SOURCE_DIR}/../../oss-ap-planning-bridge")
endif()
set(OSS_AP_SOURCE "${OSS_AP_DEFAULT}" CACHE PATH
    "Local copy of the generated ARA communication artifacts")
set(OSS_ECU "${OSS_AP_SOURCE}/src/Ecu_Machine")
if(NOT EXISTS "${OSS_ECU}/Exe_ServiceCreator/include/oss/srv/SCr2MP/mp_si_servicecreator_proxy.h")
  message(FATAL_ERROR "Generated ARA artifacts missing from adaptive/vendor/oss-ap; set OSS_AP_SOURCE to override")
endif()
if(NOT EXISTS "${OSS_ECU}/Exe_missionplanner/include/oss/srv/MP2RS/rs_si_missionplanner_skeleton.h")
  message(FATAL_ERROR "Service 1814 artifacts missing; set OSS_AP_SOURCE to oss-ap-planning-bridge")
endif()

# Stage the four service instances used by PlanningAA in the build tree.
set(PLANNING_GEN "${CMAKE_CURRENT_BINARY_DIR}/generated/PlanningAA")
file(REMOVE_RECURSE "${PLANNING_GEN}")
file(MAKE_DIRECTORY "${PLANNING_GEN}/include" "${PLANNING_GEN}/manifest/interface/PlanningAA/AA")
file(COPY "${OSS_ECU}/Exe_ServiceCreator/include/oss" DESTINATION "${PLANNING_GEN}/include")
foreach(port IN ITEMS RPort_SCr2MP PPort_MP2SCr PPort_MP2RS PPort_PV2SCr)
  if(port STREQUAL "PPort_PV2SCr")
    set(source "${OSS_ECU}/Exe_planningvalidator/manifest/interface/Exe_planningvalidator/AA/${port}.json")
    set(old_app "Exe_planningvalidator")
  else()
    set(source "${OSS_ECU}/Exe_missionplanner/manifest/interface/Exe_missionplanner/AA/${port}.json")
    set(old_app "Exe_missionplanner")
  endif()
  file(READ "${source}" contents)
  string(REPLACE "${old_app}/AA" "PlanningAA/AA" contents "${contents}")
  file(WRITE "${PLANNING_GEN}/manifest/interface/PlanningAA/AA/${port}.json" "${contents}")
endforeach()
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/app/manifest/exec" DESTINATION "${PLANNING_GEN}/manifest")

set(CM_GEN "${CMAKE_CURRENT_BINARY_DIR}/generated/CM")
file(REMOVE_RECURSE "${CM_GEN}")
file(MAKE_DIRECTORY "${CM_GEN}/manifest/com/someip")
file(COPY "${OSS_ECU}/CM/manifest/exec" DESTINATION "${CM_GEN}/manifest")
set(cm_manifests
  "com/CM_CmInstantiation.json"
  "com/someip/CM_R_MP_ServiceCreator.json"
  "com/someip/CM_P_SCr_missionplanner.json"
  "com/someip/CM_P_SCr_planningvalidator.json")
foreach(manifest IN LISTS cm_manifests)
  set(cm_source "${OSS_ECU}/CM/manifest/${manifest}")
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/app/manifest/${manifest}")
    set(cm_source "${CMAKE_CURRENT_SOURCE_DIR}/app/manifest/${manifest}")
  endif()
  file(READ "${cm_source}" contents)
  string(REPLACE "192.168.100.92" "192.168.100.93" contents "${contents}")
  file(WRITE "${CM_GEN}/manifest/${manifest}" "${contents}")
endforeach()
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/app/manifest/com/someip/CM_P_RS_missionplanner.json"
     DESTINATION "${CM_GEN}/manifest/com/someip")

# Install the platform manifests alongside the SDK runtime supplied at build time.
set(PARA_PLATFORM TRUE)
set(PARA_APP_NAME EM)
set(PARA_APP_VERSION 1.0.0)
set(PARA_APP_GEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/platform")
Para_App()
unset(PARA_PLATFORM)

set(PARA_PLATFORM TRUE)
set(PARA_APP_NAME CM)
set(PARA_APP_VERSION 1.0.0)
set(PARA_APP_GEN_DIR "${CM_GEN}")
Para_App()
unset(PARA_PLATFORM)

# The existing ROS-free planning library remains the calculation implementation.
set(BUILD_TESTING OFF CACHE BOOL "No core tests in the runtime image" FORCE)
set(Boost_NO_BOOST_CMAKE ON CACHE BOOL "Find the staged Boost serialization library" FORCE)
set(PLANNING_BUILD_GRID_ADAPTER OFF CACHE BOOL "No grid adapter in bootstrap" FORCE)
set(PLANNING_BUILD_OSM_ADAPTER OFF CACHE BOOL "No OSM adapter in bootstrap" FORCE)
set(PLANNING_BUILD_LANELET_BIN_ADAPTER ON CACHE BOOL "Decode LaneletMapBin for live planning" FORCE)
set(PLANNING_USE_PROXQP OFF CACHE BOOL "No ProxQP in bootstrap" FORCE)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/.." planning-core)

set(PARA_APP_NAME PlanningAA)
set(PARA_APP_VERSION 0.1.0)
set(PARA_APP_GEN_DIR "${PLANNING_GEN}")
Para_App()

add_executable(PlanningAA app/main.cpp app/trajectory_output.cpp)
target_compile_features(PlanningAA PRIVATE cxx_std_17)
target_link_libraries(PlanningAA PRIVATE planning_lanelet_bin_adapter pthread)
target_include_directories(PlanningAA PRIVATE "${OSS_ECU}/Exe_missionplanner/include")
install(FILES
  "$<TARGET_FILE:lanelet2_core>"
  "$<TARGET_FILE:lanelet2_routing>"
  "$<TARGET_FILE:lanelet2_traffic_rules>"
  DESTINATION lib COMPONENT Standalone)
Para_Target(PlanningAA PARA_LIBS_PRIVATE core log exec com
  PARA_GEN_DIR "${PARA_APP_GEN_DIR}" OUTPUT_NAME PlanningAA)
