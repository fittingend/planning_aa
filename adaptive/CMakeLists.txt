cmake_minimum_required(VERSION 3.20)
project(decision_ap_adaptive VERSION 0.1.0 LANGUAGES CXX)

if(NOT PARA_SDK AND DEFINED ENV{PARA_SDK})
  set(PARA_SDK "$ENV{PARA_SDK}")
endif()
if(NOT PARA_SDK)
  set(PARA_SDK "${CMAKE_CURRENT_SOURCE_DIR}/vendor/para-sdk/${CMAKE_SYSTEM_PROCESSOR}")
endif()
if(NOT EXISTS "${PARA_SDK}/share/cmake/Modules/PARA/Para.cmake")
  message(FATAL_ERROR "PARA SDK missing from ${PARA_SDK}; set PARA_SDK to override")
endif()

list(APPEND CMAKE_MODULE_PATH "${PARA_SDK}/share/cmake/Modules")
include(PARA/Para)

set(OSS_AP_DEFAULT "${CMAKE_CURRENT_SOURCE_DIR}/vendor/oss-ap")
set(OSS_AP_SOURCE "${OSS_AP_DEFAULT}" CACHE PATH
    "Local copy of the generated ARA communication artifacts")
set(OSS_ECU "${OSS_AP_SOURCE}/src/Ecu_Machine")
if(NOT EXISTS "${OSS_ECU}/Exe_ServiceCreator/include/oss/srv/SCr2MP/mp_si_servicecreator_proxy.h")
  message(FATAL_ERROR "Generated ARA artifacts missing from adaptive/vendor/oss-ap; set OSS_AP_SOURCE to override")
endif()
if(NOT EXISTS "${OSS_ECU}/Exe_missionplanner/include/oss/srv/MP2RS/rs_si_missionplanner_skeleton.h")
  message(FATAL_ERROR "Service 1814 artifacts missing from ${OSS_AP_SOURCE}")
endif()

# Stage the four service instances used by DecisionAP in the build tree.
set(DECISION_GEN "${CMAKE_CURRENT_BINARY_DIR}/generated/DecisionAP")
file(REMOVE_RECURSE "${DECISION_GEN}")
file(MAKE_DIRECTORY "${DECISION_GEN}/include" "${DECISION_GEN}/manifest/interface/DecisionAP/AA")
file(COPY "${OSS_ECU}/Exe_ServiceCreator/include/oss" DESTINATION "${DECISION_GEN}/include")
file(COPY "${OSS_ECU}/Exe_scenarioselector/include/oss/srv/SS2VS"
     DESTINATION "${DECISION_GEN}/include/oss/srv")
foreach(family IN ITEMS AA2SCrMetrics AA2SCrRemaining AA2SCrTurn AA2SCrHazard)
  file(COPY "${OSS_ECU}/Exe_ServiceCreator/include/oss/srv/${family}"
       DESTINATION "${DECISION_GEN}/include/oss/srv")
endforeach()
foreach(port IN ITEMS RPort_SCr2MP PPort_MP2SCr PPort_MP2RS PPort_PV2SCr PPort_SS2VS PPort_AA2SCrMetrics PPort_AA2SCrRemaining PPort_AA2SCrTurn PPort_AA2SCrHazard)
  if(port MATCHES "^PPort_AA2SCr")
    set(source "${CMAKE_CURRENT_SOURCE_DIR}/app/manifest/${port}.json")
    set(old_app "DecisionAP")
  elseif(port STREQUAL "PPort_SS2VS")
    set(source "${OSS_ECU}/Exe_scenarioselector/manifest/interface/Exe_scenarioselector/AA/${port}.json")
    set(old_app "Exe_scenarioselector")
  elseif(port STREQUAL "PPort_PV2SCr")
    set(source "${OSS_ECU}/Exe_planningvalidator/manifest/interface/Exe_planningvalidator/AA/${port}.json")
    set(old_app "Exe_planningvalidator")
  else()
    set(source "${OSS_ECU}/Exe_missionplanner/manifest/interface/Exe_missionplanner/AA/${port}.json")
    set(old_app "Exe_missionplanner")
  endif()
  file(READ "${source}" contents)
  string(REPLACE "${old_app}/AA" "DecisionAP/AA" contents "${contents}")
  file(WRITE "${DECISION_GEN}/manifest/interface/DecisionAP/AA/${port}.json" "${contents}")
endforeach()
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/app/manifest/exec" DESTINATION "${DECISION_GEN}/manifest")

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
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/app/manifest/com/someip/CM_P_VS_scenarioselector.json" scenario_cm)
string(REPLACE "192.168.100.92" "192.168.100.93" scenario_cm "${scenario_cm}")
file(WRITE "${CM_GEN}/manifest/com/someip/CM_P_VS_scenarioselector.json" "${scenario_cm}")
foreach(family IN ITEMS Metrics Remaining Turn Hazard)
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/app/manifest/com/someip/CM_P_AA2SCr${family}.json" custom_cm)
  string(REPLACE "192.168.100.92" "192.168.100.93" custom_cm "${custom_cm}")
  file(WRITE "${CM_GEN}/manifest/com/someip/CM_P_${family}.json" "${custom_cm}")
endforeach()

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
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../lib/proxsuite-devel/include/proxsuite/proxqp/sparse/sparse.hpp")
  message(FATAL_ERROR "DecisionAP requires the bundled ProxQP solver")
endif()
set(PLANNING_USE_PROXQP ON CACHE BOOL "Use the bundled ProxQP solver" FORCE)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/.." planning-core)

set(PARA_APP_NAME DecisionAP)
set(PARA_APP_VERSION 0.1.0)
set(PARA_APP_GEN_DIR "${DECISION_GEN}")
Para_App()

add_executable(DecisionAP app/main.cpp app/trajectory_output.cpp
  app/diagnostic_output.cpp app/route_visualization_output.cpp
  app/behavior_signal_output.cpp)
target_compile_features(DecisionAP PRIVATE cxx_std_17)
target_link_libraries(DecisionAP PRIVATE planning_lanelet_bin_adapter pthread)
target_include_directories(DecisionAP PRIVATE "${OSS_ECU}/Exe_missionplanner/include")
target_include_directories(DecisionAP PRIVATE "${OSS_ECU}/Exe_motionvelocityplanner/include")
install(FILES
  "$<TARGET_FILE:lanelet2_core>"
  "$<TARGET_FILE:lanelet2_routing>"
  "$<TARGET_FILE:lanelet2_traffic_rules>"
  DESTINATION lib COMPONENT Standalone)
Para_Target(DecisionAP PARA_LIBS_PRIVATE core log exec com
  PARA_GEN_DIR "${PARA_APP_GEN_DIR}" OUTPUT_NAME decision-ap)
