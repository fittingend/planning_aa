# Prime CMake's library search before ROS package configs use find_library().
# Under arm64 QEMU, the first lookup inside builtin_interfacesConfig.cmake can
# report an existing library as missing unless a lookup has already succeeded.
find_library(_planning_aa_ros_builtin_interfaces_generator_c
  NAMES builtin_interfaces__rosidl_generator_c
  PATHS /opt/ros/humble/lib
  NO_DEFAULT_PATH)
if(NOT _planning_aa_ros_builtin_interfaces_generator_c)
  message(FATAL_ERROR "ROS builtin_interfaces library is missing")
endif()
