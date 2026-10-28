# Based on oss-ap/docker/toolchain-aarch64.cmake. Build on x86_64 against
# the aarch64 ROS and Ubuntu sysroot staged at /sysroot.
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

set(XROOT /sysroot)
set(CMAKE_C_FLAGS_INIT "--sysroot=${XROOT}")
set(CMAKE_CXX_FLAGS_INIT "--sysroot=${XROOT}")
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "--sysroot=${XROOT} -Wl,-rpath-link,${XROOT}/usr/lib/aarch64-linux-gnu -Wl,-rpath-link,${XROOT}/opt/ros/humble/lib")
set(CMAKE_SHARED_LINKER_FLAGS_INIT
    "--sysroot=${XROOT} -Wl,-rpath-link,${XROOT}/usr/lib/aarch64-linux-gnu -Wl,-rpath-link,${XROOT}/opt/ros/humble/lib")

set(CMAKE_FIND_ROOT_PATH ${XROOT} ${XROOT}/usr ${XROOT}/opt/ros/humble /opt/para-sdk)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)

set(PYTHON_SOABI cpython-310-aarch64-linux-gnu CACHE STRING "" FORCE)
set(PYTHON_LIBRARY ${XROOT}/usr/lib/aarch64-linux-gnu/libpython3.10.so CACHE FILEPATH "" FORCE)
set(PYTHON_INCLUDE_DIR ${XROOT}/usr/include/python3.10 CACHE PATH "" FORCE)
