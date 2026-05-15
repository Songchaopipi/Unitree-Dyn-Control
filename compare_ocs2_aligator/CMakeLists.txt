cmake_minimum_required(VERSION 3.16)
project(compare_ocs2_aligator LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

get_filename_component(OPENLOONG_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(WB_HUMANOID_MPC_WS "/home/songchao/OPTControl_env/wb_humanoid_mpc_ws" CACHE PATH
    "Path to wb_humanoid_mpc_ws")

list(PREPEND CMAKE_PREFIX_PATH "${OPENLOONG_ROOT}/cmake/shims")

find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Threads REQUIRED)

add_compile_definitions(
  BOOST_MPL_LIMIT_LIST_SIZE=30
  PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

set(BUILD_WITH_PINOCCHIO_SUPPORT ON CACHE BOOL "" FORCE)
set(BUILD_WITH_OPENMP_SUPPORT OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_MIMALLOC OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_INTERFACE OFF CACHE BOOL "" FORCE)
set(INSTALL_DOCUMENTATION OFF CACHE BOOL "" FORCE)
set(fmt_DIR "/usr/lib/x86_64-linux-gnu/cmake/fmt" CACHE PATH "" FORCE)
set(pinocchio_DIR "${OPENLOONG_ROOT}/cmake/shims/pinocchio" CACHE PATH "" FORCE)
add_subdirectory("${OPENLOONG_ROOT}/third_party/aligator" "${CMAKE_BINARY_DIR}/aligator" EXCLUDE_FROM_ALL)

set(OCS2_CORE_INCLUDE "${WB_HUMANOID_MPC_WS}/third_party/ocs2_core/include")
set(OCS2_THIRDPARTY_INCLUDE "${WB_HUMANOID_MPC_WS}/third_party/ocs2_thirdparty/include")
set(OCS2_CORE_LIB "${WB_HUMANOID_MPC_WS}/third_party/ocs2_core/lib/libocs2_core.a")

if(NOT EXISTS "${OCS2_CORE_LIB}")
  message(FATAL_ERROR "Cannot find OCS2 core library: ${OCS2_CORE_LIB}")
endif()

if(CMAKE_CXX_COMPILER MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(OPENLOONG_LIB_SUFFIX "lin_arm64")
else()
  set(OPENLOONG_LIB_SUFFIX "lin_x64")
endif()
set(OPENLOONG_PINOCCHIO_ARCHIVE "${OPENLOONG_ROOT}/third_party/pinocchio/libpinocchio_${OPENLOONG_LIB_SUFFIX}.a")

link_directories(
  "${OPENLOONG_ROOT}/third_party/pinocchio"
  "${OPENLOONG_ROOT}/third_party/urdfdom"
)

add_executable(compare_derivatives
  src/compare_derivatives.cpp
)

target_include_directories(compare_derivatives PRIVATE
  "${OPENLOONG_ROOT}/third_party/eigen3"
  "${OPENLOONG_ROOT}/third_party/pinocchio"
  "${OPENLOONG_ROOT}/third_party/boost"
  "${OPENLOONG_ROOT}/third_party/quill/quill/bundled"
  "${OCS2_CORE_INCLUDE}"
  "${OCS2_THIRDPARTY_INCLUDE}"
)

target_link_libraries(compare_derivatives PRIVATE
  aligator::aligator
  "${OCS2_CORE_LIB}"
  "${OPENLOONG_PINOCCHIO_ARCHIVE}"
  "urdfdom_model_${OPENLOONG_LIB_SUFFIX}"
  "tinyxml_${OPENLOONG_LIB_SUFFIX}"
  "console_bridge_${OPENLOONG_LIB_SUFFIX}"
  Boost::filesystem
  Boost::system
  Threads::Threads
  dl
)

set_target_properties(compare_derivatives PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build"
)
