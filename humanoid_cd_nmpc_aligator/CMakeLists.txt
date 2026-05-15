cmake_minimum_required(VERSION 3.16)
project(humanoid_cd_nmpc_aligator LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(OPENLOONG_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." CACHE PATH
    "Path to OpenLoong-Dyn-Control. Used for bundled aligator, Eigen, Pinocchio, and shims.")
get_filename_component(OPENLOONG_ROOT "${OPENLOONG_ROOT}" ABSOLUTE)
set(WB_HUMANOID_MPC_WS "/home/songchao/OPTControl_env/wb_humanoid_mpc_ws" CACHE PATH
    "Path to wb_humanoid_mpc_ws. Used for G1 config and URDF defaults.")
set(CENTROIDAL_NMPC_ROOT "${OPENLOONG_ROOT}/third_party/centroidal_nmpc" CACHE PATH
    "Path to the prebuilt OCS2/humanoid centroidal MPC libraries bundled in OpenLoong.")

list(PREPEND CMAKE_PREFIX_PATH "${OPENLOONG_ROOT}/cmake/shims")

set(Boost_NO_BOOST_CMAKE ON CACHE BOOL "" FORCE)
set(Boost_NO_SYSTEM_PATHS ON CACHE BOOL "" FORCE)
set(BOOST_ROOT "/usr" CACHE PATH "" FORCE)
set(BOOST_INCLUDEDIR "/usr/include" CACHE PATH "" FORCE)
set(BOOST_LIBRARYDIR "/usr/lib/x86_64-linux-gnu" CACHE PATH "" FORCE)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)

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

if(CMAKE_CXX_COMPILER MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(OPENLOONG_LIB_SUFFIX "lin_arm64")
else()
  set(OPENLOONG_LIB_SUFFIX "lin_x64")
endif()

set(OPENLOONG_PINOCCHIO_ARCHIVE "${OPENLOONG_ROOT}/third_party/pinocchio/libpinocchio_${OPENLOONG_LIB_SUFFIX}.a")
if(NOT EXISTS "${OPENLOONG_PINOCCHIO_ARCHIVE}")
  message(FATAL_ERROR "Cannot find OpenLoong Pinocchio archive: ${OPENLOONG_PINOCCHIO_ARCHIVE}")
endif()

set(SYSTEM_PINOCCHIO_DEFAULT "/usr/local/lib/libpinocchio_default.so" CACHE FILEPATH
    "System Pinocchio default library used by the prebuilt OCS2 centroidal libraries.")
set(SYSTEM_PINOCCHIO_PARSERS "/usr/local/lib/libpinocchio_parsers.so" CACHE FILEPATH
    "System Pinocchio parser library used by the prebuilt OCS2 centroidal libraries.")
set(SYSTEM_URDFDOM_MODEL "/usr/lib/x86_64-linux-gnu/liburdfdom_model.so" CACHE FILEPATH "")
set(SYSTEM_TINYXML "/usr/lib/x86_64-linux-gnu/libtinyxml.so" CACHE FILEPATH "")
set(SYSTEM_CONSOLE_BRIDGE "/usr/lib/x86_64-linux-gnu/libconsole_bridge.so" CACHE FILEPATH "")

set(OCS2_INCLUDE_DIRS
  "${CENTROIDAL_NMPC_ROOT}/humanoid_centroidal_mpc/include"
  "${CENTROIDAL_NMPC_ROOT}/humanoid_common_mpc/include"
  "${CENTROIDAL_NMPC_ROOT}/ocs2_centroidal_model/include"
  "${CENTROIDAL_NMPC_ROOT}/ocs2_core/include"
  "${CENTROIDAL_NMPC_ROOT}/ocs2_ddp/include"
  "${CENTROIDAL_NMPC_ROOT}/ocs2_mpc/include"
  "${CENTROIDAL_NMPC_ROOT}/ocs2_oc/include"
  "${CENTROIDAL_NMPC_ROOT}/ocs2_pinocchio_interface/include"
  "${CENTROIDAL_NMPC_ROOT}/ocs2_qp_solver/include"
  "${CENTROIDAL_NMPC_ROOT}/ocs2_robotic_tools/include"
  "${CENTROIDAL_NMPC_ROOT}/ocs2_sqp/include"
  "${CENTROIDAL_NMPC_ROOT}/ocs2_thirdparty/include"
  "${CENTROIDAL_NMPC_ROOT}/blasfeo_catkin/include"
  "${CENTROIDAL_NMPC_ROOT}/hpipm_catkin/include"
)

set(OCS2_STATIC_LIBS
  "${CENTROIDAL_NMPC_ROOT}/humanoid_centroidal_mpc/lib/libhumanoid_centroidal_mpc.a"
  "${CENTROIDAL_NMPC_ROOT}/humanoid_common_mpc/lib/libhumanoid_common_mpc.a"
  "${CENTROIDAL_NMPC_ROOT}/ocs2_centroidal_model/lib/libocs2_centroidal_model.a"
  "${CENTROIDAL_NMPC_ROOT}/ocs2_pinocchio_interface/lib/libocs2_pinocchio_interface.a"
  "${CENTROIDAL_NMPC_ROOT}/ocs2_robotic_tools/lib/libocs2_robotic_tools.a"
  "${CENTROIDAL_NMPC_ROOT}/ocs2_sqp/lib/libocs2_sqp.a"
  "${CENTROIDAL_NMPC_ROOT}/ocs2_ddp/lib/libocs2_ddp.a"
  "${CENTROIDAL_NMPC_ROOT}/ocs2_oc/lib/libocs2_oc.a"
  "${CENTROIDAL_NMPC_ROOT}/ocs2_mpc/lib/libocs2_mpc.a"
  "${CENTROIDAL_NMPC_ROOT}/ocs2_qp_solver/lib/libocs2_qp_solver.a"
  "${CENTROIDAL_NMPC_ROOT}/ocs2_core/lib/libocs2_core.a"
)

foreach(lib ${OCS2_STATIC_LIBS})
  if(NOT EXISTS "${lib}")
    message(FATAL_ERROR "Cannot find required OCS2/humanoid library: ${lib}")
  endif()
endforeach()

set(OCS2_SHARED_LIBS
  "${CENTROIDAL_NMPC_ROOT}/hpipm_catkin/lib/libhpipm.so"
  "${CENTROIDAL_NMPC_ROOT}/hpipm_catkin/lib/libhpipm_catkin.so"
  "${CENTROIDAL_NMPC_ROOT}/blasfeo_catkin/lib/libblasfeo.so"
)

link_directories(
  "${OPENLOONG_ROOT}/third_party/pinocchio"
  "${OPENLOONG_ROOT}/third_party/urdfdom"
)

add_executable(proxddp_centroidal_layer1
  src/proxddp_centroidal_layer1.cpp
)

target_include_directories(proxddp_centroidal_layer1 PRIVATE
  "${OPENLOONG_ROOT}/third_party/eigen3"
  "/usr/local/include"
  "${OPENLOONG_ROOT}/third_party/boost"
  "${OPENLOONG_ROOT}/third_party/quill/quill/bundled"
  ${OCS2_INCLUDE_DIRS}
)

target_link_libraries(proxddp_centroidal_layer1 PRIVATE
  aligator::aligator
  -Wl,--start-group
  ${OCS2_STATIC_LIBS}
  -Wl,--end-group
  -Wl,--no-as-needed
  ${OCS2_SHARED_LIBS}
  -Wl,--as-needed
  "${SYSTEM_PINOCCHIO_PARSERS}"
  "${SYSTEM_PINOCCHIO_DEFAULT}"
  "${SYSTEM_URDFDOM_MODEL}"
  "${SYSTEM_TINYXML}"
  "${SYSTEM_CONSOLE_BRIDGE}"
  Boost::filesystem
  Boost::system
  Threads::Threads
  OpenMP::OpenMP_CXX
  dl
)

target_compile_definitions(proxddp_centroidal_layer1 PRIVATE
  DEFAULT_WB_HUMANOID_MPC_WS="${WB_HUMANOID_MPC_WS}"
)

set_target_properties(proxddp_centroidal_layer1 PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build"
  BUILD_RPATH "${CENTROIDAL_NMPC_ROOT}/hpipm_catkin/lib;${CENTROIDAL_NMPC_ROOT}/blasfeo_catkin/lib"
)
