cmake_minimum_required(VERSION 3.10)
project(openloong-dyn-control)

IF (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release) 
ENDIF ()
add_compile_options(-std=c++17)
add_compile_definitions(GLFW_INCLUDE_NONE)

# Build only the maintained walking demos and their shared control code.
set(dirEigen "third_party/eigen3")
set(dirGlfw "third_party/glfw")
set(dirPino "third_party/pinocchio")
set(dirJson "third_party/jsoncpp")
set(dirQuill "third_party/quill")
set(dirAligator "third_party/aligator")

set(libURDF "third_party/urdfdom")

set(incFmt "third_party/quill/quill/bundled")
set(incBoost "third_party/boost")
set(incMujoco "third_party/mujoco")
set(incQP "third_party/qpOASES")

set(allDir ${dirEigen} ${dirGlfw} ${dirPino} ${dirJson} ${dirQuill})
set(allLib ${allDir} ${libURDF})
set(allInc ${allDir} ${incFmt} ${incBoost} ${incMujoco} ${incQP})

include_directories(${allInc})
include_directories("${dirAligator}/include")
include_directories("algorithm")
include_directories("common")
include_directories("math")
include_directories("sim_interface")
list(PREPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/shims")

message(${CMAKE_SYSTEM_PROCESSOR})
if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
	list(APPEND allLib "third_party/mujoco/lin_arm64")
	list(APPEND allLib "third_party/qpOASES/lin_arm64")
else()
	list(APPEND allLib "third_party/mujoco/lin_x64")
	list(APPEND allLib "third_party/qpOASES/lin_x64")
endif()
link_directories(${allLib})

set(BUILD_WITH_PINOCCHIO_SUPPORT ON CACHE BOOL "" FORCE)
set(BUILD_WITH_OPENMP_SUPPORT OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_MIMALLOC OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_INTERFACE OFF CACHE BOOL "" FORCE)
set(INSTALL_DOCUMENTATION OFF CACHE BOOL "" FORCE)
set(fmt_DIR "/usr/lib/x86_64-linux-gnu/cmake/fmt" CACHE PATH "" FORCE)
set(pinocchio_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake/shims/pinocchio" CACHE PATH "" FORCE)
add_subdirectory(${dirAligator} EXCLUDE_FROM_ALL)

file(GLOB C_SOURCES *.c)
set(CPP_SOURCES
	algorithm/Eul_W_filter.cpp
	algorithm/StateEst.cpp
	algorithm/foot_placement.cpp
	algorithm/g1_centroidal_nmpc.cpp
	algorithm/g1_kinodynamics_nmpc.cpp
	algorithm/g1_srbd_mpc.cpp
	algorithm/g1_srbd_mpc_rp.cpp
	algorithm/g1_srbd_mpc_rp_enhanced.cpp
	algorithm/gait_scheduler.cpp
	algorithm/joystick_interpreter.cpp
	algorithm/pino_kin_dyn.cpp
	algorithm/priority_tasks.cpp
	algorithm/wbc_bruce_weighted.cpp
	algorithm/wbc_priority.cpp
	common/PVT_ctrl.cpp
	common/data_logger.cpp
	math/LPF_fst.cpp
	math/bezier_1D.cpp
	math/ramp_trajectory.cpp
	math/useful_math.cpp
	sim_interface/GLFW_callbacks.cpp
	sim_interface/MJ_interface.cpp
)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})

#链接库
set(corLibs pinocchio urdfdom_model tinyxml console_bridge jsoncpp quill)
set(simLibs glfw3)#mujoco区分x86/arm的方式不一样，在上面按link目录区分

#根据架构，处理连接库
set(sysCoreLibs)
set(sysSimLibs)
if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
	message("linux arm64架构")
	foreach(lib ${corLibs})
		list(APPEND sysCoreLibs ${lib}_lin_arm64)
	endforeach()
	foreach(lib ${simLibs})
		list(APPEND sysSimLibs ${lib}_lin_arm64)
	endforeach()
	list(APPEND sysSimLibs dl)#arm 需要 -ldl
else()
	message("linux x64架构")
	foreach(lib ${corLibs})
		list(APPEND sysCoreLibs ${lib}_lin_x64)
	endforeach()
	foreach(lib ${simLibs})
		list(APPEND sysSimLibs ${lib}_lin_x64)
	endforeach()
endif()
message(${sysCoreLibs})

#生成控制核心库
add_library(core ${SOURCES})
target_link_libraries(core ${sysCoreLibs} aligator::aligator pthread)


#生成仿真可执行文件
add_executable(walk_wbc demo/walk_wbc.cpp)
target_link_libraries(walk_wbc core mujoco qpOASES ${sysSimLibs} dl)

add_executable(g1_srbd_ik_walk demo/g1_srbd_ik_walk.cpp)
target_link_libraries(g1_srbd_ik_walk core mujoco qpOASES ${sysSimLibs} dl)

add_executable(g1_srbd_ik_walk_rp demo/g1_srbd_ik_walk_rp.cpp)
target_link_libraries(g1_srbd_ik_walk_rp core mujoco qpOASES ${sysSimLibs} dl)

add_executable(g1_srbd_ik_walk_rp_enhanced demo/g1_srbd_ik_walk_rp_enhanced.cpp)
target_link_libraries(g1_srbd_ik_walk_rp_enhanced core mujoco qpOASES ${sysSimLibs} dl)

add_executable(g1_cd_nmpc_ik_stand demo/g1_cd_nmpc_ik_stand.cpp)
target_link_libraries(g1_cd_nmpc_ik_stand core mujoco qpOASES ${sysSimLibs} dl)

add_executable(g1_cd_nmpc_ik_walk demo/g1_cd_nmpc_ik_walk.cpp)
target_link_libraries(g1_cd_nmpc_ik_walk core mujoco qpOASES ${sysSimLibs} dl)

add_executable(g1_cd_nmpc_wbc_walk demo/g1_cd_nmpc_wbc_walk.cpp)
target_link_libraries(g1_cd_nmpc_wbc_walk core mujoco qpOASES ${sysSimLibs} dl)

add_executable(g1_kino_nmpc_id_stand demo/g1_kino_nmpc_id_stand.cpp)
target_link_libraries(g1_kino_nmpc_id_stand core mujoco qpOASES ${sysSimLibs} dl)
