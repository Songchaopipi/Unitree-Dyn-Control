cmake_minimum_required(VERSION 3.16)
project(wb_humanoid_centroidal_standalone)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Boost REQUIRED COMPONENTS system filesystem log log_setup thread)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenMP)
find_package(OpenGL REQUIRED)
pkg_check_modules(GLFW3 REQUIRED glfw3)

set(ROS_HUMBLE_ROOT "/opt/ros/humble" CACHE PATH "ROS Humble installation used for Pinocchio/urdfdom ABI compatibility")
list(PREPEND CMAKE_PREFIX_PATH "${ROS_HUMBLE_ROOT}")
set(ENV{PKG_CONFIG_PATH}
  "${ROS_HUMBLE_ROOT}/lib/x86_64-linux-gnu/pkgconfig:$ENV{PKG_CONFIG_PATH}"
)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)

add_compile_definitions(
  BOOST_MPL_LIMIT_LIST_SIZE=30
  PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

add_compile_options(
  -Wall
  -Wextra
  -Wno-unused-parameter
  -Wno-deprecated-declarations
  -Wno-ignored-attributes
  -Wno-invalid-partial-specialization
)

set(PROJECT_THIRD_PARTY "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
set(OCS2_PACKAGES
  ocs2_core
  ocs2_oc
  ocs2_mpc
  ocs2_ddp
  ocs2_qp_solver
  ocs2_sqp
  ocs2_pinocchio_interface
  ocs2_robotic_tools
  ocs2_centroidal_model
)

set(OCS2_INCLUDES "${PROJECT_THIRD_PARTY}/ocs2_thirdparty/include")
foreach(pkg ${OCS2_PACKAGES})
  list(APPEND OCS2_INCLUDES "${PROJECT_THIRD_PARTY}/${pkg}/include")
endforeach()
list(APPEND OCS2_INCLUDES
  "${PROJECT_THIRD_PARTY}/hpipm_catkin/include"
  "${PROJECT_THIRD_PARTY}/blasfeo_catkin/include"
)

set(HUMANOID_INCLUDES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/humanoid_common_mpc/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/humanoid_centroidal_mpc/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/humanoid_wb_mpc/include"
)

set(COMMON_MPC_SOURCES
  src/compat/CentroidalModelLocal.cpp
  src/compat/PinocchioInterfaceLocal.cpp
  src/humanoid_common_mpc/src/contact/ContactCenterPoint.cpp
  src/humanoid_common_mpc/src/contact/ContactPolygon.cpp
  src/humanoid_common_mpc/src/contact/ContactRectangle.cpp
  src/humanoid_common_mpc/src/pinocchio_model/pinocchioUtils.cpp
  src/humanoid_common_mpc/src/pinocchio_model/createPinocchioModel.cpp
  src/humanoid_common_mpc/src/pinocchio_model/DynamicsHelperFunctions.cpp
  src/humanoid_common_mpc/src/common/ModelSettings.cpp
  src/humanoid_common_mpc/src/constraint/JointLimitsSoftConstraint.cpp
  src/humanoid_common_mpc/src/constraint/FootCollisionConstraint.cpp
  src/humanoid_common_mpc/src/constraint/ZeroWrenchConstraint.cpp
  src/humanoid_common_mpc/src/constraint/EndEffectorKinematicsTwistConstraint.cpp
  src/humanoid_common_mpc/src/constraint/FrictionForceConeConstraint.cpp
  src/humanoid_common_mpc/src/constraint/ContactMomentXYConstraintCppAd.cpp
  src/humanoid_common_mpc/src/constraint/EndEffectorKinematicsLinearVelConstraint.cpp
  src/humanoid_common_mpc/src/cost/StateInputQuadraticCost.cpp
  src/humanoid_common_mpc/src/cost/EndEffectorKinematicCostHelpers.cpp
  src/humanoid_common_mpc/src/cost/EndEffectorKinematicsQuadraticCost.cpp
  src/humanoid_common_mpc/src/cost/ExternalTorqueQuadraticCostAD.cpp
  src/humanoid_common_mpc/src/command/TargetTrajectoriesCalculatorBase.cpp
  src/humanoid_common_mpc/src/gait/Gait.cpp
  src/humanoid_common_mpc/src/gait/GaitSchedule.cpp
  src/humanoid_common_mpc/src/gait/ModeSequenceTemplate.cpp
  src/humanoid_common_mpc/src/gait/GaitScheduleUpdater.cpp
  src/humanoid_common_mpc/src/swing_foot_planner/CubicSpline.cpp
  src/humanoid_common_mpc/src/swing_foot_planner/SplineCpg.cpp
  src/humanoid_common_mpc/src/swing_foot_planner/SwingTrajectoryPlanner.cpp
  src/humanoid_common_mpc/src/reference_manager/SwitchedModelReferenceManager.cpp
  src/humanoid_common_mpc/src/reference_manager/ProceduralMpcMotionManager.cpp
  src/humanoid_common_mpc/src/HumanoidCostConstraintFactory.cpp
  src/humanoid_common_mpc/src/HumanoidPreComputation.cpp
  src/humanoid_common_mpc/src/initialization/WeightCompInitializer.cpp
)

add_library(humanoid_common_mpc STATIC ${COMMON_MPC_SOURCES})
target_include_directories(humanoid_common_mpc PUBLIC
  ${HUMANOID_INCLUDES}
  ${OCS2_INCLUDES}
  ${PINOCCHIO_INCLUDE_DIRS}
)

set(CENTROIDAL_MPC_SOURCES
  src/humanoid_centroidal_mpc/src/dynamics/CentroidalDynamicsAD.cpp
  src/humanoid_centroidal_mpc/src/cost/CentroidalMpcEndEffectorFootCost.cpp
  src/humanoid_centroidal_mpc/src/cost/ICPCost.cpp
  src/humanoid_centroidal_mpc/src/constraint/ZeroVelocityConstraintCppAd.cpp
  src/humanoid_centroidal_mpc/src/constraint/NormalVelocityConstraintCppAd.cpp
  src/humanoid_centroidal_mpc/src/constraint/JointMimicKinematicConstraint.cpp
  src/humanoid_centroidal_mpc/src/initialization/CentroidalWeightCompInitializer.cpp
  src/humanoid_centroidal_mpc/src/CentroidalMpcInterface.cpp
  src/humanoid_centroidal_mpc/src/command/CentroidalMpcTargetTrajectoriesCalculator.cpp
)

add_library(humanoid_centroidal_mpc STATIC ${CENTROIDAL_MPC_SOURCES})
target_include_directories(humanoid_centroidal_mpc PUBLIC
  ${HUMANOID_INCLUDES}
  ${OCS2_INCLUDES}
  ${PINOCCHIO_INCLUDE_DIRS}
)
target_link_libraries(humanoid_centroidal_mpc PUBLIC humanoid_common_mpc)

set(WB_MPC_SOURCES
  src/humanoid_wb_mpc/src/dynamics/DynamicsHelperFunctions.cpp
  src/humanoid_wb_mpc/src/dynamics/WBAccelDynamicsAD.cpp
  src/humanoid_wb_mpc/src/end_effector/PinocchioEndEffectorDynamicsCppAd.cpp
  src/humanoid_wb_mpc/src/constraint/EndEffectorDynamicsLinearAccConstraint.cpp
  src/humanoid_wb_mpc/src/constraint/EndEffectorDynamicsAccelerationsConstraint.cpp
  src/humanoid_wb_mpc/src/constraint/ZeroAccelerationConstraintCppAd.cpp
  src/humanoid_wb_mpc/src/constraint/SwingLegVerticalConstraintCppAd.cpp
  src/humanoid_wb_mpc/src/constraint/JointMimicDynamicsConstraint.cpp
  src/humanoid_wb_mpc/src/WBMpcInterface.cpp
  src/humanoid_wb_mpc/src/WBMpcPreComputation.cpp
  src/humanoid_wb_mpc/src/cost/EndEffectorDynamicsCostHelpers.cpp
  src/humanoid_wb_mpc/src/cost/EndEffectorDynamicsQuadraticCost.cpp
  src/humanoid_wb_mpc/src/cost/EndEffectorDynamicsFootCost.cpp
  src/humanoid_wb_mpc/src/cost/JointTorqueCostCppAd.cpp
  src/humanoid_wb_mpc/src/command/WBMpcTargetTrajectoriesCalculator.cpp
  src/humanoid_wb_mpc/src/mrt/WBMpcMrtJointController.cpp
)

add_library(humanoid_wb_mpc STATIC ${WB_MPC_SOURCES})
target_include_directories(humanoid_wb_mpc PUBLIC
  ${HUMANOID_INCLUDES}
  ${OCS2_INCLUDES}
  ${PINOCCHIO_INCLUDE_DIRS}
)
target_link_libraries(humanoid_wb_mpc PUBLIC humanoid_common_mpc)

set(OCS2_LIBS
  "${PROJECT_THIRD_PARTY}/ocs2_sqp/lib/libocs2_sqp.a"
  "${PROJECT_THIRD_PARTY}/ocs2_qp_solver/lib/libocs2_qp_solver.a"
  "${PROJECT_THIRD_PARTY}/ocs2_mpc/lib/libocs2_mpc.a"
  "${PROJECT_THIRD_PARTY}/ocs2_ddp/lib/libocs2_ddp.a"
  "${PROJECT_THIRD_PARTY}/ocs2_oc/lib/libocs2_oc.a"
  "${PROJECT_THIRD_PARTY}/ocs2_robotic_tools/lib/libocs2_robotic_tools.a"
  "${PROJECT_THIRD_PARTY}/ocs2_centroidal_model/lib/libocs2_centroidal_model.a"
  "${PROJECT_THIRD_PARTY}/ocs2_pinocchio_interface/lib/libocs2_pinocchio_interface.a"
  "${PROJECT_THIRD_PARTY}/ocs2_core/lib/libocs2_core.a"
)

add_executable(centroidal_mpc_mujoco
  src/standalone/centroidal_mpc_mujoco.cpp
  src/standalone/centroidal_mpc_mujoco_utils.cpp
  src/standalone/mj_sim_interface.cpp
)
target_compile_definitions(centroidal_mpc_mujoco PRIVATE
  WB_HUMANOID_MPC_ROOT="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_include_directories(centroidal_mpc_mujoco PRIVATE
  ${HUMANOID_INCLUDES}
  ${OCS2_INCLUDES}
  ${PINOCCHIO_INCLUDE_DIRS}
  ${GLFW3_INCLUDE_DIRS}
  "${PROJECT_THIRD_PARTY}/mujoco/include"
)
target_link_directories(centroidal_mpc_mujoco PRIVATE
  ${PINOCCHIO_LIBRARY_DIRS}
  ${GLFW3_LIBRARY_DIRS}
  "${PROJECT_THIRD_PARTY}/mujoco/lib"
  "${PROJECT_THIRD_PARTY}/hpipm_catkin/lib"
  "${PROJECT_THIRD_PARTY}/blasfeo_catkin/lib"
)
target_link_libraries(centroidal_mpc_mujoco PRIVATE
  -Wl,--start-group
  humanoid_centroidal_mpc
  humanoid_common_mpc
  ${OCS2_LIBS}
  -Wl,--no-as-needed
  "${PROJECT_THIRD_PARTY}/hpipm_catkin/lib/libhpipm_catkin.so"
  "${PROJECT_THIRD_PARTY}/hpipm_catkin/lib/libhpipm.so"
  "${PROJECT_THIRD_PARTY}/blasfeo_catkin/lib/libblasfeo.so"
  -Wl,--as-needed
  -Wl,--end-group
  ${PINOCCHIO_LIBRARIES}
  "${PROJECT_THIRD_PARTY}/mujoco/lib/libmujoco.so"
  ${GLFW3_LIBRARIES}
  OpenGL::GL
  ${Boost_LIBRARIES}
  Threads::Threads
  dl
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(centroidal_mpc_mujoco PRIVATE OpenMP::OpenMP_CXX)
endif()

set_target_properties(centroidal_mpc_mujoco PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build"
  BUILD_RPATH "${PROJECT_THIRD_PARTY}/mujoco/lib;${PROJECT_THIRD_PARTY}/hpipm_catkin/lib;${PROJECT_THIRD_PARTY}/blasfeo_catkin/lib;${PINOCCHIO_LIBRARY_DIRS}"
)

add_executable(wb_mpc_mujoco
  src/standalone/wb_mpc_mujoco.cpp
  src/standalone/wb_mpc_mujoco_utils.cpp
  src/standalone/mj_sim_interface.cpp
)
target_compile_definitions(wb_mpc_mujoco PRIVATE
  WB_HUMANOID_MPC_ROOT="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_include_directories(wb_mpc_mujoco PRIVATE
  ${HUMANOID_INCLUDES}
  ${OCS2_INCLUDES}
  ${PINOCCHIO_INCLUDE_DIRS}
  ${GLFW3_INCLUDE_DIRS}
  "${PROJECT_THIRD_PARTY}/mujoco/include"
)
target_link_directories(wb_mpc_mujoco PRIVATE
  ${PINOCCHIO_LIBRARY_DIRS}
  ${GLFW3_LIBRARY_DIRS}
  "${PROJECT_THIRD_PARTY}/mujoco/lib"
  "${PROJECT_THIRD_PARTY}/hpipm_catkin/lib"
  "${PROJECT_THIRD_PARTY}/blasfeo_catkin/lib"
)
target_link_libraries(wb_mpc_mujoco PRIVATE
  -Wl,--start-group
  humanoid_wb_mpc
  humanoid_common_mpc
  ${OCS2_LIBS}
  -Wl,--no-as-needed
  "${PROJECT_THIRD_PARTY}/hpipm_catkin/lib/libhpipm_catkin.so"
  "${PROJECT_THIRD_PARTY}/hpipm_catkin/lib/libhpipm.so"
  "${PROJECT_THIRD_PARTY}/blasfeo_catkin/lib/libblasfeo.so"
  -Wl,--as-needed
  -Wl,--end-group
  ${PINOCCHIO_LIBRARIES}
  "${PROJECT_THIRD_PARTY}/mujoco/lib/libmujoco.so"
  ${GLFW3_LIBRARIES}
  OpenGL::GL
  ${Boost_LIBRARIES}
  Threads::Threads
  dl
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(wb_mpc_mujoco PRIVATE OpenMP::OpenMP_CXX)
endif()

set_target_properties(wb_mpc_mujoco PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build"
  BUILD_RPATH "${PROJECT_THIRD_PARTY}/mujoco/lib;${PROJECT_THIRD_PARTY}/hpipm_catkin/lib;${PROJECT_THIRD_PARTY}/blasfeo_catkin/lib;${PINOCCHIO_LIBRARY_DIRS}"
)
